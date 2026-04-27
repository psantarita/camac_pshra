"""
MULTI-MODULE CAMAC DAQ (Dual ADC + TDC) - POLLED MODE
=========================================================
Slot 17: Gamma detectors (Phillips 7164 ADC)
Slot 19: Neutron wall   (Phillips 7164 ADC)
Slot 21: TDC (Timing)   (Phillips 7186 TDC)

CHANNEL NUMBERING — CRITICAL:
  Phillips 7164 front panel labels channels 1-16.
  CAMAC addressing is 0-15.  Front panel ch N = CAMAC address N-1.

  Example: signal on front panel ch5 → CAMAC address 4 in software.

  The GUI and .npy files always use CAMAC addresses (0-based).
  To convert: front_panel_channel = camac_address + 1

HARDWARE NOTES (Phillips 7164):
  - CAMAC A=15 (front panel ch16) does NOT appear in F(4)A(0) sparse reads.
    Use channels 0-14 (front panel 1-15) only.
  - Quad groups {0-3},{4-7},{8-11},{12-15} share internal circuitry.
    Large signals couple weakly (~1%) across group boundaries.
    LOWER_THRESHOLD=100 eliminates these artefacts since pedestals sit at ~22.
"""

import ctypes
import datetime
import gc
import math
import os
import queue
import signal
import sys
import threading
import time
import tkinter as tk
from collections import deque

# ==================== CONFIGURATION ====================
DLL_PATH = r"C:\Windows\SysWOW64\libxxusb.dll"
SERIAL = b"CC0284"

# Channels 0-14 = front panel 1-15.
# Channel 15 (front panel 16) omitted — not returned by sparse read on this module.
ADC_GAMMA = {"slot": 17, "channels": list(range(16)), "name": "Gamma Detectors"}
ADC_NEUTRON = {"slot": 19, "channels": list(range(16)), "name": "Neutron Wall"}
TDC_MAIN = {"slot": 21, "channels": list(range(2)), "name": "Main Timing"}

# FP_OFFSET: add to CAMAC address to get front panel channel number displayed in GUI
FP_OFFSET = 1

ENABLE_TDC = False

# List-mode: CC-USB executes stack autonomously on every LAM trigger.
# Dead time drops from ~420 µs to ~4 µs (pure CAMAC execution time).
# NOTE: ch15 (front panel 16) is not available in list mode — the 7164
# sparse read omits CAMAC A=15 and there is no slot in the autonomous
# stack for a direct read. Use polled mode if ch15 is needed.
ENABLE_LIST_MODE = True
LIST_MODE_BUF_BYTES = 65536  # 64 kB — room for multi-event buffers
LIST_MODE_TIMEOUT_MS = 500  # longer timeout to collect full buffers

# Polon 420A scaler — slot 22, channel 0 = gate pulse (real events produced).
# Counts freely after F(9) clear, no enable needed.
# In list mode the acquisition thread never calls CAMAC_read, so the GUI
# thread can read the scaler without a lock.
SCALER = {"slot": 22, "channel": 0, "name": "Polon 420A"}
ENABLE_SCALER = False

# LOWER_THRESHOLD must be above the ADC pedestal floor (~22 counts).
# 100 is safe — real gamma peaks sit hundreds of channels above pedestal.
# Raising this eliminates quad-group coupling artefacts completely.
LOWER_THRESHOLD = 100
UPPER_THRESHOLD_ADC = 4080
UPPER_THRESHOLD_TDC = 4080

NBINS = 512
VMIN = 0
VMAX = 4095

X_TICKS = [0, 512, 1024, 1536, 2048, 2560, 3072, 3584, 4096]

DISPLAY_UPDATE_INTERVAL_MS = 100
MAX_EVENTS_BETWEEN_DISPLAYS = 200000

AUTO_SAVE_ENABLED = True
AUTO_SAVE_INTERVAL = 100000  # 100k events (~1.4M hits) — safe for 32-bit Python
last_auto_save_count = 0
auto_save_file_counter = 0

ENABLE_LIVE_DISPLAY = False
# =======================================================

dll = None
hdev = None
running = False
acquisition_thread = None
event_queue = queue.Queue(maxsize=500000)

gamma_data = [deque(maxlen=1000000) for _ in range(16)]
neutron_data = [deque(maxlen=1000000) for _ in range(16)]
tdc_data = [deque(maxlen=1000000) for _ in range(16)]

MAX_EVENTS_IN_MEMORY = 500000
event_list = deque(maxlen=MAX_EVENTS_IN_MEMORY)
events_to_save = []

pre_built_stack = None
t0 = None
event_count = 0
trigger_count = 0
read_cycles = 0
poll_cycles = 0
total_sparse_reads = 0
log_scale = False

scaler_count_total = 0  # cumulative counts since run start
scaler_count_last = 0  # last raw reading for overflow detection

readout_times = deque(maxlen=1000)
measured_readout_time = 500e-6
TAU_MIN_SAMPLES = 20

DT_LOG_INTERVAL_S = 5.0
_last_dt_log_time = 0.0
_dt_log_file = None

root = None
gamma_canvas = None
neutron_canvas_grid = None
tdc_canvas = None
info_label = None
deadtime_label = None
scaler_label = None
toggle_display_btn = None
clear_display_btn = None
gamma_channel_selector = None
tdc_channel_selector = None
current_gamma_channel = 4  # CAMAC A=4 = front panel ch5
current_tdc_channel = 0

gamma_canvas_width = 800
gamma_canvas_height = 400
tdc_canvas_width = 800
tdc_canvas_height = 300
neutron_grid_size = 4
neutron_cell_width = 200
neutron_cell_height = 150


# ==================== HARDWARE ====================


def disable_adc_pedestal(slot):
    try:
        data = ctypes.c_long(0)
        q = ctypes.c_int(0)
        x = ctypes.c_int(0)
        dll.CAMAC_read(
            hdev, slot, 0, 0, ctypes.byref(data), ctypes.byref(q), ctypes.byref(x)
        )
        if q.value:
            current = data.value
            new_value = current & 0xFFFE
            if current != new_value:
                dll.CAMAC_write.restype = ctypes.c_short
                dll.CAMAC_write.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_long,
                    ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_int),
                ]
                dll.CAMAC_write(
                    hdev, slot, 0, 16, new_value, ctypes.byref(q), ctypes.byref(x)
                )
                if q.value:
                    print(f"  [OK]   ADC N={slot}: Pedestal DISABLED")
                else:
                    print(f"  [WARN] ADC N={slot}: Failed to disable pedestal")
            else:
                print(f"  [OK]   ADC N={slot}: Pedestal already disabled")
    except Exception as e:
        print(f"  [ERROR] ADC N={slot} pedestal: {e}")


def open_ccusb():
    global dll, hdev, pre_built_stack

    dll = ctypes.WinDLL(DLL_PATH)
    dll.xxusb_serial_open.restype = ctypes.c_void_p
    dll.xxusb_serial_open.argtypes = [ctypes.c_char_p]
    dll.xxusb_device_close.restype = ctypes.c_short
    dll.xxusb_device_close.argtypes = [ctypes.c_void_p]
    dll.CAMAC_read.restype = ctypes.c_short
    dll.CAMAC_read.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_long),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    dll.CAMAC_C.restype = ctypes.c_short
    dll.CAMAC_C.argtypes = [ctypes.c_void_p]
    dll.CAMAC_Z.restype = ctypes.c_short
    dll.CAMAC_Z.argtypes = [ctypes.c_void_p]
    dll.xxusb_stack_execute.restype = ctypes.c_short
    dll.xxusb_stack_execute.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_long)]
    dll.xxusb_stack_write.restype = ctypes.c_short
    dll.xxusb_stack_write.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_long),
    ]
    dll.xxusb_stack_read.restype = ctypes.c_short
    dll.xxusb_stack_read.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_long),
    ]
    dll.xxusb_bulk_write.restype = ctypes.c_int
    dll.xxusb_bulk_write.argtypes = [
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_int,
    ]
    dll.xxusb_bulk_read.restype = ctypes.c_int
    dll.xxusb_bulk_read.argtypes = [
        ctypes.c_void_p,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_int,
    ]

    hdev = dll.xxusb_serial_open(SERIAL)
    if not hdev:
        raise RuntimeError("Cannot open CC-USB")

    # Stop any residual list-mode from a previous session before C/Z.
    # xxusb_stack_write returns -116 if Action Register is still set.
    import struct as _struct

    _pkt = _struct.pack("<3H", 5, 0, 0)  # Action Register = 0
    dll.xxusb_bulk_write(hdev, _pkt, len(_pkt), 500)
    _q = ctypes.c_int(0)
    _x = ctypes.c_int(0)
    dll.CAMAC_write(
        hdev, 25, 9, 16, ctypes.c_long(0), ctypes.byref(_q), ctypes.byref(_x)
    )  # LAM mask = 0
    dll.CAMAC_write(
        hdev, 25, 1, 16, ctypes.c_long(0), ctypes.byref(_q), ctypes.byref(_x)
    )  # Global Mode = 0

    dll.CAMAC_C(hdev)
    time.sleep(0.2)
    dll.CAMAC_Z(hdev)
    time.sleep(0.2)

    print("Disabling ADC pedestal correction...")
    disable_adc_pedestal(ADC_GAMMA["slot"])
    disable_adc_pedestal(ADC_NEUTRON["slot"])

    if ENABLE_SCALER:
        scaler_clear()
        print(f"  [OK]   Scaler N={SCALER['slot']} cleared")

    # Enable LAM on gamma ADC — required for list mode trigger.
    # F(19)A(0): set Control Register LAM enable bit (0x0004)
    # F(26)A(0): enable LAM on S1 strobe
    # Both are control-class functions — must use CAMAC_read per CC-USB §4.5
    # Q=0 is normal for Phillips 7164 on control functions.
    print("Enabling LAM on ADCs...")
    for slot in [ADC_GAMMA["slot"], ADC_NEUTRON["slot"]]:
        _data = ctypes.c_long(0x0004)
        _q = ctypes.c_int(0)
        _x = ctypes.c_int(0)
        dll.CAMAC_read(
            hdev, slot, 0, 19, ctypes.byref(_data), ctypes.byref(_q), ctypes.byref(_x)
        )
        _data.value = 0
        dll.CAMAC_read(
            hdev, slot, 0, 26, ctypes.byref(_data), ctypes.byref(_q), ctypes.byref(_x)
        )
        print(f"  [OK]   ADC N={slot}: LAM enabled")

    if ENABLE_TDC:
        # Enable LAM on TDC — F(26) enables LAM on S1 strobe (same as ADC)
        # Also clear the TDC with F(9) to start fresh
        _data = ctypes.c_long(0)
        _q = ctypes.c_int(0)
        _x = ctypes.c_int(0)
        dll.CAMAC_read(
            hdev,
            TDC_MAIN["slot"],
            0,
            9,
            ctypes.byref(_data),
            ctypes.byref(_q),
            ctypes.byref(_x),
        )
        dll.CAMAC_read(
            hdev,
            TDC_MAIN["slot"],
            0,
            26,
            ctypes.byref(_data),
            ctypes.byref(_q),
            ctypes.byref(_x),
        )
        print(f"  [OK]   TDC N={TDC_MAIN['slot']}: cleared and LAM enabled")

    N_COMMANDS = 16
    pre_built_stack = (ctypes.c_long * (N_COMMANDS + 10))()

    if ENABLE_LIST_MODE:
        print("Configuring list-mode acquisition...")
        listmode_init()

    print("[OK] DAQ Hardware Ready")
    print(f"     Mode: {'LIST MODE' if ENABLE_LIST_MODE else 'POLLED'}")
    print(f"     LOWER_THRESHOLD = {LOWER_THRESHOLD}")
    print(f"     Active gamma channels (CAMAC): {ADC_GAMMA['channels']}")
    print(f"     = Front panel: {[c + FP_OFFSET for c in ADC_GAMMA['channels']]}")


def close_ccusb():
    global dll, hdev
    if dll and hdev:
        dll.xxusb_device_close(hdev)
        print("[OK] CC-USB closed.")
    dll = hdev = None


# ==================== SCALER ====================


def scaler_clear():
    """Clear scaler. Polon 420A only — called during init before list mode starts."""
    if not ENABLE_SCALER:
        return
    try:
        data = ctypes.c_long(0)
        q = ctypes.c_int(0)
        x = ctypes.c_int(0)
        dll.CAMAC_read(
            hdev,
            SCALER["slot"],
            0,
            9,
            ctypes.byref(data),
            ctypes.byref(q),
            ctypes.byref(x),
        )
    except Exception as e:
        print(f"[SCALER] Clear error: {e}")


def scaler_read():
    """Read current scaler count.
    List mode: reads CC-USB internal SCLR_A (N=25, A=8) via CAMAC_read —
               this internal register is readable even when Action Register=1.
    Polled mode: reads Polon 420A directly.
    Returns count as integer, -1 on failure."""
    if not ENABLE_SCALER:
        return -1
    try:
        data = ctypes.c_long(0)
        q = ctypes.c_int(0)
        x = ctypes.c_int(0)
        if ENABLE_LIST_MODE:
            # CC-USB internal SCLR_A counts NIM input I1 pulses.
            # Connect gate signal to NIM I1 for dead time cross-check.
            dll.CAMAC_read(
                hdev, 25, 8, 0, ctypes.byref(data), ctypes.byref(q), ctypes.byref(x)
            )
            return data.value & 0xFFFFFF
        else:
            dll.CAMAC_read(
                hdev,
                SCALER["slot"],
                SCALER["channel"],
                0,
                ctypes.byref(data),
                ctypes.byref(q),
                ctypes.byref(x),
            )
            return data.value & 0xFFFFFF if q.value else -1
    except Exception:
        return -1


def scaler_tick():
    """Accumulate scaler count.
    In list mode: CC-USB blocks ALL EASY-CAMAC reads (including internal
    registers) while Action Register is set.  Scaler is unavailable.
    In polled mode: reads the Polon 420A scaler directly."""
    global scaler_count_total, scaler_count_last
    if not ENABLE_SCALER or not running:
        return
    if ENABLE_LIST_MODE:
        return  # ALL EASY-CAMAC blocked during autonomous acquisition
    raw = scaler_read()
    if raw < 0:
        return
    if raw < scaler_count_last:
        scaler_count_total += (0xFFFFFF - scaler_count_last) + raw + 1
    else:
        scaler_count_total += raw - scaler_count_last
    scaler_count_last = raw


# ==================== LIST MODE ====================


def listmode_init():
    """
    Q-stop stack: drains the FULL sparse FIFO from each module per trigger.

    Stack structure (11 words):
      ADC_GAMMA  Q-stop  (3 words)
      N25 marker         (1 word)
      ADC_NEUTRON Q-stop (3 words)
      N25 marker         (1 word)
      TDC Q-stop         (3 words)  — only if ENABLE_TDC

    The Phillips 7186 TDC F(4)A(0) sparse read automatically clears
    Hit Register bits as channels are read.  When the last channel is
    read (Q=0), LAM resets.  No separate F(11)A(3) clear needed.

    CC-USB complex command format (manual §4.5):
      Word 1: 0x8000 | (F + 32*A + 512*N)     — continuation bit set
      Word 2: 0x8010                            — C=1 (max-count follows), QS=1
      Word 3: max repeat count (20, safe margin above 16 channels)
    """
    import struct

    QS_OPT = 0x8010  # C=1 (continuation), QS bit 4 = 1
    QS_MAX = 20  # safety limit (> 16 ch + possible duplicate)

    w1_gamma = 0x8000 | build_stack_command(ADC_GAMMA["slot"], 0, 4)
    w_mark = build_stack_command(25, 0, 0)
    w1_neut = 0x8000 | build_stack_command(ADC_NEUTRON["slot"], 0, 4)
    w1_tdc = 0x8000 | build_stack_command(TDC_MAIN["slot"], 0, 4)

    if ENABLE_TDC:
        N_STACK = 11
        stack = (ctypes.c_long * (N_STACK + 2))()
        stack[0] = N_STACK
        stack[1] = w1_gamma  # F(4)A(0) ADC_GAMMA + continuation
        stack[2] = QS_OPT
        stack[3] = QS_MAX
        stack[4] = w_mark  # F(0)A(0)N(25) marker 1
        stack[5] = w1_neut  # F(4)A(0) ADC_NEUTRON + continuation
        stack[6] = QS_OPT
        stack[7] = QS_MAX
        stack[8] = w_mark  # F(0)A(0)N(25) marker 2
        stack[9] = w1_tdc  # F(4)A(0) TDC + continuation
        stack[10] = QS_OPT
        stack[11] = QS_MAX
        desc = (
            f"Gamma_N{ADC_GAMMA['slot']}_Qstop + marker + "
            f"Neutron_N{ADC_NEUTRON['slot']}_Qstop + marker + "
            f"TDC_N{TDC_MAIN['slot']}_Qstop"
        )
    else:
        N_STACK = 7
        stack = (ctypes.c_long * (N_STACK + 2))()
        stack[0] = N_STACK
        stack[1] = w1_gamma
        stack[2] = QS_OPT
        stack[3] = QS_MAX
        stack[4] = w_mark
        stack[5] = w1_neut
        stack[6] = QS_OPT
        stack[7] = QS_MAX
        desc = (
            f"Gamma_N{ADC_GAMMA['slot']}_Qstop + marker + "
            f"Neutron_N{ADC_NEUTRON['slot']}_Qstop"
        )

    ret = dll.xxusb_stack_write(hdev, 2, stack)
    if ret < 0:
        raise RuntimeError(f"xxusb_stack_write failed: {ret}")
    print(f"  [OK]   Stack: {desc} ({N_STACK} words, ret={ret})")

    # Verify readback
    readback = (ctypes.c_long * (N_STACK + 4))()
    dll.xxusb_stack_read(hdev, 2, readback)
    rb_hex = " ".join(f"0x{readback[i]:04X}" for i in range(1, N_STACK + 1))
    print(f"  [OK]   Readback: {rb_hex}")

    # --- LAM mask: trigger on ADC slots having data ---
    # TDC LAM is NOT in the mask — the ADC gate triggers the stack,
    # and by the time the stack reaches the TDC (~100 µs later),
    # conversion (7.2 µs) is long done.  If TDC has no data,
    # Q-stop returns Q=0 immediately with zero words — no harm.
    lam_mask = (1 << (ADC_GAMMA["slot"] - 1)) | (1 << (ADC_NEUTRON["slot"] - 1))
    q = ctypes.c_int(0)
    x = ctypes.c_int(0)
    dll.CAMAC_write(
        hdev, 25, 9, 16, ctypes.c_long(lam_mask), ctypes.byref(q), ctypes.byref(x)
    )
    slots_str = f"slots {ADC_GAMMA['slot']}+{ADC_NEUTRON['slot']}"
    print(f"  [OK]   LAM mask: 0x{lam_mask:06X} ({slots_str})")

    GLOBAL_MODE_MULTIEVENT = 0x0104
    BUFFER_TIMEOUT_TICKS = 200

    dll.CAMAC_write(
        hdev,
        25,
        1,
        16,
        ctypes.c_long(GLOBAL_MODE_MULTIEVENT),
        ctypes.byref(q),
        ctypes.byref(x),
    )
    print(
        f"  [OK]   Global Mode: 0x{GLOBAL_MODE_MULTIEVENT:04X} (multi-event, 512-word buffers)"
    )

    dll.CAMAC_write(
        hdev,
        25,
        2,
        16,
        ctypes.c_long(BUFFER_TIMEOUT_TICKS),
        ctypes.byref(q),
        ctypes.byref(x),
    )
    print(
        f"  [OK]   Buffer timeout: {BUFFER_TIMEOUT_TICKS} ticks ({BUFFER_TIMEOUT_TICKS * 12.5 / 1000:.1f} ms)"
    )

    pkt = struct.pack("<3H", 5, 0, 1)
    ret = dll.xxusb_bulk_write(hdev, pkt, len(pkt), 2000)
    print(f"  [OK]   List-mode STARTED (ret={ret})")


def listmode_stop():
    """Stop list-mode using the exact sequence that reliably resets the CC-USB.
    Derived from empirical testing — order matters."""
    import struct

    try:
        # 1. Kill autonomous acquisition in FPGA
        # CRITICAL: [5, 0, 0] = register 0 (Action Register) = 0 (stop)
        pkt = struct.pack("<3H", 5, 0, 0)
        dll.xxusb_bulk_write(hdev, pkt, len(pkt), 2000)
        time.sleep(0.1)

        # 2. Clear LAM mask and Global Mode
        q = ctypes.c_int(0)
        x = ctypes.c_int(0)
        dll.CAMAC_write(
            hdev, 25, 9, 16, ctypes.c_long(0), ctypes.byref(q), ctypes.byref(x)
        )
        dll.CAMAC_write(
            hdev, 25, 1, 16, ctypes.c_long(0), ctypes.byref(q), ctypes.byref(x)
        )

        # 3. CAMAC C/Z — reset all modules
        dll.CAMAC_C(hdev)
        time.sleep(0.2)
        dll.CAMAC_Z(hdev)
        time.sleep(0.2)

        # 4. Clear ADC and TDC data registers
        data = ctypes.c_long(0)
        dll.CAMAC_read(
            hdev,
            ADC_GAMMA["slot"],
            0,
            9,
            ctypes.byref(data),
            ctypes.byref(q),
            ctypes.byref(x),
        )
        dll.CAMAC_read(
            hdev,
            ADC_NEUTRON["slot"],
            0,
            9,
            ctypes.byref(data),
            ctypes.byref(q),
            ctypes.byref(x),
        )
        if ENABLE_TDC:
            dll.CAMAC_read(
                hdev,
                TDC_MAIN["slot"],
                0,
                9,
                ctypes.byref(data),
                ctypes.byref(q),
                ctypes.byref(x),
            )

        # 5. Flush any remaining data in USB FIFO
        buf = ctypes.create_string_buffer(32768)
        for _ in range(50):
            if dll.xxusb_bulk_read(hdev, buf, 32768, 100) <= 0:
                break

        print("[OK] List-mode stopped, hardware fully reset.")
    except Exception as e:
        print(f"[WARN] listmode_stop: {e}")


def listmode_parse_buffer(raw_bytes, n_bytes, event_time):
    """
    Parse Q-stop list-mode buffer with 2 or 3 module sections.

    Event structure (ENABLE_TDC=True):
      [evt_len][gamma_data...][0x0600][0x7C00][neutron_data...][0x0600][0x7C00][tdc_data...]

    Event structure (ENABLE_TDC=False):
      [evt_len][gamma_data...][0x0600][0x7C00][neutron_data...]

    The firmware ID read (N=25, returns 0x0600 + Q/X word 0x7C00) acts as
    a reliable separator.  Marker count determines which section we're in:
      0 markers seen → gamma (ADC_GAMMA)
      1 marker  seen → neutron (ADC_NEUTRON)
      2 markers seen → TDC (TDC_MAIN)

    The `seen` sets per section catch the ch0 duplicate from the Phillips
    7164 hardware peek quirk.  The 7186 TDC does not have this quirk but
    uses the same seen-set logic for safety.
    """
    import struct as _struct

    events = []
    if n_bytes < 4:
        return events

    n_words = n_bytes // 2
    words = list(_struct.unpack_from(f"<{n_words}H", bytes(raw_bytes[: n_words * 2])))

    # Debug: first 20 buffers
    if not hasattr(listmode_parse_buffer, "_dbg"):
        listmode_parse_buffer._dbg = 0
    if listmode_parse_buffer._dbg < 20:
        listmode_parse_buffer._dbg += 1
        hw = " ".join(f"{w:04X}" for w in words)
        print(f"[LMDEBUG #{listmode_parse_buffer._dbg}] {n_bytes}B: {hw}")

    header = words[0]
    n_events = header & 0x0FFF
    overflow = bool(header & 0x1000)
    pos = 1

    # Multi-event mode: skip total buffer word count
    if n_events > 1 or (pos < len(words) and words[pos] > 100):
        if pos < len(words) and words[pos] > 100:
            pos = 2

    for _ in range(n_events):
        if pos >= len(words):
            break

        evt_len = words[pos] & 0x0FFF
        pos += 1

        # Collect words into sections separated by 0x0600 markers
        sections = [[]]  # start with section 0 (gamma)
        marker_count = 0

        end_pos = pos + evt_len
        while pos < end_pos and pos < len(words):
            w = words[pos]
            pos += 1
            # NOTE: do NOT break on 0xFFFF here!  In the TDC section,
            # 0xFFFF is valid data (ch15, val=4095 = overflow/no stop).
            # The position-based end (end_pos) handles event boundaries.
            if w == 0x0000:
                continue
            if w == 0x0600:
                marker_count += 1
                sections.append([])  # start new section
                # Skip the Q/X response word (0x7C00)
                if pos < end_pos and pos < len(words):
                    pos += 1
                continue
            # 0x0300 could be ADC end-of-sparse marker OR valid TDC data
            # (ch0, val=768).  With markers separating sections, we don't
            # need 0x0300 as a boundary.  Just add it as data and let the
            # per-section decoder handle it.
            if marker_count < len(sections):
                sections[marker_count].append(w)
            else:
                sections.append([w])
                marker_count = len(sections) - 1

        # Section 0 = gamma, section 1 = neutron, section 2 = TDC
        gamma_words = sections[0] if len(sections) > 0 else []
        neutron_words = sections[1] if len(sections) > 1 else []
        tdc_words = sections[2] if len(sections) > 2 else []

        hits = []

        # --- Parse gamma ADC section ---
        seen_g = set()
        for w in gamma_words:
            ch = (w >> 12) & 0xF
            val = w & 0xFFF
            if w == 0x0300 or w == 0:
                continue
            if ch in seen_g:
                if not hasattr(listmode_parse_buffer, "_dup_g"):
                    listmode_parse_buffer._dup_g = 0
                listmode_parse_buffer._dup_g += 1
                if listmode_parse_buffer._dup_g <= 5:
                    print(f"  [QSTOP-FIX] Caught gamma duplicate ch{ch} val={val}")
                continue
            seen_g.add(ch)
            if ch < 15 and LOWER_THRESHOLD <= val <= UPPER_THRESHOLD_ADC:
                hits.append(
                    {
                        "slot": ADC_GAMMA["slot"],
                        "channel": ch,
                        "amplitude": val,
                        "timestamp": event_time,
                    }
                )

        # --- Parse neutron ADC section ---
        seen_n = set()
        for w in neutron_words:
            ch = (w >> 12) & 0xF
            val = w & 0xFFF
            if w == 0x0300 or w == 0:
                continue
            if ch in seen_n:
                if not hasattr(listmode_parse_buffer, "_dup_n"):
                    listmode_parse_buffer._dup_n = 0
                listmode_parse_buffer._dup_n += 1
                if listmode_parse_buffer._dup_n <= 5:
                    print(f"  [QSTOP-FIX] Caught neutron duplicate ch{ch} val={val}")
                continue
            seen_n.add(ch)
            if LOWER_THRESHOLD <= val <= UPPER_THRESHOLD_ADC:
                hits.append(
                    {
                        "slot": ADC_NEUTRON["slot"],
                        "channel": ch,
                        "amplitude": val,
                        "timestamp": event_time,
                    }
                )

        # --- Parse TDC section (if present) ---
        if ENABLE_TDC:
            seen_t = set()
            for w in tdc_words:
                ch = (w >> 12) & 0xF
                val = w & 0xFFF
                if w == 0x0300 or w == 0:
                    continue
                if ch in seen_t:
                    continue  # TDC shouldn't have duplicates but guard anyway
                seen_t.add(ch)
                if ch in TDC_MAIN["channels"] or not TDC_MAIN["channels"]:
                    if LOWER_THRESHOLD <= val <= UPPER_THRESHOLD_TDC:
                        hits.append(
                            {
                                "slot": TDC_MAIN["slot"],
                                "channel": ch,
                                "amplitude": val,
                                "timestamp": event_time,
                            }
                        )

        if hits:
            events.append({"timestamp": event_time, "hits": hits})

    return events


def build_stack_command(n, a, f, long_mode=0):
    cmd = f & 0x1F
    cmd |= (a & 0x0F) << 5
    cmd |= (n & 0x1F) << 9
    cmd |= (long_mode & 1) << 14
    return cmd


def clear_adc_hit_register(slot):
    """
    F(11)A(1): Reset Hit Register and LAM only (data memory preserved).
    Must be called after every ADC read in polled mode — without it the
    module keeps reporting 'data present' between gate pulses, causing
    stale ADC values to be re-read on subsequent polls. This produces
    overcounting on ch0 (always last in sparse output) and ch15 (direct
    read). Uses a separate stack buffer to avoid clobbering pre_built_stack.
    """
    clear_stack = (ctypes.c_long * 4)()
    clear_stack[0] = 1
    clear_stack[1] = build_stack_command(slot, 1, 11)
    dll.xxusb_stack_execute(hdev, clear_stack)


def read_adc_block(slot, active_channels):
    """
    Read a Phillips 7164 ADC.

    Sequence:
      1. F(6)A(1): Read Hit Register — bitmask of which channels have data.
                   If zero, no gate fired since last clear → skip entirely.
      2. F(4)A(0): Sparse read for channels 0-14 (ch15 omitted by hardware).
      3. F(0)A(15): Direct read for ch15, only if Hit Register bit 15 was set.
      4. F(11)A(1): Clear Hit Register so next poll cycle starts clean.

    This prevents ch0 and ch15 overcounting: without the Hit Register check,
    both get counted on every poll cycle at ~25x the true rate.
    """
    global total_sparse_reads

    # Step 1 — read Hit Register
    hr_data = ctypes.c_long(0)
    hr_q = ctypes.c_int(0)
    hr_x = ctypes.c_int(0)
    dll.CAMAC_read(
        hdev, slot, 1, 6, ctypes.byref(hr_data), ctypes.byref(hr_q), ctypes.byref(hr_x)
    )
    hit_register = hr_data.value & 0xFFFF

    if hit_register == 0:
        return []  # no gate fired since last clear — nothing to read

    # Step 2 — sparse read ch0-ch14
    N = 16
    pre_built_stack[0] = N
    cmd = build_stack_command(slot, 0, 4, 0)
    for i in range(1, N + 1):
        pre_built_stack[i] = cmd

    try:
        result = dll.xxusb_stack_execute(hdev, pre_built_stack)

        active_set = set(active_channels)
        seen = set()
        events = []

        if result >= 0:
            for i in range(1, 16):  # position 16 is a CC-USB firmware artefact
                word = pre_built_stack[i] & 0xFFFFFF
                if word == 0x300:
                    break
                if word == 0:
                    continue
                ch = (word >> 12) & 0xF
                val = word & 0xFFF
                if ch in seen or ch not in active_set:
                    continue
                seen.add(ch)
                if LOWER_THRESHOLD <= val <= UPPER_THRESHOLD_ADC:
                    total_sparse_reads += 1
                    events.append({"slot": slot, "channel": ch, "value": val})

        # Step 3 — ch15 direct read, only if Hit Register bit 15 was set
        if 15 in active_set and (hit_register & (1 << 15)):
            data = ctypes.c_long(0)
            q = ctypes.c_int(0)
            x = ctypes.c_int(0)
            dll.CAMAC_read(
                hdev, slot, 15, 0, ctypes.byref(data), ctypes.byref(q), ctypes.byref(x)
            )
            if q.value:
                val = data.value & 0xFFF
                if LOWER_THRESHOLD <= val <= UPPER_THRESHOLD_ADC:
                    total_sparse_reads += 1
                    events.append({"slot": slot, "channel": 15, "value": val})

        # Step 4 — clear Hit Register
        clear_adc_hit_register(slot)

        return events
    except Exception as e:
        print(f"[ERROR] ADC N={slot}: {e}")
        clear_adc_hit_register(slot)
        return []


def clear_tdc_hit_register(slot):
    pre_built_stack[0] = 1
    pre_built_stack[1] = build_stack_command(slot, 3, 11)
    dll.xxusb_stack_execute(hdev, pre_built_stack)


def read_tdc_block(slot, active_channels):
    N = 16
    pre_built_stack[0] = N
    cmd = build_stack_command(slot, 0, 4, 0)
    for i in range(1, N + 1):
        pre_built_stack[i] = cmd
    try:
        result = dll.xxusb_stack_execute(hdev, pre_built_stack)
        clear_tdc_hit_register(slot)
        if result < 0:
            return []
        active_set = set(active_channels)
        seen = set()
        events = []
        for i in range(1, 16):
            word = pre_built_stack[i] & 0xFFFFFF
            if word == 0x300:
                break
            if word == 0:
                continue
            ch = (word >> 12) & 0xF
            val = word & 0xFFF
            if ch in seen or ch not in active_set:
                continue
            seen.add(ch)
            if LOWER_THRESHOLD <= val <= UPPER_THRESHOLD_TDC:
                events.append({"slot": slot, "channel": ch, "value": val})
        return events
    except Exception as e:
        print(f"[ERROR] TDC N={slot}: {e}")
        clear_tdc_hit_register(slot)
        return []


def read_all_modules():
    event_time = time.time()
    all_hits = []
    for cfg in [ADC_GAMMA, ADC_NEUTRON]:
        for evt in read_adc_block(cfg["slot"], cfg["channels"]):
            all_hits.append(
                {
                    "slot": evt["slot"],
                    "channel": evt["channel"],
                    "amplitude": evt["value"],
                    "timestamp": event_time,
                }
            )
    if ENABLE_TDC:
        for evt in read_tdc_block(TDC_MAIN["slot"], TDC_MAIN["channels"]):
            all_hits.append(
                {
                    "slot": evt["slot"],
                    "channel": evt["channel"],
                    "amplitude": evt["value"],
                    "timestamp": event_time,
                }
            )
    return {"timestamp": event_time, "hits": all_hits} if all_hits else None


# ==================== ACQUISITION THREAD ====================


def acquisition_thread_func():
    global running, read_cycles, poll_cycles, readout_times, measured_readout_time
    print("[THREAD] Acquisition started")
    if ENABLE_LIST_MODE:
        _run_listmode()
    else:
        _run_polled()
    print("[THREAD] Stopped")


def _run_listmode():
    """
    List-mode acquisition loop.
    CC-USB FPGA executes the stack autonomously on every slot-17 LAM.
    Host drains the onboard FIFO via xxusb_bulk_read and parses buffers.
    τ per event = buffer collection time / events in buffer.

    Multi-event mode: each bulk_read may return a buffer with many events.
    The CC-USB header word bits 12-14 encode buffer status:
      bit 12 = overflow (events were lost).
    """
    global running, read_cycles, poll_cycles, readout_times, measured_readout_time

    buf = ctypes.create_string_buffer(LIST_MODE_BUF_BYTES)
    events_read = 0
    overflow_total = 0
    consecutive_errors = 0
    last_report = time.time()

    while running:
        t_start = time.perf_counter()
        n_bytes = dll.xxusb_bulk_read(
            hdev, buf, LIST_MODE_BUF_BYTES, LIST_MODE_TIMEOUT_MS
        )
        t_end = time.perf_counter()
        evt_time = time.time()

        if n_bytes <= 0:
            poll_cycles += 1
            if n_bytes < -1:
                # USB error (not just timeout)
                consecutive_errors += 1
                if consecutive_errors > 20:
                    print(
                        f"[THREAD] Too many USB errors ({consecutive_errors}), stopping"
                    )
                    running = False
                    break
            else:
                consecutive_errors = 0
            continue

        consecutive_errors = 0
        raw = bytearray(buf.raw[:n_bytes])

        # Check overflow bit in header (bit 12)
        if n_bytes >= 2:
            header_word = raw[0] | (raw[1] << 8)
            if header_word & 0x1000:
                overflow_total += 1
                if overflow_total <= 10 or overflow_total % 100 == 0:
                    print(
                        f"[WARN] CC-USB FIFO overflow #{overflow_total} — events lost! "
                        f"Consider reducing rate or increasing buffer size."
                    )

        evts = listmode_parse_buffer(raw, n_bytes, evt_time)

        dt = t_end - t_start
        n_evts = len(evts)

        # Measured τ: at 7 kHz true rate, 3530 ev/s observed →
        # τ = (1 - 3530/7000) / 3530 ≈ 140 µs.
        # Consistent with 32 CAMAC Q-stop cycles (2 slots × 16 ch) + USB framing.
        # For paper: sweep pulser 100 Hz → 20 kHz, fit non-paralyzable model.
        LIST_MODE_TAU_S = 140e-6
        if n_evts > 0:
            for _ in range(min(n_evts, 100)):
                readout_times.append(LIST_MODE_TAU_S)
            if len(readout_times) >= TAU_MIN_SAMPLES:
                measured_readout_time = LIST_MODE_TAU_S

        for evt in evts:
            read_cycles += 1
            events_read += 1
            try:
                event_queue.put_nowait(evt)
            except queue.Full:
                pass

        poll_cycles += 1

        now = time.time()
        if now - last_report > 5.0:
            rate = events_read / (now - last_report)
            dt_pct = rate * LIST_MODE_TAU_S * 100
            dup_g = getattr(listmode_parse_buffer, "_dup_g", 0)
            dup_n = getattr(listmode_parse_buffer, "_dup_n", 0)
            ovf_str = f" | OVERFLOW={overflow_total}" if overflow_total else ""
            print(
                f"[THREAD] {rate:.1f} ev/s | τ≈{LIST_MODE_TAU_S * 1e6:.0f}µs (Qstop) | DT≈{dt_pct:.2f}%"
                f" | buf {n_bytes}B {n_evts}evt"
                f" | dup_caught: g={dup_g} n={dup_n}{ovf_str}"
            )
            events_read = 0
            last_report = now


def _run_polled():
    """Original polled EASY-CAMAC loop with Hit Register gating."""
    global running, read_cycles, poll_cycles, readout_times, measured_readout_time

    events_read = 0
    last_report = time.time()

    while running:
        t_start = time.perf_counter()
        event = read_all_modules()
        t_end = time.perf_counter()
        dt = t_end - t_start
        if dt > 0:
            readout_times.append(dt)
            if len(readout_times) >= TAU_MIN_SAMPLES:
                measured_readout_time = sum(readout_times) / len(readout_times)
        if event:
            read_cycles += 1
            events_read += 1
            try:
                event_queue.put_nowait(event)
            except queue.Full:
                pass
        poll_cycles += 1
        now = time.time()
        if now - last_report > 5.0:
            rate = events_read / (now - last_report)
            print(f"[THREAD] {rate:.1f} ev/s | τ {measured_readout_time * 1e6:.0f} µs")
            events_read = 0
            last_report = now


# ==================== SAVE / AUTO-SAVE ====================


def save_numpy_format(filename, events):
    """
    Save events as structured NumPy .npy file.
    channel field is CAMAC 0-based address. Front panel = channel + 1.

    Memory-optimised for 32-bit Python: pre-allocates the array and fills
    it in-place instead of building an intermediate list of tuples.
    """
    try:
        import numpy as np

        # Count total hits first (cheap — no allocation)
        n_hits = sum(len(event["hits"]) for event in events)
        if n_hits == 0:
            print(f"[SAVE] {filename} — no hits, skipped")
            return

        # Pre-allocate structured array (single allocation)
        dt = np.dtype(
            [
                ("event", np.uint32),
                ("time_s", np.float64),
                ("slot", np.uint8),
                ("channel", np.uint8),  # CAMAC 0-based; front panel = +1
                ("value", np.uint16),
            ]
        )
        data = np.empty(n_hits, dtype=dt)

        # Fill in-place — no intermediate list
        idx = 0
        for enum, event in enumerate(events):
            ts = event["timestamp"] - t0
            for hit in event["hits"]:
                data[idx] = (enum, ts, hit["slot"], hit["channel"], hit["amplitude"])
                idx += 1

        np.save(filename, data)
        print(f"[SAVE] {filename} ({len(events)} events, {n_hits} hits)")
    except ImportError:
        print("[WARN] NumPy not available")
    except MemoryError:
        print(
            f"[SAVE] MemoryError on {filename} — {n_hits} hits too large for 32-bit Python"
        )
        # Emergency: try saving in two halves
        try:
            mid = len(events) // 2
            save_numpy_format(filename.replace(".npy", "_A.npy"), events[:mid])
            save_numpy_format(filename.replace(".npy", "_B.npy"), events[mid:])
        except Exception:
            print(f"[SAVE] Emergency split also failed — data lost")


def check_auto_save():
    global last_auto_save_count, auto_save_file_counter, events_to_save
    if not AUTO_SAVE_ENABLED or len(events_to_save) < AUTO_SAVE_INTERVAL:
        return
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    auto_save_file_counter += 1
    try:
        save_numpy_format(
            f"run_{ts}_part{auto_save_file_counter:03d}.npy", events_to_save
        )
        events_to_save.clear()
        gc.collect()
    except Exception as e:
        print(f"[AUTO-SAVE] Error: {e}")


# ==================== EVENT PROCESSING ====================


def process_event_queue():
    global event_count, trigger_count
    processed = 0
    while processed < MAX_EVENTS_BETWEEN_DISPLAYS:
        try:
            event = event_queue.get_nowait()
        except queue.Empty:
            break
        processed += 1
        trigger_count += 1
        event_list.append(event)
        events_to_save.append(event)
        for hit in event["hits"]:
            slot = hit["slot"]
            ch = hit["channel"]
            val = hit["amplitude"]
            event_count += 1
            if 0 <= ch < 16:
                if slot == ADC_GAMMA["slot"]:
                    gamma_data[ch].append(val)
                elif slot == ADC_NEUTRON["slot"]:
                    neutron_data[ch].append(val)
                elif slot == TDC_MAIN["slot"]:
                    tdc_data[ch].append(val)
    return processed


def clear_display_data():
    for ch in range(16):
        gamma_data[ch].clear()
        neutron_data[ch].clear()
        tdc_data[ch].clear()
    gc.collect()
    print("[CLEAR] Display buffers cleared")


# ==================== HISTOGRAM DRAWING ====================


def compute_hist(values_list, nbins=NBINS):
    if not values_list:
        return [0] * nbins
    bins = [0] * nbins
    span = VMAX - VMIN + 1
    for v in values_list:
        if VMIN <= v <= VMAX:
            idx = min((v - VMIN) * nbins // span, nbins - 1)
            bins[idx] += 1
    return bins


def draw_histogram(canvas, data, width, height, title, xlabel, color):
    canvas.delete("all")
    ml, mb, mt, mr = 55, 45, 30, 20
    pw = width - ml - mr
    ph = height - mt - mb
    canvas.create_rectangle(ml, mt, ml + pw, mt + ph, outline="white")
    bins = compute_hist(list(data), NBINS)
    bd = [math.log10(c + 1) for c in bins] if log_scale else bins
    maxc = max(bd) if bd else 1
    if maxc == 0:
        maxc = 1
    bw = pw / len(bd)
    for i, c in enumerate(bd):
        if c > 0:
            x0 = ml + i * bw
            h = (c / maxc) * ph
            canvas.create_rectangle(
                x0, mt + ph - h, x0 + bw, mt + ph, fill=color, outline=""
            )
    for tick_val in X_TICKS:
        frac = (tick_val - VMIN) / (VMAX - VMIN)
        xpix = ml + frac * pw
        y_bot = mt + ph
        canvas.create_line(xpix, y_bot, xpix, y_bot + 4, fill="white")
        canvas.create_text(
            xpix,
            y_bot + 14,
            text=str(tick_val),
            fill="white",
            font=("Arial", 8),
            anchor="center",
        )
    for tick_val in X_TICKS[1:-1]:
        frac = (tick_val - VMIN) / (VMAX - VMIN)
        xpix = ml + frac * pw
        canvas.create_line(xpix, mt, xpix, mt + ph, fill="#333333", dash=(2, 4))
    canvas.create_text(
        width // 2, 15, text=title, fill="white", font=("Arial", 11, "bold")
    )
    canvas.create_text(
        width // 2, height - 8, text=xlabel, fill="white", font=("Arial", 9)
    )
    count = len(list(data))
    canvas.create_text(
        ml + pw - 5, mt + 15, text=f"N={count}", fill="yellow", anchor="e"
    )
    if bins:
        peak_bin = bins.index(max(bins))
        peak_ch = int(peak_bin * (VMAX - VMIN) / NBINS + VMIN)
        canvas.create_text(
            ml + pw - 5,
            mt + 30,
            text=f"Peak ch≈{peak_ch}",
            fill="cyan",
            anchor="e",
            font=("Arial", 9),
        )


# ==================== GUI DRAWING ====================


def draw_all():
    if not ENABLE_LIVE_DISPLAY:
        return
    fp = current_gamma_channel + FP_OFFSET
    draw_histogram(
        gamma_canvas,
        gamma_data[current_gamma_channel],
        gamma_canvas_width,
        gamma_canvas_height,
        f"Gamma N={ADC_GAMMA['slot']}  CAMAC ch{current_gamma_channel}  (front panel ch{fp})",
        "ADC channel",
        "cyan",
    )
    if ENABLE_TDC:
        draw_histogram(
            tdc_canvas,
            tdc_data[current_tdc_channel],
            tdc_canvas_width,
            tdc_canvas_height,
            f"TDC N={TDC_MAIN['slot']} ch{current_tdc_channel} (raw)",
            "TDC channel (raw)",
            "magenta",
        )
    neutron_canvas_grid.delete("all")
    for row in range(neutron_grid_size):
        for col in range(neutron_grid_size):
            ch = row * neutron_grid_size + col
            x0 = col * neutron_cell_width
            y0 = row * neutron_cell_height
            x1, y1 = x0 + neutron_cell_width, y0 + neutron_cell_height
            neutron_canvas_grid.create_rectangle(x0, y0, x1, y1, outline="gray")
            margin_g = 5
            cell_pw = neutron_cell_width - 2 * margin_g
            for tick_val in X_TICKS[1:-1]:
                frac = (tick_val - VMIN) / (VMAX - VMIN)
                xpix = x0 + margin_g + frac * cell_pw
                neutron_canvas_grid.create_line(
                    xpix, y0 + 18, xpix, y1 - margin_g, fill="#333333", dash=(2, 4)
                )
            fp2 = ch + FP_OFFSET
            neutron_canvas_grid.create_text(
                x0 + 5,
                y0 + 5,
                text=f"C{ch}/FP{fp2}",
                fill="white",
                anchor="nw",
                font=("Arial", 7),
            )
            data = list(neutron_data[ch])
            if data:
                bins = compute_hist(data, 64)
                maxc = max(bins) or 1
                hist_h = neutron_cell_height - 30
                bw = (neutron_cell_width - 2 * 5) / 64
                for i, c in enumerate(bins):
                    if c > 0:
                        bx = x0 + 5 + i * bw
                        h = (c / maxc) * hist_h
                        neutron_canvas_grid.create_rectangle(
                            bx, y1 - 5 - h, bx + bw, y1 - 5, fill="lime", outline=""
                        )
                neutron_canvas_grid.create_text(
                    x1 - 5,
                    y1 - 5,
                    text=str(len(data)),
                    fill="yellow",
                    anchor="se",
                    font=("Arial", 8),
                )


# ==================== DEAD TIME ====================


def tau_converged():
    return len(readout_times) >= TAU_MIN_SAMPLES


def compute_deadtime():
    """Non-paralyzable dead time model.
    Uses trigger_count (one per gate pulse / stack execution), NOT event_count
    (which counts individual hits across all channels).
    τ is the time per trigger, so n must be the trigger rate."""
    elapsed = time.time() - t0 if t0 else 1e-9
    tau = measured_readout_time
    n = trigger_count / elapsed if elapsed > 0 else 0.0
    nt = min(n * tau, 0.9999)
    m = n / (1.0 - nt)
    return tau * 1e6, n, m, nt * 100.0


def dt_log_tick():
    global _last_dt_log_time
    if _dt_log_file is None or not tau_converged():
        return
    now = time.time()
    if now - _last_dt_log_time < DT_LOG_INTERVAL_S:
        return
    _last_dt_log_time = now
    tau_us, n_obs, m_true, dt_pct = compute_deadtime()
    if dt_pct >= 95.0:
        return
    elapsed = now - t0 if t0 else 0.0
    dt_hw = 0.0
    if ENABLE_SCALER and scaler_count_total > 0:
        dt_hw = max(0.0, (1.0 - trigger_count / scaler_count_total) * 100.0)
    try:
        _dt_log_file.write(
            f"{elapsed:.1f},{n_obs:.3f},{m_true:.3f},{tau_us:.1f},{dt_pct:.4f},"
            f"{trigger_count},{scaler_count_total},{dt_hw:.4f}\n"
        )
        _dt_log_file.flush()
    except Exception as e:
        print(f"[DT-LOG] Write error: {e}")


def update_stats():
    elapsed = time.time() - t0 if t0 else 1
    trig_rate = trigger_count / elapsed if elapsed > 0 else 0
    hit_rate = event_count / elapsed if elapsed > 0 else 0
    gc_cnt = sum(len(gamma_data[ch]) for ch in ADC_GAMMA["channels"])
    nc_cnt = sum(len(neutron_data[ch]) for ch in ADC_NEUTRON["channels"])
    tc_cnt = sum(len(tdc_data[ch]) for ch in TDC_MAIN["channels"]) if ENABLE_TDC else 0
    tdc_str = f" | TDC: {tc_cnt}" if ENABLE_TDC else " | TDC: OFF"
    info_label.config(
        text=(
            f"Triggers: {trigger_count} ({trig_rate:.1f}/s) | "
            f"Hits: {event_count} ({hit_rate:.1f}/s) | "
            f"Gamma: {gc_cnt} | Neutron: {nc_cnt}{tdc_str} | "
            f"Save buf: {len(events_to_save)}"
        ),
        fg="blue",
    )
    if not tau_converged():
        deadtime_label.config(text="Dead time: calibrating τ…", fg="gray")
    else:
        tau_us, n_obs, m_true, dt_pct = compute_deadtime()
        if dt_pct >= 95.0:
            deadtime_label.config(
                text=f"τ={tau_us:.1f}µs | Obs:{n_obs:.1f}cps | ⚠ SATURATED ({dt_pct:.1f}%)",
                fg="red",
            )
        else:
            col = "green" if dt_pct < 5 else ("orange" if dt_pct < 20 else "red")
            deadtime_label.config(
                text=(
                    f"τ={tau_us:.1f}µs | Obs:{n_obs:.1f}cps | "
                    f"True:{m_true:.1f}cps | DT:{dt_pct:.2f}%"
                ),
                fg=col,
            )

    if ENABLE_SCALER and scaler_label:
        if ENABLE_LIST_MODE:
            elapsed = time.time() - t0 if t0 else 1
            trig_rate = trigger_count / elapsed if elapsed > 0 else 0
            scaler_label.config(
                text=f"List mode: {trigger_count} triggers | {trig_rate:.1f} trig/s | "
                f"DT model only (scaler blocked in list mode)",
                fg="gray",
            )
        elif scaler_count_total > 0:
            dt_hw = max(0.0, (1.0 - trigger_count / scaler_count_total) * 100.0)
            model_dt = compute_deadtime()[3] if tau_converged() else 0.0
            agree = "agrees" if abs(dt_hw - model_dt) < 3.0 else "CHECK"
            col = "green" if dt_hw < 5 else ("orange" if dt_hw < 20 else "red")
            scaler_label.config(
                text=(
                    f"Scaler N={SCALER['slot']}: {scaler_count_total} | "
                    f"DAQ: {trigger_count} | "
                    f"HW DT: {dt_hw:.2f}% ({agree})"
                ),
                fg=col,
            )
        else:
            scaler_label.config(text="Scaler: waiting…", fg="gray")


def gui_update_loop():
    if running:
        process_event_queue()
        check_auto_save()
        scaler_tick()
        dt_log_tick()
        if ENABLE_LIVE_DISPLAY:
            draw_all()
        update_stats()
    root.after(DISPLAY_UPDATE_INTERVAL_MS, gui_update_loop)


# ==================== BUTTON CALLBACKS ====================


def on_toggle_display():
    global ENABLE_LIVE_DISPLAY
    ENABLE_LIVE_DISPLAY = not ENABLE_LIVE_DISPLAY
    toggle_display_btn.config(
        text=f"Display: {'ON (SLOW)' if ENABLE_LIVE_DISPLAY else 'OFF (FAST)'}",
        bg="#ff6b6b" if ENABLE_LIVE_DISPLAY else "#51cf66",
    )
    if ENABLE_LIVE_DISPLAY:
        draw_all()


def on_gamma_channel_change(event=None):
    global current_gamma_channel
    try:
        ch = int(gamma_channel_selector.get())
        if ch in ADC_GAMMA["channels"]:
            current_gamma_channel = ch
            if ENABLE_LIVE_DISPLAY:
                draw_all()
    except Exception:
        pass


def on_tdc_channel_change(event=None):
    global current_tdc_channel
    try:
        ch = int(tdc_channel_selector.get())
        if ch in TDC_MAIN["channels"]:
            current_tdc_channel = ch
            if ENABLE_LIVE_DISPLAY:
                draw_all()
    except Exception:
        pass


def toggle_log_scale():
    global log_scale
    log_scale = not log_scale
    if ENABLE_LIVE_DISPLAY:
        draw_all()


def on_start():
    global \
        running, \
        t0, \
        event_count, \
        trigger_count, \
        read_cycles, \
        poll_cycles, \
        total_sparse_reads
    global \
        acquisition_thread, \
        last_auto_save_count, \
        auto_save_file_counter, \
        events_to_save
    global _dt_log_file, _last_dt_log_time
    global scaler_count_total, scaler_count_last
    if running:
        return
    try:
        open_ccusb()
    except Exception as e:
        print(f"[ERROR] {e}")
        return
    for ch in range(16):
        gamma_data[ch].clear()
        neutron_data[ch].clear()
        tdc_data[ch].clear()
    event_list.clear()
    events_to_save.clear()
    while not event_queue.empty():
        try:
            event_queue.get_nowait()
        except Exception:
            break
    event_count = read_cycles = poll_cycles = total_sparse_reads = 0
    trigger_count = 0
    last_auto_save_count = auto_save_file_counter = 0
    scaler_count_total = scaler_count_last = 0
    readout_times.clear()
    t0 = time.time()
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    dt_log_path = f"deadtime_{ts}.csv"
    try:
        _dt_log_file = open(dt_log_path, "w")
        _dt_log_file.write(
            "elapsed_s,rate_obs_cps,rate_true_cps,tau_us,deadtime_pct,n_daq,n_scaler,dt_hw_pct\n"
        )
        _last_dt_log_time = 0.0
        print(f"[DT-LOG] {dt_log_path}")
    except Exception as e:
        print(f"[DT-LOG] Could not open: {e}")
        _dt_log_file = None
    running = True
    acquisition_thread = threading.Thread(target=acquisition_thread_func, daemon=True)
    acquisition_thread.start()


def on_stop():
    global running, acquisition_thread, _dt_log_file
    if not running:
        return
    running = False

    # In list mode: kill the CC-USB Action Register FIRST so xxusb_bulk_read
    # returns immediately (timeout), allowing the thread to exit cleanly.
    # Do this before join() so we don't wait 200ms for the timeout.
    if ENABLE_LIST_MODE and dll and hdev:
        try:
            import struct

            # CRITICAL: [5, 0, 0] = Action Register = 0 (stop)
            pkt = struct.pack("<3H", 5, 0, 0)
            dll.xxusb_bulk_write(hdev, pkt, len(pkt), 500)
        except Exception:
            pass

    if acquisition_thread and acquisition_thread.is_alive():
        acquisition_thread.join(timeout=3.0)

    process_event_queue()
    draw_all()

    if ENABLE_LIST_MODE and dll and hdev:
        listmode_stop()  # full cleanup: registers, C/Z, ADC clear, FIFO drain

    close_ccusb()
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    if events_to_save:
        save_numpy_format(f"run_{ts}_final.npy", events_to_save)
    if _dt_log_file:
        try:
            _dt_log_file.close()
            print("[DT-LOG] Log file closed.")
        except Exception:
            pass
        _dt_log_file = None


def on_close():
    global running, _dt_log_file
    if running:
        running = False
        if ENABLE_LIST_MODE and dll and hdev:
            try:
                import struct

                # CRITICAL: [5, 0, 0] = Action Register = 0 (stop)
                pkt = struct.pack("<3H", 5, 0, 0)
                dll.xxusb_bulk_write(hdev, pkt, len(pkt), 500)
            except Exception:
                pass
        if acquisition_thread and acquisition_thread.is_alive():
            acquisition_thread.join(timeout=3.0)
    if _dt_log_file:
        try:
            _dt_log_file.close()
        except Exception:
            pass
        _dt_log_file = None
    if dll and hdev:
        if ENABLE_LIST_MODE:
            listmode_stop()
        close_ccusb()
    root.quit()
    root.destroy()
    sys.exit(0)


def signal_handler(sig, frame):
    on_close()


# ==================== MAIN GUI ====================


def main():
    global root, gamma_canvas, neutron_canvas_grid, tdc_canvas, info_label
    global deadtime_label, scaler_label, toggle_display_btn, clear_display_btn
    global gamma_channel_selector, tdc_channel_selector

    signal.signal(signal.SIGINT, signal_handler)

    root = tk.Tk()
    root.title("CAMAC DAQ — List Mode Multi-Event")

    tk.Label(
        root,
        text=(
            f"Gamma N={ADC_GAMMA['slot']} | Neutron N={ADC_NEUTRON['slot']} | "
            f"Threshold={LOWER_THRESHOLD}  ·  "
            f"CAMAC address shown in GUI  ·  Front panel = CAMAC + 1"
        ),
        bg="lightblue",
        fg="darkblue",
        font=("Arial", 9, "bold"),
        pady=3,
    ).pack(fill=tk.X)

    display_frame = tk.Frame(root, bg="black")
    display_frame.pack(fill=tk.BOTH, expand=True)

    left_frame = tk.Frame(display_frame, bg="black")
    left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    gamma_canvas = tk.Canvas(
        left_frame, width=gamma_canvas_width, height=gamma_canvas_height, bg="black"
    )
    gamma_canvas.pack()

    tdc_canvas = tk.Canvas(
        left_frame, width=tdc_canvas_width, height=tdc_canvas_height, bg="black"
    )
    if ENABLE_TDC:
        tdc_canvas.pack()

    right_frame = tk.Frame(display_frame, bg="black")
    right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    tk.Label(
        right_frame,
        text="Neutron Wall  (labels: C=CAMAC addr / FP=front panel ch)",
        bg="black",
        fg="white",
        font=("Arial", 8, "bold"),
    ).pack()

    neutron_canvas_grid = tk.Canvas(
        right_frame,
        width=neutron_grid_size * neutron_cell_width,
        height=neutron_grid_size * neutron_cell_height,
        bg="black",
    )
    neutron_canvas_grid.pack()

    btn = tk.Frame(root)
    btn.pack(fill=tk.X, pady=2)

    tk.Button(
        btn,
        text="START",
        command=on_start,
        bg="green",
        fg="white",
        font=("Arial", 10, "bold"),
        width=8,
    ).pack(side=tk.LEFT, padx=4)
    tk.Button(
        btn,
        text="STOP",
        command=on_stop,
        bg="red",
        fg="white",
        font=("Arial", 10, "bold"),
        width=8,
    ).pack(side=tk.LEFT, padx=4)
    tk.Button(
        btn, text="LOG/LIN", command=toggle_log_scale, bg="blue", fg="white", width=8
    ).pack(side=tk.LEFT, padx=4)

    toggle_display_btn = tk.Button(
        btn,
        text="Display: OFF (FAST)",
        command=on_toggle_display,
        bg="#51cf66",
        fg="white",
        font=("Arial", 9, "bold"),
        width=16,
    )
    toggle_display_btn.pack(side=tk.LEFT, padx=4)

    tk.Button(
        btn,
        text="Clear Display",
        command=lambda: [
            clear_display_data(),
            draw_all() if ENABLE_LIVE_DISPLAY else None,
        ],
        bg="orange",
        fg="white",
        width=12,
    ).pack(side=tk.LEFT, padx=4)

    tk.Label(btn, text="Gamma CAMAC ch:").pack(side=tk.LEFT, padx=4)
    gamma_channel_selector = tk.Spinbox(
        btn, values=ADC_GAMMA["channels"], width=3, command=on_gamma_channel_change
    )
    gamma_channel_selector.pack(side=tk.LEFT)
    gamma_channel_selector.delete(0, tk.END)
    gamma_channel_selector.insert(0, str(current_gamma_channel))
    gamma_channel_selector.bind("<Return>", on_gamma_channel_change)
    tk.Label(
        btn,
        text=f"= FP ch{current_gamma_channel + FP_OFFSET}",
        fg="gray",
        font=("Arial", 8),
    ).pack(side=tk.LEFT)

    if ENABLE_TDC:
        tk.Label(btn, text="TDC ch:").pack(side=tk.LEFT, padx=4)
        tdc_channel_selector = tk.Spinbox(
            btn, values=TDC_MAIN["channels"], width=3, command=on_tdc_channel_change
        )
        tdc_channel_selector.pack(side=tk.LEFT)
        tdc_channel_selector.delete(0, tk.END)
        tdc_channel_selector.insert(0, "0")
        tdc_channel_selector.bind("<Return>", on_tdc_channel_change)

    info_label = tk.Label(root, text="", font=("Arial", 9))
    info_label.pack()

    deadtime_label = tk.Label(
        root, text="Dead time: —", font=("Arial", 9, "bold"), fg="green"
    )
    deadtime_label.pack()

    if ENABLE_SCALER:
        scaler_label = tk.Label(
            root, text="Scaler: —", font=("Arial", 9, "bold"), fg="gray"
        )
        scaler_label.pack()
    else:
        scaler_label = tk.Label(root, text="")  # dummy

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(DISPLAY_UPDATE_INTERVAL_MS, gui_update_loop)
    root.mainloop()


if __name__ == "__main__":
    main()
