"""
MULTI-MODULE CAMAC DAQ (Dual ADC + TDC) - CHANNELS ONLY
=========================================================
Slot 21: TDC (Timing)
Slot 17: Channels 5, 6 (Gamma detectors)
Slot 19: All 16 channels (Neutron wall)

NOTE: All TDC values stored and displayed in raw channels only.
      No ns conversion as we are not madmen
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

ADC_GAMMA = {
    "slot": 17,
    "channels": [14, 15],
    "name": "Gamma Detectors",
}  # list(range(16))
ADC_NEUTRON = {"slot": 19, "channels": list(range(0)), "name": "Neutron Wall"}
TDC_MAIN = {"slot": 21, "channels": list(range(2)), "name": "Main Timing"}

# Set to False to skip TDC readout entirely (removes dead time contribution
# from the TDC module when timing measurements are not needed)
ENABLE_TDC = True

# Set to True to enable Hit Register pre-check before full sparse read.
# Instead of unconditionally executing the 16-command sparse read stack,
# a single F(6)A(1) command reads the Hit Register (~50 µs) and the full
# read is skipped if no channels fired. Dramatically reduces dead time at
# low-to-moderate count rates. Requires F(26) LAM enable on each ADC at init.
ENABLE_LAM_POLL = True #still not working!!!

LOWER_THRESHOLD = 50
UPPER_THRESHOLD_ADC = 4080
UPPER_THRESHOLD_TDC = 3000  # 4091-4095 = TAC ran to end of range (no STOP received)

NBINS = 512
VMIN = 0
VMAX = 4095

# X-axis tick positions (in ADC/TDC channel units)
X_TICKS = [0, 512, 1024, 1536, 2048, 2560, 3072, 3584, 4096]

DISPLAY_UPDATE_INTERVAL_MS = 100
MAX_EVENTS_BETWEEN_DISPLAYS = 50000

AUTO_SAVE_ENABLED = True
AUTO_SAVE_INTERVAL = 1000000
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
read_cycles = 0
poll_cycles = 0  # every acquisition loop iteration, hit or not
total_sparse_reads = 0
log_scale = False

readout_times = deque(maxlen=1000)
# Initial estimate: ~500 µs without LAM poll (two full ADC reads + overhead).
# With ENABLE_LAM_POLL=True this converges quickly to ~50 µs for empty cycles,
# ~420 µs for hit cycles. The rolling mean reflects the actual mix at runtime.
measured_readout_time = 500e-6
TAU_MIN_SAMPLES = 20  # minimum samples before dead time display is trusted

# Dead time logging — appends one row every DT_LOG_INTERVAL_S seconds
DT_LOG_INTERVAL_S = 5.0
_last_dt_log_time = 0.0
_dt_log_file = None  # open CSV file handle, set on START

root = None
gamma_canvas = None
neutron_canvas_grid = None
tdc_canvas = None
info_label = None
deadtime_label = None  # second stats row for dead time display
toggle_display_btn = None
clear_display_btn = None
gamma_channel_selector = None
tdc_channel_selector = None
current_gamma_channel = 5
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


def enable_adc_lam(slot):
    """
    F(26)A(0): Enable LAM on the ADC module.
    Must be called after C/Z initialisation. LAM is cleared by C, Z, F(9),
    F(24), and F(11)A(1/3), so we re-enable it here explicitly.
    Required for ENABLE_LAM_POLL to work correctly.
    """
    try:
        q = ctypes.c_int(0)
        x = ctypes.c_int(0)
        # F(26) is a write-class command — use CAMAC_write with dummy data=0
        dll.CAMAC_write(
            hdev, slot, 0, 26, ctypes.c_long(0), ctypes.byref(q), ctypes.byref(x)
        )
        if q.value:
            print(f"  [OK]   ADC N={slot}: LAM enabled")
        else:
            print(f"  [WARN] ADC N={slot}: LAM enable returned Q=0")
    except Exception as e:
        print(f"  [ERROR] ADC N={slot} LAM enable: {e}")


def read_adc_hit_register(slot):
    """
    F(6)A(1): Read Hit Register non-destructively.
    Returns the raw 16-bit Hit Register word, or 0 on failure.
    A nonzero value means at least one channel has data within thresholds.
    This single command takes ~50 µs vs ~420 µs for a full sparse read,
    allowing fast polling with full reads only when data is present.
    """
    try:
        data = ctypes.c_long(0)
        q = ctypes.c_int(0)
        x = ctypes.c_int(0)
        dll.CAMAC_read(
            hdev, slot, 1, 6, ctypes.byref(data), ctypes.byref(q), ctypes.byref(x)
        )
        return data.value if q.value else 0
    except Exception:
        return 0


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

    hdev = dll.xxusb_serial_open(SERIAL)
    if not hdev:
        raise RuntimeError("Cannot open CC-USB")

    dll.CAMAC_C(hdev)
    time.sleep(0.2)
    dll.CAMAC_Z(hdev)
    time.sleep(0.2)

    print("Disabling ADC pedestal correction...")
    disable_adc_pedestal(ADC_GAMMA["slot"])
    disable_adc_pedestal(ADC_NEUTRON["slot"])

    if ENABLE_LAM_POLL:
        print("Enabling ADC LAM...")
        enable_adc_lam(ADC_GAMMA["slot"])
        enable_adc_lam(ADC_NEUTRON["slot"])

    N_COMMANDS = 16
    pre_built_stack = (ctypes.c_long * (N_COMMANDS + 10))()
    print("[OK] DAQ Hardware Ready")


def close_ccusb():
    global dll, hdev
    if dll and hdev:
        dll.xxusb_device_close(hdev)
        print("[OK] CC-USB closed.")
    dll = hdev = None


def build_stack_command(n, a, f, long_mode=0):
    cmd = f & 0x1F
    cmd |= (a & 0x0F) << 5
    cmd |= (n & 0x1F) << 9
    cmd |= (long_mode & 1) << 14
    return cmd


def read_adc_block(slot, active_channels):
    """Read a Phillips ADC via F(4)A(0) sparse read."""
    global total_sparse_reads
    N = 16
    pre_built_stack[0] = N
    cmd = build_stack_command(slot, 0, 4, 0)
    for i in range(1, N + 1):
        pre_built_stack[i] = cmd

    try:
        result = dll.xxusb_stack_execute(hdev, pre_built_stack)
        if result < 0:
            return []

        active_set = set(active_channels)
        seen = set()
        events = []

        for i in range(1, 16):  # skip position 16 (CC-USB firmware bug)
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

        return events
    except Exception as e:
        print(f"[ERROR] ADC N={slot}: {e}")
        return []


def clear_tdc_hit_register(slot):
    """
    F(11)A(3): Reset Hit Register, LAM and data memory.
    MUST be called after every TDC read — without this the module freezes
    on its first event and re-presents the same stale data on every poll.
    """
    pre_built_stack[0] = 1
    pre_built_stack[1] = build_stack_command(slot, 3, 11)
    dll.xxusb_stack_execute(hdev, pre_built_stack)


def read_tdc_block(slot, active_channels):
    """
    Read a Phillips 7186 TDC via F(4)A(0) sparse read.

    Key differences from ADC read:
      - Channels output in DESCENDING order (ch15 first, ch0 last)
      - val=4091-4095 means TAC ran to end of range (no STOP) — excluded
      - Hit Register MUST be cleared after every read via F(11)A(3)
    """
    N = 16
    pre_built_stack[0] = N
    cmd = build_stack_command(slot, 0, 4, 0)
    for i in range(1, N + 1):
        pre_built_stack[i] = cmd

    try:
        result = dll.xxusb_stack_execute(hdev, pre_built_stack)

        # Always clear Hit Register — even if read failed
        clear_tdc_hit_register(slot)

        if result < 0:
            return []

        active_set = set(active_channels)
        seen = set()
        events = []

        for i in range(1, 16):  # skip position 16 (CC-USB firmware bug)
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
        clear_tdc_hit_register(slot)  # clear even on exception
        return []


def read_all_modules():
    event_time = time.time()
    all_hits = []

    for cfg in [ADC_GAMMA, ADC_NEUTRON]:
        # LAM poll: skip the expensive sparse read if Hit Register is empty.
        # F(6)A(1) takes ~50 µs; full F(4)A(0) sparse read takes ~200 µs.
        # At low rates this eliminates the vast majority of USB transactions.
        if ENABLE_LAM_POLL and read_adc_hit_register(cfg["slot"]) == 0:
            continue
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

        poll_cycles += 1  # count every iteration regardless of hit

        now = time.time()
        if now - last_report > 5.0:
            rate = events_read / (now - last_report)
            print(
                f"[THREAD] {rate:.1f} ev/s | readout {measured_readout_time * 1e6:.0f} µs"
            )
            events_read = 0
            last_report = now

    print("[THREAD] Stopped")


# ==================== SAVE / AUTO-SAVE ====================


def save_numpy_format(filename, events):
    """Save events as a structured NumPy array (.npy).

    dtype fields:
      event   – sequential event index (uint32)
      time_s  – seconds since run start (float64)
      slot    – CAMAC slot number (uint8)
      channel – module channel (uint8)
      value   – raw ADC/TDC channel, no ns conversion (uint16)
    """
    try:
        import numpy as np

        rows = []
        for enum, event in enumerate(events):
            ts = event["timestamp"] - t0
            for hit in event["hits"]:
                rows.append((enum, ts, hit["slot"], hit["channel"], hit["amplitude"]))

        data = np.array(
            rows,
            dtype=[
                ("event", np.uint32),
                ("time_s", np.float64),
                ("slot", np.uint8),
                ("channel", np.uint8),
                ("value", np.uint16),  # raw channel, no ns
            ],
        )
        np.save(filename, data)
        print(f"[SAVE] {filename} ({len(events)} events, {len(data)} hits)")
    except ImportError:
        print("[WARN] NumPy not available — cannot save .npy")


def check_auto_save():
    global last_auto_save_count, auto_save_file_counter, events_to_save
    if not AUTO_SAVE_ENABLED or len(events_to_save) < AUTO_SAVE_INTERVAL:
        return
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    auto_save_file_counter += 1
    try:
        save_numpy_format(
            f"run_{ts}_part{auto_save_file_counter:03d}.npy",
            events_to_save,
        )
        print(
            f"[AUTO-SAVE] part {auto_save_file_counter} ({len(events_to_save)} events)"
        )
        events_to_save.clear()
        gc.collect()
    except Exception as e:
        print(f"[AUTO-SAVE] Error: {e}")


# ==================== EVENT PROCESSING ====================


def process_event_queue():
    global event_count
    processed = 0
    while processed < MAX_EVENTS_BETWEEN_DISPLAYS:
        try:
            event = event_queue.get_nowait()
        except queue.Empty:
            break
        processed += 1
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
    """Draw a histogram with labelled x-axis ticks at X_TICKS positions."""
    canvas.delete("all")
    ml, mb, mt, mr = 55, 45, 30, 20  # margins: left, bottom, top, right
    pw = width - ml - mr
    ph = height - mt - mb

    # Plot border
    canvas.create_rectangle(ml, mt, ml + pw, mt + ph, outline="white")

    # Histogram bars
    bins = compute_hist(list(data), NBINS)
    if log_scale:
        bd = [math.log10(c + 1) for c in bins]
    else:
        bd = bins
    maxc = max(bd) if bd else 1
    if maxc == 0:
        maxc = 1

    bw = pw / len(bd)
    for i, c in enumerate(bd):
        if c > 0:
            x0 = ml + i * bw
            h = (c / maxc) * ph
            canvas.create_rectangle(
                x0,
                mt + ph - h,
                x0 + bw,
                mt + ph,
                fill=color,
                outline="",
            )

    # ------ X-axis ticks & labels ------
    # VMIN..VMAX maps to pixel range [ml, ml+pw]
    for tick_val in X_TICKS:
        # fractional position along the axis
        frac = (tick_val - VMIN) / (VMAX - VMIN)
        xpix = ml + frac * pw
        y_bot = mt + ph

        # tick mark (4 px below baseline)
        canvas.create_line(xpix, y_bot, xpix, y_bot + 4, fill="white")

        # label
        canvas.create_text(
            xpix,
            y_bot + 14,
            text=str(tick_val),
            fill="white",
            font=("Arial", 8),
            anchor="center",
        )

    # Faint vertical grid lines at tick positions
    for tick_val in X_TICKS[1:-1]:  # skip the very first and last (on the border)
        frac = (tick_val - VMIN) / (VMAX - VMIN)
        xpix = ml + frac * pw
        canvas.create_line(xpix, mt, xpix, mt + ph, fill="#333333", dash=(2, 4))

    # ------ Titles & annotations ------
    canvas.create_text(
        width // 2,
        15,
        text=title,
        fill="white",
        font=("Arial", 11, "bold"),
    )
    canvas.create_text(
        width // 2,
        height - 8,
        text=xlabel,
        fill="white",
        font=("Arial", 9),
    )

    count = len(list(data))
    canvas.create_text(
        ml + pw - 5,
        mt + 15,
        text=f"N={count}",
        fill="yellow",
        anchor="e",
    )

    # Peak channel annotation
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
    draw_histogram(
        gamma_canvas,
        gamma_data[current_gamma_channel],
        gamma_canvas_width,
        gamma_canvas_height,
        f"Gamma ADC (N=17) Ch{current_gamma_channel}",
        "ADC channel",
        "cyan",
    )

    if ENABLE_TDC:
        draw_histogram(
            tdc_canvas,
            tdc_data[current_tdc_channel],
            tdc_canvas_width,
            tdc_canvas_height,
            f"TDC (N={TDC_MAIN['slot']}) Ch{current_tdc_channel} — raw channels (no ns conversion)",
            "TDC channel (raw)",
            "magenta",
        )

    # Neutron 4×4 mini-grid
    neutron_canvas_grid.delete("all")
    for row in range(neutron_grid_size):
        for col in range(neutron_grid_size):
            ch = row * neutron_grid_size + col
            x0 = col * neutron_cell_width
            y0 = row * neutron_cell_height
            x1, y1 = x0 + neutron_cell_width, y0 + neutron_cell_height
            neutron_canvas_grid.create_rectangle(x0, y0, x1, y1, outline="gray")

            # Faint grid lines at same X_TICKS fractions as main histograms
            margin_g = 5
            cell_pw = neutron_cell_width - 2 * margin_g
            cell_bot = y1 - margin_g
            cell_top = y0 + 18  # leave room for the "Ch" label
            for tick_val in X_TICKS[1:-1]:
                frac = (tick_val - VMIN) / (VMAX - VMIN)
                xpix = x0 + margin_g + frac * cell_pw
                neutron_canvas_grid.create_line(
                    xpix,
                    cell_top,
                    xpix,
                    cell_bot,
                    fill="#333333",
                    dash=(2, 4),
                )

            neutron_canvas_grid.create_text(
                x0 + 5,
                y0 + 5,
                text=f"Ch{ch}",
                fill="white",
                anchor="nw",
                font=("Arial", 8),
            )
            data = list(neutron_data[ch])
            if data:
                bins = compute_hist(data, 64)
                maxc = max(bins) or 1
                margin = 5
                hist_h = neutron_cell_height - 30
                bw = (neutron_cell_width - 2 * margin) / 64
                for i, c in enumerate(bins):
                    if c > 0:
                        bx = x0 + margin + i * bw
                        h = (c / maxc) * hist_h
                        neutron_canvas_grid.create_rectangle(
                            bx,
                            y1 - margin - h,
                            bx + bw,
                            y1 - margin,
                            fill="lime",
                            outline="",
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
    """True once we have enough readout time samples to trust τ."""
    return len(readout_times) >= TAU_MIN_SAMPLES


def compute_deadtime():
    """
    Non-paralyzable (Type I / Gedcke-Hale) dead time model.

    τ   = measured average readout cycle time  [s]
    n   = observed rate = event_count / elapsed [cps]
    DT% = n·τ · 100
    m   = n / (1 − n·τ)   true rate  [cps]  (undefined / clamped if n·τ ≥ 1)

    Returns (tau_us, n_obs, m_true, dt_pct) — all floats.
    Only call after tau_converged() is True.
    """
    elapsed = time.time() - t0 if t0 else 1e-9
    tau = measured_readout_time
    n = event_count / elapsed if elapsed > 0 else 0.0
    nt = n * tau
    if nt >= 1.0:
        nt = 0.9999
    m = n / (1.0 - nt)
    dt_pct = nt * 100.0
    return tau * 1e6, n, m, dt_pct


def dt_log_tick():
    """Append one CSV row to the dead time log file if the interval has elapsed."""
    global _last_dt_log_time
    if _dt_log_file is None or not tau_converged():
        return
    now = time.time()
    if now - _last_dt_log_time < DT_LOG_INTERVAL_S:
        return
    _last_dt_log_time = now
    tau_us, n_obs, m_true, dt_pct = compute_deadtime()
    elapsed = now - t0 if t0 else 0.0
    try:
        _dt_log_file.write(
            f"{elapsed:.1f},{n_obs:.3f},{m_true:.3f},{tau_us:.1f},{dt_pct:.4f}\n"
        )
        _dt_log_file.flush()
    except Exception as e:
        print(f"[DT-LOG] Write error: {e}")


def update_stats():
    elapsed = time.time() - t0 if t0 else 1
    rate = event_count / elapsed if elapsed > 0 else 0
    gc_cnt = sum(len(gamma_data[ch]) for ch in ADC_GAMMA["channels"])
    nc_cnt = sum(len(neutron_data[ch]) for ch in range(16))
    tc_cnt = sum(len(tdc_data[ch]) for ch in TDC_MAIN["channels"]) if ENABLE_TDC else 0
    tdc_str = f" | TDC: {tc_cnt}" if ENABLE_TDC else " | TDC: OFF"
    info_label.config(
        text=(
            f"Events: {event_count} | Rate: {rate:.1f} cps | "
            f"Gamma: {gc_cnt} | Neutron: {nc_cnt}{tdc_str} | "
            f"Save buf: {len(events_to_save)}"
        ),
        fg="blue",
    )

    # Dead time row
    if not tau_converged():
        deadtime_label.config(
            text="Dead time: calibrating τ…",
            fg="gray",
        )
    else:
        tau_us, n_obs, m_true, dt_pct = compute_deadtime()
        if dt_pct < 5.0:
            dt_color = "green"
        elif dt_pct < 20.0:
            dt_color = "orange"
        else:
            dt_color = "red"
        deadtime_label.config(
            text=(
                f"τ = {tau_us:.1f} µs  |  "
                f"Observed: {n_obs:.1f} cps  |  "
                f"True (corrected): {m_true:.1f} cps  |  "
                f"Dead time: {dt_pct:.2f}%"
            ),
            fg=dt_color,
        )


def gui_update_loop():
    if running:
        process_event_queue()
        check_auto_save()
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
    global running, t0, event_count, read_cycles, poll_cycles, total_sparse_reads
    global \
        acquisition_thread, \
        last_auto_save_count, \
        auto_save_file_counter, \
        events_to_save
    global _dt_log_file, _last_dt_log_time

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
    last_auto_save_count = auto_save_file_counter = 0
    readout_times.clear()
    t0 = time.time()

    # Open dead time CSV log
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    dt_log_path = f"deadtime_{ts}.csv"
    try:
        _dt_log_file = open(dt_log_path, "w")
        _dt_log_file.write("elapsed_s,rate_obs_cps,rate_true_cps,tau_us,deadtime_pct\n")
        _last_dt_log_time = 0.0
        print(f"[DT-LOG] Logging to {dt_log_path}")
    except Exception as e:
        print(f"[DT-LOG] Could not open log file: {e}")
        _dt_log_file = None

    running = True
    acquisition_thread = threading.Thread(target=acquisition_thread_func, daemon=True)
    acquisition_thread.start()


def on_stop():
    global running, acquisition_thread, _dt_log_file
    if not running:
        return
    running = False
    if acquisition_thread and acquisition_thread.is_alive():
        acquisition_thread.join(timeout=2.0)
    process_event_queue()
    draw_all()
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
        if acquisition_thread and acquisition_thread.is_alive():
            acquisition_thread.join(timeout=2.0)
    if _dt_log_file:
        try:
            _dt_log_file.close()
        except Exception:
            pass
        _dt_log_file = None
    if dll and hdev:
        close_ccusb()
    root.quit()
    root.destroy()
    sys.exit(0)


def signal_handler(sig, frame):
    on_close()


# ==================== MAIN GUI ====================


def main():
    global root, gamma_canvas, neutron_canvas_grid, tdc_canvas, info_label
    global \
        deadtime_label, \
        toggle_display_btn, \
        clear_display_btn, \
        gamma_channel_selector, \
        tdc_channel_selector

    signal.signal(signal.SIGINT, signal_handler)

    root = tk.Tk()
    root.title("CAMAC DAQ — Raw Channels (no time conversion)")

    tk.Label(
        root,
        text=f"Gamma N=17 | Neutron N=19 | TDC N={TDC_MAIN['slot']} — ALL VALUES IN RAW CHANNELS",
        bg="lightblue",
        fg="darkblue",
        font=("Arial", 10, "bold"),
        pady=4,
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
        text="Neutron Wall",
        bg="black",
        fg="white",
        font=("Arial", 10, "bold"),
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

    tk.Label(btn, text="Gamma Ch:").pack(side=tk.LEFT, padx=4)
    gamma_channel_selector = tk.Spinbox(
        btn, values=ADC_GAMMA["channels"], width=3, command=on_gamma_channel_change
    )
    gamma_channel_selector.pack(side=tk.LEFT)
    gamma_channel_selector.delete(0, tk.END)
    gamma_channel_selector.insert(0, "5")
    gamma_channel_selector.bind("<Return>", on_gamma_channel_change)

    if ENABLE_TDC:
        tk.Label(btn, text="TDC Ch:").pack(side=tk.LEFT, padx=4)
        tdc_channel_selector = tk.Spinbox(
            btn, values=list(range(16)), width=3, command=on_tdc_channel_change
        )
        tdc_channel_selector.pack(side=tk.LEFT)
        tdc_channel_selector.delete(0, tk.END)
        tdc_channel_selector.insert(0, "0")
        tdc_channel_selector.bind("<Return>", on_tdc_channel_change)

    info_label = tk.Label(root, text="", font=("Arial", 9))
    info_label.pack()

    deadtime_label = tk.Label(
        root,
        text="Dead time: —",
        font=("Arial", 9, "bold"),
        fg="green",
    )
    deadtime_label.pack()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.after(DISPLAY_UPDATE_INTERVAL_MS, gui_update_loop)
    root.mainloop()


if __name__ == "__main__":
    main()
