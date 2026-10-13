"""
MULTI-MODULE CAMAC DAQ (Dual Phillips 7164 ADC + optional 7186 TDC), CC-USB
===========================================================================
Version 10 (10list.py). Same acquisition logic, stack, thresholds and .npy
format as 9list.py, rebuilt so that nothing can overwrite memory and data is
written to disk continuously.

Slot 5 : Gamma detectors (Phillips 7164)      -> ADC_GAMMA
Slot 17: Neutron wall   (Phillips 7164)       -> ADC_NEUTRON
Slot 21: TDC            (Phillips 7186)       -> TDC_MAIN (ENABLE_TDC)

CHANNEL NUMBERING: CAMAC address 0-15 = front panel 1-16 (front = CAMAC + 1).
.npy fields: event (uint32), time_s (float64), slot (uint8),
             channel (uint8, CAMAC 0-based), value (uint16).
The event number is now global for the whole run (it no longer restarts at
0 in every part file), so parts can be concatenated directly.

WHAT CHANGED (checked against the CC-USB manual rev. 6.00 and libxxusb source)
-------------------------------------------------------------------------------
 1. Only four DLL functions are used: xxusb_serial_open, xxusb_device_close,
    xxusb_bulk_write, xxusb_bulk_read, always with buffers owned by Python
    and an explicit size. In libxxusb, xxusb_stack_execute copies every byte
    it receives (up to 26700) into the caller's array without a size check,
    CAMAC_read / CAMAC_write / CAMAC_C / CAMAC_Z hand it a 4-element local
    array, and xxusb_stack_read copies up to 1600 bytes into the caller's
    array. Any leftover or late data in the CC-USB output buffer overwrote
    memory, which in a 32-bit process kills Python with no traceback.
    Direct CAMAC commands are now built from the packet formats of manual
    sections 4.4 to 4.6 and the length of every answer is checked; on a
    mismatch the output buffer is drained and the command is repeated once.
 2. The CC-USB output buffer is drained after every stop of the autonomous
    acquisition and before any direct command (open, stop, recovery).
 3. Bulk read length 16384 bytes. The manual declares the length as a 16-bit
    WORD: 65536 does not fit and 32768 is negative as a signed short.
 4. -116 is the libusb-win32 timeout code (no data yet), not a USB error.
    List-mode read timeout 2000 ms, longer than the 1 s CC-USB watchdog
    (manual 3.3.10).
 5. Data read while stopping or during a recovery is parsed and saved.
 6. Parser: walks every CC-USB buffer contained in a transfer, always uses
    the second header word (Global Mode bit 8, HeaderOpt, is set; 9list.py
    only skipped it when it was > 100, so buffers of 100 words or less were
    misaligned), handles split events, and accepts the separator only as the
    pair 0x0600 0x7C00 (a real hit 0x0600 = ch0, 1536 was taken as separator).
 7. Saving: a writer thread saves a part every AUTO_SAVE_EVENTS events or
    AUTO_SAVE_SECONDS seconds (temporary file + rename). The last part is
    written before any hardware cleanup. NumPy is checked at start-up.
 8. Memory: no per-hit dicts, no 500k-event list, fixed 4096-bin histograms.
 9. One thread owns the USB during a run; the GUI only reads counters.
10. Session log in logs/ with faulthandler: a native crash leaves a
    Python traceback of every thread in the log.
11. F(19) is a write function (manual 4.5, F16-F23 carry data). 9list.py sent
    it through CAMAC_read, so the 0x0004 was never transmitted. It is no
    longer sent; the 7164 control register F(6)A(0) is printed instead.
    (7186 manual, same family: control bits 0x1 PED, 0x2 LT, 0x4 UT enable;
    LAM is enabled with F(26), which is still sent.)
    disable_adc_pedestal() is gone: F(0)A(0) is channel-1 data, not the
    control register, and F(16)A(0) writes channel-1 data memory.
12. N25 A2 is the Delays register (manual 3.3.3): bits 0-7 trigger delay in
    microseconds, bits 8-15 LAM timeout. 9list.py wrote 200 there as a
    "buffer timeout", which is a 200 us trigger delay. Kept at 200 so the
    behaviour is unchanged; see TRIGGER_DELAY_US.
13. Channel 15 (front panel 16). List mode: 9list.py dropped it in the gamma
    section with a hard-coded "ch < 15"; channels are now filtered with the
    "channels" lists of the configuration. Polled mode: 9list.py skipped the
    first sparse answer, which is the highest channel (channel 15 when it has
    data); now every answer is used and validated with the Hit Register
    (POLLED_SKIP_FIRST_WORD = True restores the old selection).
14. The list-mode separator is read from the firmware ID at start, so a
    CC-USB with other firmware does not break the gamma/neutron split.

DIAGNOSTICS (no GUI):
    python 10list.py --diag                 checks, USB test, raw sparse answers
    python 10list.py --diag --listtest 30   also 30 s of list mode, statistics
The last block of the diagnostic log is a summary meant to be compared
between two CC-USB units (firmware ID, registers, USB, channel 15).
"""

import argparse
import array
import collections
import ctypes
import datetime
import faulthandler
import hashlib
import os
import queue
import signal
import struct
import sys
import threading
import time
import traceback

try:
    import numpy as np
except ImportError as _e:
    print("NumPy cannot be imported by this interpreter:\n    %s" % sys.executable)
    if "DLL load failed" in str(_e):
        # NumPy is installed, but its C extension needs MSVCP140.dll (Microsoft
        # C++ runtime), which Python does not ship. A 32-bit Python needs the
        # x86 package, even on 64-bit Windows.
        print("NumPy is installed but a Windows DLL (MSVCP140.dll) is missing.")
        print("Install the Microsoft Visual C++ Redistributable x86 (32-bit):")
        print("    https://aka.ms/vc14/vc_redist.x86.exe")
    else:
        print("Error: %s" % _e)
        print('Install it with:\n    "%s" -m pip install numpy' % sys.executable)
    sys.exit(1)

# ==================== CONFIGURATION ====================
DLL_PATH = r"C:\Windows\SysWOW64\libxxusb.dll"
SERIAL = b"CC0284"

ADC_GAMMA = {"slot": 19, "channels": list(range(16)), "name": "Gamma Detectors"}
ADC_NEUTRON = {"slot": 17, "channels": list(range(16)), "name": "Neutron Wall"}
TDC_MAIN = {"slot": 21, "channels": list(range(2)), "name": "Main Timing"}
FP_OFFSET = 1  # front panel channel = CAMAC address + 1
ENABLE_TDC = False
ENABLE_LIST_MODE = True  # default state of the GUI checkbox

LOWER_THRESHOLD = 100
UPPER_THRESHOLD_ADC = 4080
UPPER_THRESHOLD_TDC = 4080

# Polon 420A scaler, polled mode only (as in 9list.py).
SCALER = {"slot": 22, "channel": 0, "name": "Polon 420A"}
ENABLE_SCALER = False

# ---- USB transfers ----
USB_READ_BYTES = 16384  # must stay <= 32767 (16-bit length in libxxusb)
EASY_TIMEOUT_MS = 1000  # one direct CAMAC command (answer normally < 1 ms)
LIST_MODE_TIMEOUT_MS = 2000  # > 1 s CC-USB watchdog (manual 3.3.10)
DRAIN_TIMEOUT_MS = 100
USB_TIMEOUT_CODE = -116  # libusb-win32 timeout: "no data yet"
USB_ERROR_LIMIT = 10  # consecutive real bulk_read errors before a recovery
MAX_RECOVERIES = 5  # recoveries in a row before a clean stop
GOOD_READS_TO_FORGIVE = 1000
POLLED_ERROR_LIMIT = 20  # consecutive failed polled cycles before a clean stop

# ---- CC-USB internal registers ----
# Action Register: manual 4.2 / 6.6 give sub-address 1. All runs with
# firmware 6.00 so far used 0 and it worked; change to 1 if START does not
# start the acquisition.
ACTION_SUBADDR = 0
GLOBAL_MODE = 0x0104  # BuffOpt=4 (256-word buffers), bit 8 HeaderOpt=1
HEADER_OPT = bool(GLOBAL_MODE & 0x0100)
TRIGGER_DELAY_US = 200  # N25 A2 bits 0-7. Same value 9list.py wrote.
LAM_TIMEOUT_US = 0  # N25 A2 bits 8-15
QSTOP_MAX = 20  # max repeats of each Q-stop read in the stack
DEFAULT_SEPARATOR = (0x0600, 0x7C00)  # N25 A0 F0 (firmware ID) in list mode

# Polled mode: False = use every sparse answer, validated with the Hit Register
# (channel 15 comes from the sparse read). True = 9list.py selection (first
# answer skipped, channel 15 by direct read).
POLLED_SKIP_FIRST_WORD = False

# 7164 control register: None = do not touch (default). Example: clear the
# pedestal-enable bit with ADC_CONTROL_CLEAR_BITS = 0x0001 (sent as F(23)A(0)).
ADC_CONTROL_SET_BITS = None  # sent as F(19)A(0) with data
ADC_CONTROL_CLEAR_BITS = None  # sent as F(23)A(0) with data

# ---- Saving ----
AUTO_SAVE_EVENTS = 100000  # part file every N events (as 9list.py) ...
AUTO_SAVE_SECONDS = 300  # ... or every N seconds, whichever comes first
DEFAULT_RUN_NAME = "run"  # each START makes <name>, <name>_2, <name>_3, ...
LOG_DIR = "logs"

# ---- Display and dead-time estimate ----
NBINS = 4096
DISPLAY_BINS = 1024
VMIN = 0
VMAX = 4096
X_TICKS = [0, 512, 1024, 1536, 2048, 2560, 3072, 3584, 4096]
DISPLAY_UPDATE_INTERVAL_MS = 200
LIST_MODE_TAU_S = 140e-6  # display-only estimate, same constant as 9list.py
TAU_MIN_SAMPLES = 20
DT_LOG_INTERVAL_S = 5.0
# =======================================================

_DLL_FACTORY = None  # tests can inject a simulated DLL here


# ==================== LOGGING ====================


class _Tee:
    """Duplicate console output into the session log file."""

    def __init__(self, stream, fh, lock):
        self.stream = stream
        self.fh = fh
        self.lock = lock

    def write(self, s):
        with self.lock:
            if self.stream is not None:
                try:
                    self.stream.write(s)
                except Exception:
                    pass
            try:
                self.fh.write(s)
                self.fh.flush()
            except Exception:
                pass
        return len(s)

    def flush(self):
        for x in (self.stream, self.fh):
            try:
                if x is not None:
                    x.flush()
            except Exception:
                pass

    def isatty(self):
        return False


_session_log_path = None


def setup_logging(tag="daq"):
    """Session log + faulthandler. Returns the log path."""
    global _session_log_path
    os.makedirs(LOG_DIR, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(LOG_DIR, "%s_%s.log" % (tag, ts))
    fh = open(path, "a", encoding="utf-8", buffering=1)
    lock = threading.Lock()
    sys.stdout = _Tee(sys.__stdout__, fh, lock)
    sys.stderr = _Tee(sys.__stderr__, fh, lock)
    faulthandler.enable(file=fh, all_threads=True)

    def _thread_hook(args):
        log("[EXCEPTION] in thread %s" % (args.thread.name if args.thread else "?"))
        traceback.print_exception(args.exc_type, args.exc_value, args.exc_traceback)

    threading.excepthook = _thread_hook
    _session_log_path = path
    return path


def log(msg):
    print("%s %s" % (datetime.datetime.now().strftime("%H:%M:%S"), msg), flush=True)


def hexwords(words, n=24):
    s = " ".join("%04X" % w for w in words[:n])
    if len(words) > n:
        s += " ... (%d words)" % len(words)
    return s


def dll_info(path=DLL_PATH):
    try:
        st = os.stat(path)
        with open(path, "rb") as f:
            sha = hashlib.sha256(f.read()).hexdigest()
        mt = datetime.datetime.fromtimestamp(st.st_mtime).isoformat(timespec="seconds")
        return "%s | %d bytes | modified %s | sha256 %s" % (path, st.st_size, mt, sha[:16])
    except Exception as e:
        return "%s | cannot read (%s)" % (path, e)


def environment_report():
    bits = struct.calcsize("P") * 8
    log("[ENV] Python %s (%d-bit) %s" % (sys.version.split()[0], bits, sys.executable))
    log("[ENV] NumPy %s" % np.__version__)
    log("[ENV] DLL %s" % dll_info())
    if _session_log_path:
        log("[ENV] Session log: %s" % os.path.abspath(_session_log_path))


# ==================== CC-USB ACCESS (bounds-safe) ====================


class CCUSBError(RuntimeError):
    pass


def camac_cmd(n, a, f, long_mode=False):
    """Stack / direct command word: F + 32*A + 512*N + 16384*LongMode (manual 4.5)."""
    return (f & 0x1F) | ((a & 0x0F) << 5) | ((n & 0x1F) << 9) | (0x4000 if long_mode else 0)


class CCUSB:
    """CC-USB through libxxusb.dll using only bulk transfers with owned buffers.

    Out packets (manual 4.2 to 4.4, 16-bit little-endian words):
      Action Register write : [5, sub-address, value]
      Direct CAMAC (NAF gen): [12, n, n command words]   -> answer on bulk IN
      Stack write           : [stack + 4, n, n words]     (stack 2 or 3)
      Stack read request    : [stack]
    """

    def __init__(self, dll_path=DLL_PATH, serial=SERIAL):
        self.serial = serial
        if _DLL_FACTORY is not None:
            dll = _DLL_FACTORY()
        else:
            dll = ctypes.WinDLL(dll_path)
            dll.xxusb_serial_open.restype = ctypes.c_void_p
            dll.xxusb_serial_open.argtypes = [ctypes.c_char_p]
            dll.xxusb_device_close.restype = ctypes.c_short
            dll.xxusb_device_close.argtypes = [ctypes.c_void_p]
            for name in ("xxusb_bulk_write", "xxusb_bulk_read"):
                fn = getattr(dll, name)
                fn.restype = ctypes.c_short
                fn.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
        self.dll = dll
        self.h = None
        self._rbuf = ctypes.create_string_buffer(USB_READ_BYTES)
        self._lock = threading.RLock()  # one USB transaction at a time
        self._hlock = threading.Lock()  # protects the handle for action_async
        self.resyncs = 0
        self.easy_errors = 0
        self.drained_bytes = 0

    # ---- device ----
    def open(self):
        h = self.dll.xxusb_serial_open(self.serial)
        if not h:
            raise CCUSBError(
                "xxusb_serial_open(%s) failed: CC-USB not found, powered off, "
                "or libusb-win32 driver not installed" % self.serial.decode()
            )
        self.h = h
        log("[USB] CC-USB %s opened" % self.serial.decode())

    def close(self):
        with self._lock, self._hlock:
            if self.h:
                try:
                    self.dll.xxusb_device_close(self.h)
                finally:
                    self.h = None
                log("[USB] CC-USB closed")

    # ---- raw transfers (callers hold self._lock) ----
    def _write_words(self, words, timeout_ms):
        pkt = struct.pack("<%dH" % len(words), *[w & 0xFFFF for w in words])
        return self.dll.xxusb_bulk_write(self.h, pkt, len(pkt), timeout_ms)

    def _read(self, timeout_ms):
        n = self.dll.xxusb_bulk_read(self.h, self._rbuf, USB_READ_BYTES, timeout_ms)
        if n > 0:
            n = min(n, USB_READ_BYTES)
            return n, ctypes.string_at(self._rbuf, n)
        return n, b""

    def read_bulk(self, timeout_ms):
        with self._lock:
            return self._read(timeout_ms)

    def drain(self, quiet_reads=2, timeout_ms=DRAIN_TIMEOUT_MS, max_seconds=5.0, sink=None):
        """Read until `quiet_reads` consecutive empty reads. Data goes to `sink`
        (list-mode parser) or is discarded. Returns the number of bytes read."""
        total = 0
        quiet = 0
        t_end = time.monotonic() + max_seconds
        with self._lock:
            while quiet < quiet_reads and time.monotonic() < t_end:
                n, raw = self._read(timeout_ms)
                if n > 0:
                    total += n
                    quiet = 0
                    if sink is not None:
                        try:
                            sink(raw)
                        except Exception:
                            log("[DRAIN] parser error:\n" + traceback.format_exc())
                else:
                    quiet += 1
        self.drained_bytes += total
        return total

    # ---- Action Register ----
    def action(self, value):
        with self._lock:
            ret = self._write_words([5, ACTION_SUBADDR, value], EASY_TIMEOUT_MS)
        if ret < 0:
            raise CCUSBError("Action Register write %d failed (%d)" % (value, ret))
        return ret

    def action_async(self, value):
        """Action Register write from another thread while the acquisition
        thread waits in bulk_read (different endpoint). Never raises."""
        try:
            with self._hlock:
                if self.h:
                    return self._write_words([5, ACTION_SUBADDR, value], EASY_TIMEOUT_MS)
        except Exception:
            pass
        return None

    # ---- direct CAMAC (EASY-CAMAC) ----
    def execute(self, cmds, n_min, n_max, what="", retries=1):
        """Execute a direct stack. The answer must contain n_min..n_max data
        words plus an optional terminator word. Returns the answer words."""
        pkt = [0x000C, len(cmds) & 0x0FFF] + [c & 0xFFFF for c in cmds]
        problem = ""
        for attempt in range(retries + 1):
            with self._lock:
                if not self.h:
                    raise CCUSBError("device not open")
                wr = self._write_words(pkt, EASY_TIMEOUT_MS)
                n, raw = (wr, b"") if wr < 0 else self._read(EASY_TIMEOUT_MS)
            if wr < 0:
                problem = "bulk_write returned %d" % wr
            elif n > 0 and n % 2 == 0:
                words = struct.unpack("<%dH" % (n // 2), raw)
                if n_min <= len(words) <= n_max + 1:
                    return words
                problem = "answer of %d words (expected %d to %d + terminator): %s" % (
                    len(words), n_min, n_max, hexwords(words, 16))
            elif n > 0:
                problem = "odd answer length (%d bytes)" % n
            else:
                problem = "bulk_read returned %d" % n
            self.resyncs += 1
            if self.resyncs <= 50 or self.resyncs % 100 == 0:
                log("[USB] %s: %s -> drain%s (resync #%d)" % (
                    what or hexwords(cmds), problem,
                    " and retry" if attempt < retries else "", self.resyncs))
            self.drain(quiet_reads=2, timeout_ms=DRAIN_TIMEOUT_MS, max_seconds=3.0)
        self.easy_errors += 1
        raise CCUSBError("%s: %s" % (what or hexwords(cmds), problem))

    def naf_read(self, n, a, f, what=None):
        """24-bit read, same encoding as libxxusb CAMAC_read. Returns (data, q, x)."""
        w = self.execute([camac_cmd(n, a, f, True)], 2, 2, what or "N%d A%d F%d read" % (n, a, f))
        return w[0] | ((w[1] & 0xFF) << 16), (w[1] >> 8) & 1, (w[1] >> 9) & 1

    def naf_write(self, n, a, f, data, what=None):
        """24-bit write (F16-F23), same encoding as libxxusb CAMAC_write. Returns (q, x)."""
        if not 16 <= f <= 23:
            raise ValueError("F%d is not a write function" % f)
        w = self.execute([camac_cmd(n, a, f, True), data & 0xFFFF, (data >> 16) & 0xFF],
                         1, 1, what or "N%d A%d F%d write 0x%X" % (n, a, f, data))
        return w[0] & 1, (w[0] >> 1) & 1

    def naf_control(self, n, a, f, what=None):
        """Control function (F8-F15, F24-F31): no data, answer is a Q/X word."""
        if 16 <= f <= 23 or f < 8:
            raise ValueError("F%d is not a control function" % f)
        w = self.execute([camac_cmd(n, a, f)], 1, 2, what or "N%d A%d F%d" % (n, a, f))
        return w[0] & 1, (w[0] >> 1) & 1

    def probe(self, n, a, f, long_mode=False, what=None):
        """Lenient read used only for diagnostics: returns all answer words."""
        return self.execute([camac_cmd(n, a, f, long_mode)], 1, 4, what or "N%d A%d F%d probe" % (n, a, f))

    def camac_c(self):
        return self.execute([camac_cmd(28, 9, 29)], 0, 2, "CAMAC C")

    def camac_z(self):
        return self.execute([camac_cmd(28, 8, 29)], 0, 2, "CAMAC Z")

    # ---- command stacks ----
    def write_stack(self, stack_addr, words):
        pkt = [(stack_addr & 0x33) + 4, len(words) & 0x0FFF] + [w & 0xFFFF for w in words]
        with self._lock:
            ret = self._write_words(pkt, 2000)
        if ret != 2 * len(pkt):
            raise CCUSBError("stack write returned %d (expected %d)" % (ret, 2 * len(pkt)))
        return ret

    def read_stack(self, stack_addr):
        with self._lock:
            wr = self._write_words([stack_addr & 0x33], 2000)
            if wr < 0:
                return None
            n, raw = self._read(2000)
        if n <= 0 or n % 2:
            return None
        return struct.unpack("<%dH" % (n // 2), raw)


# ==================== HARDWARE SETUP / CLEANUP ====================


def build_stack():
    """Q-stop sparse read of each module, separated by N25 A0 F0 markers."""
    qs = [0x8010, QSTOP_MAX]  # options word: C=1, QS=1; then max count
    w = [0x8000 | camac_cmd(ADC_GAMMA["slot"], 0, 4)] + qs
    w += [camac_cmd(25, 0, 0)]
    w += [0x8000 | camac_cmd(ADC_NEUTRON["slot"], 0, 4)] + qs
    if ENABLE_TDC:
        w += [camac_cmd(25, 0, 0)]
        w += [0x8000 | camac_cmd(TDC_MAIN["slot"], 0, 4)] + qs
    return w


def read_separator(cc):
    """Read N25 A0 F0 as the stack does (16-bit) and decode the firmware ID."""
    sep = DEFAULT_SEPARATOR
    try:
        w = cc.probe(25, 0, 0, False, "firmware ID")
        log("[CC-USB] N25 A0 F0 answer: %s" % hexwords(w))
        if len(w) >= 3 and len(sep) == 2 and (w[0], w[1]) != tuple(sep):
            log("[CC-USB] WARNING: separator differs from %s, using %04X %04X" % (hexwords(sep), w[0], w[1]))
            sep = (w[0], w[1])
        log("[CC-USB] firmware ID %s" % fw_decode(w)[1])
    except CCUSBError as e:
        log("[CC-USB] firmware ID read failed (%s); using default separator" % e)
    return sep


def adc_report_and_setup(cc, cfg):
    slot = cfg["slot"]
    ctrl, q, x = cc.naf_read(slot, 0, 6, "N%d F(6)A(0) control register" % slot)
    note = "" if x else "   <-- X=0: nothing answered in this slot"
    log("[7164 N=%d] control register 0x%04X (bit0 PED=%d bit1 LT=%d bit2 UT=%d) Q=%d X=%d%s" % (
        slot, ctrl & 0xFFFF, ctrl & 1, (ctrl >> 1) & 1, (ctrl >> 2) & 1, q, x, note))
    changed = False
    if ADC_CONTROL_CLEAR_BITS:
        cc.naf_write(slot, 0, 23, ADC_CONTROL_CLEAR_BITS, "N%d F(23)A(0) clear control bits" % slot)
        changed = True
    if ADC_CONTROL_SET_BITS:
        cc.naf_write(slot, 0, 19, ADC_CONTROL_SET_BITS, "N%d F(19)A(0) set control bits" % slot)
        changed = True
    if changed:
        ctrl, q, x = cc.naf_read(slot, 0, 6)
        log("[7164 N=%d] control register now 0x%04X" % (slot, ctrl & 0xFFFF))
    cc.naf_control(slot, 0, 26, "N%d F(26) enable LAM" % slot)


def listmode_setup(cc):
    stack = build_stack()
    ret = cc.write_stack(2, stack)
    log("[LIST] stack loaded (%d words, %d bytes): %s" % (len(stack), ret, hexwords(stack)))
    rb = cc.read_stack(2)
    if rb is None:
        log("[LIST] stack read-back: no answer (not fatal)")
    else:
        found = any(tuple(rb[i:i + len(stack)]) == tuple(stack) for i in range(0, 3))
        log("[LIST] stack read-back %s: %s" % ("OK" if found else "DIFFERENT", hexwords(rb, 16)))
    cc.drain(quiet_reads=2)  # a late read-back answer must not reach the next command

    lam_mask = (1 << (ADC_GAMMA["slot"] - 1)) | (1 << (ADC_NEUTRON["slot"] - 1))
    cc.naf_write(25, 9, 16, lam_mask, "LAM mask")
    cc.naf_write(25, 1, 16, GLOBAL_MODE, "Global Mode")
    delays = ((LAM_TIMEOUT_US & 0xFF) << 8) | (TRIGGER_DELAY_US & 0xFF)
    cc.naf_write(25, 2, 16, delays, "Delays register")
    log("[LIST] LAM mask 0x%06X | Global Mode 0x%04X (%d-word buffers, HeaderOpt=%d) | "
        "trigger delay %d us, LAM timeout %d us" % (
            lam_mask, GLOBAL_MODE, [4096, 2048, 1024, 512, 256, 128, 64, 1][GLOBAL_MODE & 7],
            int(HEADER_OPT), TRIGGER_DELAY_US, LAM_TIMEOUT_US))
    for a, name in ((9, "LAM mask"), (1, "Global Mode"), (2, "Delays")):
        try:
            log("[LIST] read-back N25 A%d (%s): %s" % (a, name, hexwords(cc.probe(25, a, 0, True))))
        except CCUSBError as e:
            log("[LIST] read-back N25 A%d failed: %s" % (a, e))


def hardware_setup(cc, list_mode):
    """Bring CC-USB and modules to a known state. Returns the list-mode separator."""
    cc.action(0)  # stop any autonomous acquisition left running
    time.sleep(0.05)
    stale = cc.drain(quiet_reads=3, timeout_ms=DRAIN_TIMEOUT_MS, max_seconds=10.0)
    if stale:
        log("[OPEN] discarded %d bytes left in the CC-USB output buffer" % stale)
    else:
        log("[OPEN] CC-USB output buffer was empty")
    cc.naf_write(25, 9, 16, 0, "LAM mask = 0")
    cc.naf_write(25, 1, 16, 0, "Global Mode = 0")
    cc.camac_c()
    time.sleep(0.2)
    cc.camac_z()
    time.sleep(0.2)
    sep = read_separator(cc)
    for cfg in (ADC_GAMMA, ADC_NEUTRON):
        adc_report_and_setup(cc, cfg)
    if ENABLE_TDC:
        cc.naf_control(TDC_MAIN["slot"], 0, 9, "TDC clear")
        cc.naf_control(TDC_MAIN["slot"], 0, 26, "TDC enable LAM")
    if ENABLE_SCALER:
        cc.naf_control(SCALER["slot"], 0, 9, "scaler clear")
    if list_mode:
        listmode_setup(cc)
    log("[OK] hardware ready, mode %s, LOWER_THRESHOLD=%d" % ("LIST" if list_mode else "POLLED", LOWER_THRESHOLD))
    return sep


def hardware_cleanup(cc, list_mode):
    """Leave the crate idle. Every step is independent."""
    steps = [
        ("stop acquisition", lambda: cc.action(0)),
        ("drain", lambda: cc.drain(quiet_reads=2)),
        ("LAM mask = 0", lambda: cc.naf_write(25, 9, 16, 0)),
        ("Global Mode = 0", lambda: cc.naf_write(25, 1, 16, 0)),
    ]
    if list_mode:
        steps += [
            ("CAMAC C", cc.camac_c),
            ("CAMAC Z", cc.camac_z),
            ("clear gamma 7164", lambda: cc.naf_control(ADC_GAMMA["slot"], 0, 9)),
            ("clear neutron 7164", lambda: cc.naf_control(ADC_NEUTRON["slot"], 0, 9)),
        ]
        if ENABLE_TDC:
            steps.append(("clear 7186", lambda: cc.naf_control(TDC_MAIN["slot"], 0, 9)))
        steps.append(("final drain", lambda: cc.drain(quiet_reads=2)))
    for name, fn in steps:
        try:
            fn()
        except Exception as e:
            log("[CLEANUP] %s failed: %s" % (name, e))
    log("[OK] hardware left idle")


# ==================== LIST-MODE PARSER ====================


class ListParser:
    """Decodes CC-USB list-mode transfers (manual 4.6).

    A transfer may hold several buffers. Buffer:
      header (bit15 LB/watchdog, bit14 scaler, bit13 cont, bit12 MB, bits0-11 events)
      [word count, if HeaderOpt]
      events: length word (bits0-11 length, bit12 = partial event) + data
      0xFFFF terminator
    Event data: gamma Q-stop words, separator pair, neutron words
    [, separator pair, TDC words]. The Q-stop returns the hit channels highest
    first, so channel 15 is the first word of its section. Channels are
    filtered with the "channels" lists of the configuration (9list.py dropped
    gamma channel 15 with a hard-coded "ch < 15").
    """

    def __init__(self, separator=DEFAULT_SEPARATOR, with_tdc=ENABLE_TDC):
        self.sep = tuple(separator)
        self.n_sections = 3 if with_tdc else 2
        self.with_tdc = with_tdc
        self.g_ch = frozenset(ADC_GAMMA["channels"])
        self.n_ch = frozenset(ADC_NEUTRON["channels"])
        self.t_ch = frozenset(TDC_MAIN["channels"])
        self.pending = []
        self.s = collections.Counter()
        # words per channel as delivered by each module (duplicates included,
        # before thresholds): shows directly whether channel 15 arrives
        self.raw = {"gamma": [0] * 16, "neutron": [0] * 16, "tdc": [0] * 16}
        self._dumps = 0
        self._count_checks = 0

    def _dump(self, why, words):
        self._dumps += 1
        if self._dumps <= 8:
            log("[PARSE] %s: %s" % (why, hexwords(words, 40)))

    @staticmethod
    def _walk(words, pos, n_ev):
        n = len(words)
        for _ in range(n_ev):
            if pos >= n:
                return None
            pos += 1 + (words[pos] & 0x0FFF)
            if pos > n:
                return None
        if pos == n or words[pos] == 0xFFFF:
            return pos
        return None

    def parse(self, words):
        """Returns a list of hit lists [(slot, ch, value), ...], one per event with hits."""
        out = []
        n = len(words)
        pos = 0
        nbuf = 0
        while pos < n:
            hdr = words[pos]
            if hdr == 0xFFFF or hdr == 0x0000:
                pos += 1
                continue
            n_ev = hdr & 0x0FFF
            start = pos + 1
            end = None
            ev_pos = start
            if HEADER_OPT and start < n:
                end = self._walk(words, start + 1, n_ev)
                if end is not None:
                    ev_pos = start + 1
                    if self._count_checks < 3:
                        self._count_checks += 1
                        log("[PARSE] buffer check: 2nd header word %d, words header..terminator %d" % (
                            words[start], end - pos + 1))
            if end is None:
                end = self._walk(words, start, n_ev)
                if end is not None:
                    ev_pos = start
                    self.s["no_count_word"] += 1
            if end is None:
                self.s["bad_buffers"] += 1
                self._dump("inconsistent buffer, rest of transfer skipped", words[pos:])
                break
            nbuf += 1
            self.s["buffers"] += 1
            for bit, name in ((0x8000, "flag_LB"), (0x4000, "flag_scaler"), (0x2000, "flag_cont"), (0x1000, "flag_MB")):
                if hdr & bit:
                    self.s[name] += 1
            if hdr & 0x4000:  # scaler buffer: no scaler stack is loaded
                pos = end
                continue
            p = ev_pos
            for _ in range(n_ev):
                lw = words[p]
                ln = lw & 0x0FFF
                ev = words[p + 1:p + 1 + ln]
                p += 1 + ln
                if lw & 0x1000:  # partial event, continues in the next part
                    self.pending.extend(ev)
                    self.s["partial_parts"] += 1
                    continue
                if self.pending:
                    ev = tuple(self.pending) + tuple(ev)
                    self.pending = []
                self.s["events"] += 1
                hits = self.decode_event(ev)
                if hits:
                    out.append(hits)
            pos = end
        if nbuf > 1:
            self.s["multi_buffer_transfers"] += 1
        return out

    def decode_event(self, ev):
        s0 = self.sep[0]
        s1 = self.sep[1] if len(self.sep) > 1 else None
        step = 2 if s1 is not None else 1
        sections = [[]]
        i = 0
        L = len(ev)
        while i < L:
            w = ev[i]
            if (w == s0 and len(sections) < self.n_sections
                    and (s1 is None or (i + 1 < L and ev[i + 1] == s1))):
                sections.append([])
                i += step
                continue
            if w != 0:
                sections[-1].append(w)
            i += 1
        if len(sections) < self.n_sections:
            self.s["missing_separator"] += 1
            self._dump("event without separator", ev)
            if self.s["missing_separator"] == 100 and self.s["events"] < 200:
                log("[PARSE] WARNING: most events have no separator %s; check DEFAULT_SEPARATOR"
                    % hexwords(self.sep))
        hits = []
        # 0x0300 is dropped in every section exactly as in 9list.py (end-of-sparse
        # word). It is also CAMAC ch0 = 768, so that single value is lost on ch0.
        # The first word of each channel is kept; the Q=0 read at the end of a
        # Q-stop repeats the last word and is counted as a duplicate.
        plan = [(sections[0], "gamma", ADC_GAMMA["slot"], self.g_ch, UPPER_THRESHOLD_ADC, "dup_gamma")]
        if len(sections) > 1:
            plan.append((sections[1], "neutron", ADC_NEUTRON["slot"], self.n_ch, UPPER_THRESHOLD_ADC, "dup_neutron"))
        if self.with_tdc and len(sections) > 2:
            plan.append((sections[2], "tdc", TDC_MAIN["slot"], self.t_ch, UPPER_THRESHOLD_TDC, "dup_tdc"))
        for words, name, slot, chans, upper, dupkey in plan:
            raw = self.raw[name]
            seen = set()
            for w in words:
                if w == 0x0300:
                    continue
                ch = w >> 12
                raw[ch] += 1
                if ch in seen:
                    self.s[dupkey] += 1
                    continue
                seen.add(ch)
                val = w & 0xFFF
                if ch in chans and LOWER_THRESHOLD <= val <= upper:
                    hits.append((slot, ch, val))
        return hits

    def raw_report(self):
        lines = []
        for name in ("gamma", "neutron", "tdc"):
            r = self.raw[name]
            if any(r):
                lines.append("%-7s words per channel 0..15: %s" % (name, " ".join(str(x) for x in r)))
        return lines


# ==================== POLLED READOUT ====================


SPARSE_READS = 17  # one more than the 16 channels: a leading duplicate cannot hide channel 0


def read_sparse_polled(cc, slot, active_channels, upper, clear_a):
    """Hit Register gated sparse read of a 7164 (or 7186).

    Sparse data come highest channel first (7186 manual, same family), so the
    first answer is channel 15 whenever it has data. 9list.py skipped that
    first answer, which is why channel 15 "never appeared" in the sparse read
    and needed a direct F(0)A(15) read. Here every answer is used, a word is
    accepted only if its channel bit is set in the Hit Register read just
    before, and only the first word of each channel is kept (after the last
    hit channel the module repeats its last word).
    POLLED_SKIP_FIRST_WORD = True restores the 9list.py selection exactly.
    """
    hit, _q, _x = cc.naf_read(slot, 1, 6, "N%d hit register" % slot)
    hit &= 0xFFFF
    if hit == 0:
        return []
    words = cc.execute([camac_cmd(slot, 0, 4)] * SPARSE_READS, SPARSE_READS, SPARSE_READS,
                       "N%d sparse read" % slot)[:SPARSE_READS]
    active = set(active_channels)
    seen = set()
    hits = []
    if POLLED_SKIP_FIRST_WORD:
        for w in words[1:16]:
            if w == 0x0300:
                break
            if w == 0:
                continue
            ch = (w >> 12) & 0xF
            val = w & 0xFFF
            if ch in seen or ch not in active:
                continue
            seen.add(ch)
            if LOWER_THRESHOLD <= val <= upper:
                hits.append((slot, ch, val))
    else:
        for w in words:
            ch = (w >> 12) & 0xF
            if not (hit >> ch) & 1 or ch in seen:
                continue
            seen.add(ch)
            val = w & 0xFFF
            if ch in active and LOWER_THRESHOLD <= val <= upper:
                hits.append((slot, ch, val))
    if 15 in active and (hit & 0x8000) and 15 not in seen:
        d, q, _x = cc.naf_read(slot, 15, 0, "N%d ch15 direct read" % slot)  # fallback only
        if q:
            val = d & 0xFFF
            if LOWER_THRESHOLD <= val <= upper:
                hits.append((slot, 15, val))
    cc.naf_control(slot, clear_a, 11, "N%d F(11)A(%d) reset" % (slot, clear_a))
    return hits


def read_all_polled(cc):
    # Modules are read one after the other, as in 9list.py: a gate that arrives
    # between the gamma and the neutron read ends up split over two events.
    hits = []
    for cfg in (ADC_GAMMA, ADC_NEUTRON):
        hits += read_sparse_polled(cc, cfg["slot"], cfg["channels"], UPPER_THRESHOLD_ADC, 1)
    if ENABLE_TDC:
        hits += read_sparse_polled(cc, TDC_MAIN["slot"], TDC_MAIN["channels"], UPPER_THRESHOLD_TDC, 3)
    return hits


# ==================== RUN STATE, STORAGE ====================


def make_run_folder(base_name):
    base = (base_name or "").strip()
    base = "".join(c for c in base if c.isalnum() or c in ("_", "-", ".", " "))
    base = base.strip().replace(" ", "_") or DEFAULT_RUN_NAME
    candidate = base
    k = 1
    while os.path.exists(candidate):
        k += 1
        candidate = "%s_%d" % (base, k)
    os.makedirs(candidate)
    return candidate


NPY_DTYPE = np.dtype([("event", np.uint32), ("time_s", np.float64), ("slot", np.uint8),
                      ("channel", np.uint8), ("value", np.uint16)])


class HitChunk:
    """Compact columnar storage for one part file."""

    def __init__(self):
        self.event = array.array("I")
        self.time_s = array.array("d")
        self.slot = array.array("B")
        self.channel = array.array("B")
        self.value = array.array("H")
        self.n_events = 0
        self.t_created = time.monotonic()

    def add(self, evno, t_rel, hits):
        for slot, ch, val in hits:
            self.event.append(evno)
            self.time_s.append(t_rel)
            self.slot.append(slot)
            self.channel.append(ch)
            self.value.append(val)
        self.n_events += 1

    def to_numpy(self):
        out = np.empty(len(self.value), dtype=NPY_DTYPE)
        if len(self.value):
            out["event"] = np.frombuffer(self.event, dtype=np.uint32)
            out["time_s"] = np.frombuffer(self.time_s, dtype=np.float64)
            out["slot"] = np.frombuffer(self.slot, dtype=np.uint8)
            out["channel"] = np.frombuffer(self.channel, dtype=np.uint8)
            out["value"] = np.frombuffer(self.value, dtype=np.uint16)
        return out


class Writer(threading.Thread):
    """Writes part files in the background: temporary file, fsync, rename."""

    def __init__(self, folder):
        super().__init__(name="writer", daemon=True)
        self.folder = folder
        self.q = queue.Queue()
        self.saved_files = 0
        self.saved_hits = 0
        self.errors = 0

    def put(self, fname, chunk):
        self.q.put((fname, chunk))

    def finish(self):
        self.q.put(None)

    def run(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            fname, chunk = item
            self._save(fname, chunk)

    def _save(self, fname, chunk):
        path = os.path.join(self.folder, fname)
        tmp = path + ".tmp"
        try:
            data = chunk.to_numpy()
            with open(tmp, "wb") as f:
                np.save(f, data)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, path)
            self.saved_files += 1
            self.saved_hits += len(data)
            log("[SAVE] %s (%d events, %d hits)" % (path, chunk.n_events, len(data)))
        except Exception:
            self.errors += 1
            log("[SAVE] FAILED %s:\n%s" % (path, traceback.format_exc()))


class RunState:
    def __init__(self, list_mode, run_base):
        self.list_mode = list_mode
        self.run_base = run_base
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.state = "starting"  # starting, running, stopping, stopped
        self.stop_reason = None  # set when the acquisition thread stops by itself
        self.error = None
        self.cc = None
        self.run_folder = None
        self.writer = None
        self.parser = None
        self.t0 = None
        self.event_number = 0
        self.trigger_count = 0
        self.hit_count = 0
        self.ccusb_events = 0
        self.idle_timeouts = 0
        self.usb_errors = 0
        self.recoveries = 0
        self.polled_errors = 0
        self.transfers = 0
        self.bytes_read = 0
        self.part = 0
        self.readout_times = collections.deque(maxlen=1000)
        self.measured_tau = 500e-6
        self.scaler_total = 0
        self.scaler_last = 0
        slots = [ADC_GAMMA["slot"], ADC_NEUTRON["slot"], TDC_MAIN["slot"]]
        self.hist = {s: [[0] * NBINS for _ in range(16)] for s in slots}
        self.chunk = HitChunk()

    def tau_converged(self):
        return self.list_mode or len(self.readout_times) >= TAU_MIN_SAMPLES

    def tau(self):
        return LIST_MODE_TAU_S if self.list_mode else self.measured_tau


def write_run_info(st, sep):
    try:
        with open(os.path.join(st.run_folder, "run_info.txt"), "w") as f:
            f.write("CAMAC DAQ run (10list.py)\n")
            f.write("start          : %s\n" % datetime.datetime.now().isoformat(timespec="seconds"))
            f.write("mode           : %s\n" % ("LIST" if st.list_mode else "POLLED"))
            f.write("lower_threshold: %d\n" % LOWER_THRESHOLD)
            f.write("gamma slot     : %d (CAMAC ch %s)\n" % (ADC_GAMMA["slot"], ADC_GAMMA["channels"]))
            f.write("neutron slot   : %d (CAMAC ch %s)\n" % (ADC_NEUTRON["slot"], ADC_NEUTRON["channels"]))
            if ENABLE_TDC:
                f.write("tdc slot       : %d\n" % TDC_MAIN["slot"])
            if st.list_mode:
                f.write("global mode    : 0x%04X\n" % GLOBAL_MODE)
                f.write("trigger delay  : %d us (N25 A2)\n" % TRIGGER_DELAY_US)
                f.write("separator      : %s\n" % hexwords(sep))
            f.write("dll            : %s\n" % dll_info())
            f.write("python         : %s (%d-bit)\n" % (sys.version.split()[0], struct.calcsize("P") * 8))
            if _session_log_path:
                f.write("session log    : %s\n" % os.path.abspath(_session_log_path))
            f.write("note           : channel in .npy is CAMAC 0-based; front panel = channel + 1\n")
            f.write("note           : event number is global for the whole run\n")
    except Exception as e:
        log("[RUN-INFO] could not write: %s" % e)


# ==================== ACQUISITION THREAD ====================


class Acquisition(threading.Thread):
    """Owns the CC-USB from open to close."""

    def __init__(self, st):
        super().__init__(name="acquisition", daemon=True)
        self.st = st

    # ---- event bookkeeping ----
    def _record(self, t_abs, hits):
        st = self.st
        evno = st.event_number
        st.event_number += 1
        st.trigger_count += 1
        st.hit_count += len(hits)
        st.chunk.add(evno, t_abs - st.t0, hits)
        hist = st.hist
        for slot, ch, val in hits:
            h = hist.get(slot)
            if h is not None and 0 <= ch < 16 and 0 <= val < NBINS:
                h[ch][val] += 1

    def _rotate(self, final=False):
        st = self.st
        c = st.chunk
        due = c.n_events >= AUTO_SAVE_EVENTS or (
            c.n_events and time.monotonic() - c.t_created >= AUTO_SAVE_SECONDS)
        if not (final or due):
            return
        with st.lock:
            st.chunk = HitChunk()
        if c.n_events and st.writer is not None:
            st.part += 1
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            name = "run_%s_final.npy" % ts if final else "run_%s_part%03d.npy" % (ts, st.part)
            st.writer.put(name, c)

    def _handle_transfer(self, raw):
        st = self.st
        st.transfers += 1
        st.bytes_read += len(raw)
        nw = len(raw) // 2
        if len(raw) % 2:
            st.parser.s["odd_transfers"] += 1
        words = struct.unpack_from("<%dH" % nw, raw)
        before = st.parser.s["events"]
        events = st.parser.parse(words)
        st.ccusb_events += st.parser.s["events"] - before
        t = time.time()
        for hits in events:
            self._record(t, hits)

    # ---- main ----
    def run(self):
        st = self.st
        cc = None
        try:
            cc = CCUSB()
            cc.open()
            st.cc = cc
            sep = hardware_setup(cc, st.list_mode)
            st.run_folder = make_run_folder(st.run_base)
            log("[RUN] output folder: %s" % os.path.abspath(st.run_folder))
            write_run_info(st, sep)
            st.writer = Writer(st.run_folder)
            st.writer.start()
            st.parser = ListParser(sep, ENABLE_TDC)
            st.t0 = time.time()
            st.state = "running"
            if st.list_mode:
                self._run_list(cc)
            else:
                self._run_polled(cc)
        except Exception as e:
            st.error = str(e)
            st.stop_reason = st.stop_reason or ("error: %s" % e)
            log("[THREAD] stopped by exception:\n" + traceback.format_exc())
        finally:
            st.state = "stopping"
            if st.list_mode and cc is not None and cc.h and st.parser is not None:
                try:  # end of run: stop, then read and keep what is still buffered
                    cc.action(0)
                    got = cc.drain(quiet_reads=3, timeout_ms=200, max_seconds=5.0, sink=self._handle_transfer)
                    log("[LIST] acquisition stopped, %d bytes collected after stop" % got)
                except Exception as e:
                    log("[LIST] stop/drain failed: %s" % e)
            try:
                self._rotate(final=True)  # data to the writer before touching hardware
            except Exception:
                log("[THREAD] final save queueing failed:\n" + traceback.format_exc())
            if cc is not None and cc.h:
                hardware_cleanup(cc, st.list_mode)
                cc.close()
            if st.writer is not None:
                st.writer.finish()
                st.writer.join(timeout=120)
            if st.parser is not None and st.list_mode:
                log("[PARSE] %s" % dict(st.parser.s))
            if cc is not None:
                log("[USB] resyncs %d, direct-command errors %d, bytes drained %d" % (
                    cc.resyncs, cc.easy_errors, cc.drained_bytes))
            log("[THREAD] finished: %d events with hits, %d hits" % (st.trigger_count, st.hit_count))
            st.state = "stopped"

    def _run_list(self, cc):
        st = self.st
        cc.action(1)
        log("[LIST] acquisition started")
        consecutive = 0
        recoveries = 0
        good = 0
        last_report = time.time()
        last_events = 0
        while not st.stop_event.is_set():
            n, raw = cc.read_bulk(LIST_MODE_TIMEOUT_MS)
            if n > 0:
                consecutive = 0
                good += 1
                if recoveries and good > GOOD_READS_TO_FORGIVE:
                    recoveries = 0
                self._handle_transfer(raw)
            elif n == 0 or n == USB_TIMEOUT_CODE:
                st.idle_timeouts += 1
                consecutive = 0
            else:
                consecutive += 1
                st.usb_errors += 1
                if st.usb_errors <= 20 or st.usb_errors % 100 == 0:
                    log("[LIST] bulk_read error %d (%d in a row)" % (n, consecutive))
                if consecutive >= USB_ERROR_LIMIT:
                    if recoveries >= MAX_RECOVERIES:
                        st.stop_reason = "%d USB recoveries failed" % recoveries
                        log("[LIST] %s, stopping cleanly" % st.stop_reason)
                        break
                    recoveries += 1
                    st.recoveries += 1
                    good = 0
                    log("[LIST] recovery %d/%d: stop, read what is buffered, restart" % (recoveries, MAX_RECOVERIES))
                    try:
                        cc.action(0)
                        cc.drain(quiet_reads=3, timeout_ms=200, max_seconds=5.0, sink=self._handle_transfer)
                        cc.action(1)
                        consecutive = 0
                    except CCUSBError as e:
                        st.stop_reason = "recovery failed: %s" % e
                        log("[LIST] " + st.stop_reason)
                        break
                else:
                    time.sleep(0.01)
            self._rotate()
            now = time.time()
            if now - last_report > 10.0:
                rate = (st.trigger_count - last_events) / (now - last_report)
                log("[LIST] %.1f ev/s | transfers %d | idle timeouts %d | USB errors %d | parse %s" % (
                    rate, st.transfers, st.idle_timeouts, st.usb_errors,
                    {k: v for k, v in st.parser.s.items() if k not in ("events", "buffers")}))
                last_report = now
                last_events = st.trigger_count

    def _run_polled(self, cc):
        st = self.st
        consecutive = 0
        last_report = time.time()
        last_events = 0
        last_scaler = 0.0
        while not st.stop_event.is_set():
            t_a = time.perf_counter()
            try:
                hits = read_all_polled(cc)
                consecutive = 0
            except CCUSBError as e:
                consecutive += 1
                st.polled_errors += 1
                if consecutive >= POLLED_ERROR_LIMIT:
                    st.stop_reason = "%d consecutive failed readouts (%s)" % (consecutive, e)
                    log("[POLLED] %s, stopping cleanly" % st.stop_reason)
                    break
                time.sleep(0.01)
                continue
            dt = time.perf_counter() - t_a
            st.readout_times.append(dt)
            if len(st.readout_times) >= TAU_MIN_SAMPLES and (len(st.readout_times) % 20 == 0):
                st.measured_tau = sum(st.readout_times) / len(st.readout_times)
            if hits:
                self._record(time.time(), hits)
            if ENABLE_SCALER and time.time() - last_scaler > 0.5:
                last_scaler = time.time()
                self._scaler_tick(cc)
            self._rotate()
            now = time.time()
            if now - last_report > 10.0:
                rate = (st.trigger_count - last_events) / (now - last_report)
                log("[POLLED] %.1f ev/s | readout %.0f us | failed readouts %d | resyncs %d" % (
                    rate, st.measured_tau * 1e6, st.polled_errors, cc.resyncs))
                last_report = now
                last_events = st.trigger_count

    def _scaler_tick(self, cc):
        st = self.st
        try:
            d, q, _x = cc.naf_read(SCALER["slot"], SCALER["channel"], 0, "scaler read")
        except CCUSBError:
            return
        if not q:
            return
        raw = d & 0xFFFFFF
        if raw < st.scaler_last:
            st.scaler_total += (0xFFFFFF - st.scaler_last) + raw + 1
        else:
            st.scaler_total += raw - st.scaler_last
        st.scaler_last = raw


# ==================== DIAGNOSTICS (no GUI) ====================


def fw_decode(words):
    """Firmware ID from the N25 A0 F0 answer (manual 3.3.1: MYBBFFRR, hex digits)."""
    data = list(words)
    if len(data) >= 3 or (len(data) == 2 and data[1] != 0xFFFF):
        fw = (data[1] << 16) | data[0]
        return fw, "0x%08X (MYBBFFRR: month %X, year %X, beta %02X, version %X.%02X)" % (
            fw, fw >> 28, (fw >> 24) & 0xF, (fw >> 16) & 0xFF, (fw >> 8) & 0xFF, fw & 0xFF)
    fw = data[0]
    return fw, "0x%04X (only 16 bits answered: version %X.%02X)" % (fw, (fw >> 8) & 0xFF, fw & 0xFF)


def sparse_dump(cc, slot, n_events=3, wait_s=3.0, clear_a=1):
    """Show the raw sparse answers of a few real events (needs signals on the module)."""
    got = 0
    seen15 = False
    t_end = time.time() + wait_s
    while got < n_events and time.time() < t_end:
        hit, _q, _x = cc.naf_read(slot, 1, 6)
        hit &= 0xFFFF
        if hit == 0:
            time.sleep(0.005)
            continue
        words = cc.execute([camac_cmd(slot, 0, 4)] * SPARSE_READS, SPARSE_READS, SPARSE_READS,
                           "N%d sparse dump" % slot)[:SPARSE_READS]
        chans = [ch for ch in range(16) if (hit >> ch) & 1]
        seen15 = seen15 or any((w >> 12) == 15 for w in words)
        log("N%-2d hit register 0x%04X, channels %s" % (slot, hit, chans))
        log("     answers channel:value -> %s" % " ".join("%d:%d" % (w >> 12, w & 0xFFF) for w in words))
        cc.naf_control(slot, clear_a, 11)
        got += 1
    if not got:
        log("N%-2d no gate in %.0f s (no signal on this module?)" % (slot, wait_s))
    return got, seen15


def run_diagnostics(listtest_s=0.0, stress_n=3000):
    environment_report()
    summary = collections.OrderedDict()
    summary["serial"] = SERIAL.decode()
    cc = CCUSB()
    cc.open()
    try:
        log("---- 1. leftover data in the CC-USB output buffer ----")
        cc.action(0)
        time.sleep(0.05)
        first = []

        def keep(raw):
            if len(first) < 64:
                first.extend(struct.unpack_from("<%dH" % (len(raw) // 2), raw))

        stale = cc.drain(quiet_reads=3, timeout_ms=200, max_seconds=15.0, sink=keep)
        log("leftover bytes: %d%s" % (stale, ("  first words: " + hexwords(first, 32)) if stale else ""))
        if stale > 8:
            log("  -> with 9list.py the next CAMAC_read/CAMAC_write would have copied this into a 4-element buffer")
        summary["leftover bytes at open"] = stale

        log("---- 2. firmware ID and internal registers ----")
        for long_mode in (False, True):
            try:
                w = cc.probe(25, 0, 0, long_mode, "firmware ID")
                log("N25 A0 F0 %s-bit answer: %s" % ("24" if long_mode else "16", hexwords(w)))
                if not long_mode:
                    summary["firmware ID"] = fw_decode(w)[1]
            except CCUSBError as e:
                log("firmware ID read failed: %s" % e)
                summary["firmware ID"] = "read failed"
        log("firmware ID: %s" % summary.get("firmware ID"))
        for a, name in ((1, "Global Mode"), (2, "Delays (bits0-7 trigger delay us, bits8-15 LAM timeout us)"),
                        (9, "LAM mask"), (14, "USB bulk transfer setup")):
            try:
                log("N25 A%-2d %-60s %s" % (a, name, hexwords(cc.probe(25, a, 0, True))))
            except CCUSBError as e:
                log("N25 A%d read failed: %s" % (a, e))

        log("---- 3. configured modules (read-only) ----")
        for cfg in (ADC_GAMMA, ADC_NEUTRON):
            slot = cfg["slot"]
            try:
                ctrl, q, x = cc.naf_read(slot, 0, 6)
                hit, _qh, _xh = cc.naf_read(slot, 1, 6)
                txt = "control 0x%04X (PED=%d LT=%d UT=%d) Q=%d X=%d" % (
                    ctrl & 0xFFFF, ctrl & 1, (ctrl >> 1) & 1, (ctrl >> 2) & 1, q, x)
                log("7164 N=%-2d %s | hit register 0x%04X%s" % (
                    slot, txt, hit & 0xFFFF, "" if x else "   <-- X=0: nothing answered in this slot"))
                summary["7164 N=%d" % slot] = txt
            except CCUSBError as e:
                log("7164 N=%d read failed: %s" % (slot, e))
        if ENABLE_TDC:
            try:
                ctrl, q, x = cc.naf_read(TDC_MAIN["slot"], 0, 6)
                log("7186 N=%-2d control 0x%04X Q=%d X=%d" % (TDC_MAIN["slot"], ctrl & 0xFFFF, q, x))
            except CCUSBError as e:
                log("7186 read failed: %s" % e)

        log("---- 4. USB reliability: %d direct reads of N25 A1 ----" % stress_n)
        times = []
        fails = 0
        r0 = cc.resyncs
        for _ in range(stress_n):
            t = time.perf_counter()
            try:
                cc.naf_read(25, 1, 0, "stress read")
            except CCUSBError:
                fails += 1
            times.append(time.perf_counter() - t)
        times.sort()
        txt = "resyncs %d, failures %d | per command: median %.0f us, 99%% %.0f us, max %.1f ms" % (
            cc.resyncs - r0, fails, times[len(times) // 2] * 1e6,
            times[int(len(times) * 0.99)] * 1e6, times[-1] * 1e3)
        log(txt)
        summary["USB reliability"] = txt

        log("---- 5. raw sparse answers of real events (needs signals; clears the modules) ----")
        for cfg in (ADC_GAMMA, ADC_NEUTRON):
            try:
                got, s15 = sparse_dump(cc, cfg["slot"])
                summary["sparse dump N=%d" % cfg["slot"]] = "%d events, channel 15 %s" % (
                    got, "present" if s15 else "not seen")
            except CCUSBError as e:
                log("sparse dump N=%d failed: %s" % (cfg["slot"], e))

        if listtest_s > 0:
            log("---- 6. list mode for %.0f s ----" % listtest_s)
            sep = hardware_setup(cc, True)
            parser = ListParser(sep, ENABLE_TDC)
            c = collections.Counter()
            sizes = collections.Counter()
            dumps = []
            per_channel = collections.Counter()

            def handle(raw):
                c["transfers"] += 1
                c["bytes"] += len(raw)
                sizes[len(raw)] += 1
                words = struct.unpack_from("<%dH" % (len(raw) // 2), raw)
                if len(dumps) < 3:
                    dumps.append(hexwords(words, 40))
                for hits in parser.parse(words):
                    c["events_with_hits"] += 1
                    for slot, ch, _v in hits:
                        per_channel[(slot, ch)] += 1

            cc.action(1)
            t0 = time.time()
            checked = False
            started = "yes"
            while time.time() - t0 < listtest_s:
                n, raw = cc.read_bulk(LIST_MODE_TIMEOUT_MS)
                if n > 0:
                    handle(raw)
                elif n == 0 or n == USB_TIMEOUT_CODE:
                    c["idle_timeouts"] += 1
                else:
                    c["usb_error_%d" % n] += 1
                if not checked and c["transfers"] == 0 and time.time() - t0 > 3.0:
                    checked = True
                    try:
                        cc.execute([camac_cmd(25, 1, 0, True)], 2, 2, "direct command during acquisition", retries=0)
                        started = "NO (Action Register write with sub-address %d ignored)" % ACTION_SUBADDR
                        log("  -> the CC-USB still answers direct commands: the acquisition did not start. "
                            "Try ACTION_SUBADDR = %d" % (1 - ACTION_SUBADDR))
                        break
                    except CCUSBError:
                        started = "yes, but no data in the first 3 s"
                        log("  -> no answer to a direct command: acquisition running, no data yet "
                            "(no LAM from slots %d/%d?)" % (ADC_GAMMA["slot"], ADC_NEUTRON["slot"]))
            cc.action(0)
            after = cc.drain(quiet_reads=3, timeout_ms=200, max_seconds=5.0, sink=handle)
            log("transfers/bytes/errors: %s | bytes after stop %d" % (dict(c), after))
            log("transfer sizes (bytes: count), largest 8: %s" % sorted(sizes.items())[-8:])
            for slot in (ADC_GAMMA["slot"], ADC_NEUTRON["slot"]):
                log("accepted hits N=%-2d channels 0..15: %s" % (
                    slot, " ".join(str(per_channel[(slot, ch)]) for ch in range(16))))
            for line in parser.raw_report():
                log(line)
            log("parser: %s" % dict(parser.s))
            for d in dumps:
                log("transfer: " + d)
            summary["list mode started"] = started
            summary["list mode data"] = "%d transfers, %d events with hits, parser warnings %d" % (
                c["transfers"], c["events_with_hits"],
                parser.s["bad_buffers"] + parser.s["missing_separator"])
            summary["list mode channel 15 words"] = "gamma %d, neutron %d" % (
                parser.raw["gamma"][15], parser.raw["neutron"][15])
            hardware_cleanup(cc, True)
    finally:
        cc.close()
    log("==== SUMMARY (compare this block between the two CC-USB) ====")
    for k, v in summary.items():
        log("%-28s %s" % (k, v))
    log("diagnostics finished; log: %s" % (os.path.abspath(_session_log_path) if _session_log_path else "-"))


# ==================== GUI ====================


def main_gui():
    import tkinter as tk

    app = {"st": None, "acq": None, "running": False, "log_scale": False,
           "display": False, "gch": 4, "tch": 0, "dt_file": None, "dt_last": 0.0,
           "closing": False}
    gamma_w, gamma_h = 800, 400
    tdc_w, tdc_h = 800, 300
    grid_n, cell_w, cell_h = 4, 200, 150

    root = tk.Tk()
    root.title("CAMAC DAQ v10 (list mode / polled)")

    def report_tk_exception(exc, val, tb):
        log("[GUI] exception in Tk callback:\n" + "".join(traceback.format_exception(exc, val, tb)))

    root.report_callback_exception = report_tk_exception

    tk.Label(root, text=("Gamma N=%d | Neutron N=%d | Threshold=%d  |  CAMAC address shown in GUI  |  "
                         "Front panel = CAMAC + 1" % (ADC_GAMMA["slot"], ADC_NEUTRON["slot"], LOWER_THRESHOLD)),
             bg="lightblue", fg="darkblue", font=("Arial", 9, "bold"), pady=3).pack(fill=tk.X)

    cfg_frame = tk.Frame(root, bg="#e8e8e8")
    cfg_frame.pack(fill=tk.X, pady=2)
    tk.Label(cfg_frame, text="Run name:", bg="#e8e8e8").pack(side=tk.LEFT, padx=(6, 2))
    run_name = tk.Entry(cfg_frame, width=18)
    run_name.insert(0, DEFAULT_RUN_NAME)
    run_name.pack(side=tk.LEFT, padx=2)
    tk.Label(cfg_frame, text="-> folder per run (cobalt, cobalt_2, ...)", bg="#e8e8e8", fg="gray",
             font=("Arial", 8)).pack(side=tk.LEFT, padx=4)
    list_var = tk.BooleanVar(value=ENABLE_LIST_MODE)
    list_check = tk.Checkbutton(cfg_frame, text="List mode (fast)  |  unchecked = polled",
                                variable=list_var, bg="#e8e8e8")
    list_check.pack(side=tk.LEFT, padx=12)

    disp = tk.Frame(root, bg="black")
    disp.pack(fill=tk.BOTH, expand=True)
    left = tk.Frame(disp, bg="black")
    left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    gamma_canvas = tk.Canvas(left, width=gamma_w, height=gamma_h, bg="black")
    gamma_canvas.pack()
    tdc_canvas = tk.Canvas(left, width=tdc_w, height=tdc_h, bg="black")
    if ENABLE_TDC:
        tdc_canvas.pack()
    right = tk.Frame(disp, bg="black")
    right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
    tk.Label(right, text="Neutron Wall  (C=CAMAC addr / FP=front panel ch)", bg="black", fg="white",
             font=("Arial", 8, "bold")).pack()
    grid_canvas = tk.Canvas(right, width=grid_n * cell_w, height=grid_n * cell_h, bg="black")
    grid_canvas.pack()

    # ---- drawing ----
    def draw_hist(canvas, counts, width, height, title, xlabel, color):
        canvas.delete("all")
        ml, mb, mt, mr = 55, 45, 30, 20
        pw, ph = width - ml - mr, height - mt - mb
        canvas.create_rectangle(ml, mt, ml + pw, mt + ph, outline="white")
        full = np.asarray(counts, dtype=np.int64)
        b = full.reshape(DISPLAY_BINS, -1).sum(axis=1)
        bd = np.log10(b + 1.0) if app["log_scale"] else b.astype(float)
        maxc = bd.max() or 1.0
        bw = pw / len(bd)
        for i in np.nonzero(bd)[0]:
            h = bd[i] / maxc * ph
            canvas.create_rectangle(ml + i * bw, mt + ph - h, ml + (i + 1) * bw, mt + ph, fill=color, outline="")
        for tv in X_TICKS:
            x = ml + (tv - VMIN) / (VMAX - VMIN) * pw
            canvas.create_line(x, mt + ph, x, mt + ph + 4, fill="white")
            canvas.create_text(x, mt + ph + 14, text=str(tv), fill="white", font=("Arial", 8))
        for tv in X_TICKS[1:-1]:
            x = ml + (tv - VMIN) / (VMAX - VMIN) * pw
            canvas.create_line(x, mt, x, mt + ph, fill="#333333", dash=(2, 4))
        canvas.create_text(width // 2, 15, text=title, fill="white", font=("Arial", 11, "bold"))
        canvas.create_text(width // 2, height - 8, text=xlabel, fill="white", font=("Arial", 9))
        canvas.create_text(ml + pw - 5, mt + 15, text="N=%d" % int(full.sum()), fill="yellow", anchor="e")
        if full.any():
            canvas.create_text(ml + pw - 5, mt + 30, text="Peak ch~%d" % int(full.argmax()), fill="cyan",
                               anchor="e", font=("Arial", 9))

    def draw_all():
        st = app["st"]
        if not app["display"] or st is None:
            return
        g = app["gch"]
        draw_hist(gamma_canvas, st.hist[ADC_GAMMA["slot"]][g], gamma_w, gamma_h,
                  "Gamma N=%d  CAMAC ch%d  (front panel ch%d)" % (ADC_GAMMA["slot"], g, g + FP_OFFSET),
                  "ADC channel", "cyan")
        if ENABLE_TDC:
            t = app["tch"]
            draw_hist(tdc_canvas, st.hist[TDC_MAIN["slot"]][t], tdc_w, tdc_h,
                      "TDC N=%d ch%d (raw)" % (TDC_MAIN["slot"], t), "TDC channel (raw)", "magenta")
        grid_canvas.delete("all")
        for row in range(grid_n):
            for col in range(grid_n):
                ch = row * grid_n + col
                x0, y0 = col * cell_w, row * cell_h
                x1, y1 = x0 + cell_w, y0 + cell_h
                grid_canvas.create_rectangle(x0, y0, x1, y1, outline="gray")
                grid_canvas.create_text(x0 + 5, y0 + 5, text="C%d/FP%d" % (ch, ch + FP_OFFSET), fill="white",
                                        anchor="nw", font=("Arial", 7))
                full = np.asarray(st.hist[ADC_NEUTRON["slot"]][ch], dtype=np.int64)
                tot = int(full.sum())
                if tot:
                    b = full.reshape(64, -1).sum(axis=1)
                    maxc = b.max() or 1
                    hh = cell_h - 30
                    bw = (cell_w - 10) / 64
                    for i in np.nonzero(b)[0]:
                        h = b[i] / maxc * hh
                        grid_canvas.create_rectangle(x0 + 5 + i * bw, y1 - 5 - h, x0 + 5 + (i + 1) * bw, y1 - 5,
                                                     fill="lime", outline="")
                    grid_canvas.create_text(x1 - 5, y1 - 5, text=str(tot), fill="yellow", anchor="se",
                                            font=("Arial", 8))

    # ---- buttons ----
    btn = tk.Frame(root)
    btn.pack(fill=tk.X, pady=2)
    info = tk.Label(root, text="", font=("Arial", 9))
    dt_label = tk.Label(root, text="Dead time: -", font=("Arial", 9, "bold"), fg="green")
    usb_label = tk.Label(root, text="", font=("Arial", 9), fg="gray")
    scaler_label = tk.Label(root, text="", font=("Arial", 9, "bold"), fg="gray")

    def on_start():
        if app["running"]:
            return
        st = RunState(bool(list_var.get()), run_name.get())
        app["st"] = st
        acq = Acquisition(st)
        app["acq"] = acq
        app["running"] = True
        app["dt_file"] = None
        list_check.config(state=tk.DISABLED)
        run_name.config(state=tk.DISABLED)
        log("[GUI] START (%s)" % ("LIST" if st.list_mode else "POLLED"))
        acq.start()

    def on_stop():
        st = app["st"]
        if not app["running"] or st is None or st.stop_event.is_set():
            return
        log("[GUI] STOP requested")
        st.stop_event.set()
        if st.list_mode and st.cc is not None:
            st.cc.action_async(0)  # wakes the pending bulk_read

    def finish_run():
        st = app["st"]
        app["running"] = False
        list_check.config(state=tk.NORMAL)
        run_name.config(state=tk.NORMAL)
        if app["dt_file"]:
            try:
                app["dt_file"].close()
            except Exception:
                pass
            app["dt_file"] = None
        w = st.writer
        msg = "Run finished: %d events, %d hits" % (st.trigger_count, st.hit_count)
        if w is not None:
            msg += ", %d files saved, %d save errors" % (w.saved_files, w.errors)
        if st.stop_reason:
            msg += " | stopped by itself: %s" % st.stop_reason
        log("[GUI] " + msg)
        usb_label.config(text=msg, fg="red" if (st.stop_reason or (w and w.errors)) else "darkgreen")

    def toggle_display():
        app["display"] = not app["display"]
        disp_btn.config(text="Display: %s" % ("ON (SLOW)" if app["display"] else "OFF (FAST)"),
                        bg="#ff6b6b" if app["display"] else "#51cf66")
        draw_all()

    def clear_display():
        st = app["st"]
        if st is not None:
            for s in st.hist:
                st.hist[s] = [[0] * NBINS for _ in range(16)]
        log("[CLEAR] display histograms cleared")
        draw_all()

    def toggle_log():
        app["log_scale"] = not app["log_scale"]
        draw_all()

    tk.Button(btn, text="START", command=on_start, bg="green", fg="white", font=("Arial", 10, "bold"),
              width=8).pack(side=tk.LEFT, padx=4)
    tk.Button(btn, text="STOP", command=on_stop, bg="red", fg="white", font=("Arial", 10, "bold"),
              width=8).pack(side=tk.LEFT, padx=4)
    tk.Button(btn, text="LOG/LIN", command=toggle_log, bg="blue", fg="white", width=8).pack(side=tk.LEFT, padx=4)
    disp_btn = tk.Button(btn, text="Display: OFF (FAST)", command=toggle_display, bg="#51cf66", fg="white",
                         font=("Arial", 9, "bold"), width=16)
    disp_btn.pack(side=tk.LEFT, padx=4)
    tk.Button(btn, text="Clear Display", command=clear_display, bg="orange", fg="white",
              width=12).pack(side=tk.LEFT, padx=4)

    tk.Label(btn, text="Gamma CAMAC ch:").pack(side=tk.LEFT, padx=4)
    fp_label = tk.Label(btn, text="", fg="gray", font=("Arial", 8))

    def on_gch(event=None):
        try:
            ch = int(gspin.get())
            if ch in ADC_GAMMA["channels"]:
                app["gch"] = ch
                fp_label.config(text="= FP ch%d" % (ch + FP_OFFSET))
                draw_all()
        except ValueError:
            pass

    gspin = tk.Spinbox(btn, values=ADC_GAMMA["channels"], width=3, command=on_gch)
    gspin.pack(side=tk.LEFT)
    gspin.delete(0, tk.END)
    gspin.insert(0, str(app["gch"]))
    gspin.bind("<Return>", on_gch)
    fp_label.config(text="= FP ch%d" % (app["gch"] + FP_OFFSET))
    fp_label.pack(side=tk.LEFT)

    if ENABLE_TDC:
        tk.Label(btn, text="TDC ch:").pack(side=tk.LEFT, padx=4)

        def on_tch(event=None):
            try:
                ch = int(tspin.get())
                if ch in TDC_MAIN["channels"]:
                    app["tch"] = ch
                    draw_all()
            except ValueError:
                pass

        tspin = tk.Spinbox(btn, values=TDC_MAIN["channels"], width=3, command=on_tch)
        tspin.pack(side=tk.LEFT)
        tspin.bind("<Return>", on_tch)

    info.pack()
    dt_label.pack()
    usb_label.pack()
    if ENABLE_SCALER:
        scaler_label.pack()

    # ---- periodic update ----
    def compute_deadtime(st):
        elapsed = max(time.time() - st.t0, 1e-9)
        tau = st.tau()
        n = st.trigger_count / elapsed
        nt = min(n * tau, 0.9999)
        return tau * 1e6, n, n / (1.0 - nt), nt * 100.0

    def dt_log_tick(st):
        if st.run_folder is None or st.t0 is None or not st.tau_converged():
            return
        if app["dt_file"] is None:
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(st.run_folder, "deadtime_%s.csv" % ts)
            try:
                app["dt_file"] = open(path, "w")
                app["dt_file"].write("elapsed_s,rate_obs_cps,rate_true_cps,tau_us,deadtime_pct,"
                                     "n_daq,n_scaler,dt_hw_pct\n")
                log("[DT-LOG] %s" % path)
            except Exception as e:
                log("[DT-LOG] could not open: %s" % e)
                app["dt_file"] = False
        if not app["dt_file"]:
            return
        now = time.time()
        if now - app["dt_last"] < DT_LOG_INTERVAL_S:
            return
        app["dt_last"] = now
        tau_us, n_obs, m_true, dt_pct = compute_deadtime(st)
        if dt_pct >= 95.0:
            return
        dt_hw = 0.0
        if ENABLE_SCALER and st.scaler_total > 0:
            dt_hw = max(0.0, (1.0 - st.trigger_count / st.scaler_total) * 100.0)
        try:
            app["dt_file"].write("%.1f,%.3f,%.3f,%.1f,%.4f,%d,%d,%.4f\n" % (
                now - st.t0, n_obs, m_true, tau_us, dt_pct, st.trigger_count, st.scaler_total, dt_hw))
            app["dt_file"].flush()
        except Exception as e:
            log("[DT-LOG] write error: %s" % e)

    def update_labels(st):
        if st.t0 is None:
            info.config(text="Opening CC-USB and configuring modules...", fg="gray")
            return
        elapsed = max(time.time() - st.t0, 1e-9)
        w = st.writer
        saved = ("%d files" % w.saved_files) if w else "-"
        info.config(text=("Triggers: %d (%.1f/s) | Hits: %d (%.1f/s) | in memory: %d events | saved: %s | dir: %s" % (
            st.trigger_count, st.trigger_count / elapsed, st.hit_count, st.hit_count / elapsed,
            st.chunk.n_events, saved, st.run_folder)), fg="blue")
        if not st.tau_converged():
            dt_label.config(text="Dead time: calibrating tau...", fg="gray")
        else:
            tau_us, n_obs, m_true, dt_pct = compute_deadtime(st)
            if dt_pct >= 95.0:
                dt_label.config(text="tau=%.1fus | Obs:%.1fcps | SATURATED (%.1f%%)" % (tau_us, n_obs, dt_pct), fg="red")
            else:
                col = "green" if dt_pct < 5 else ("orange" if dt_pct < 20 else "red")
                dt_label.config(text="tau=%.1fus | Obs:%.1fcps | True:%.1fcps | DT:%.2f%%" % (
                    tau_us, n_obs, m_true, dt_pct), fg=col)
        cc = st.cc
        if st.list_mode:
            ps = st.parser.s if st.parser else {}
            warn = ps.get("bad_buffers", 0) + ps.get("missing_separator", 0)
            usb_label.config(text=("USB: transfers %d | idle timeouts %d | errors %d | recoveries %d | resyncs %d | "
                                   "parse warnings %d" % (st.transfers, st.idle_timeouts, st.usb_errors,
                                                          st.recoveries, cc.resyncs if cc else 0, warn)),
                             fg="red" if (st.usb_errors or warn) else "gray")
        else:
            usb_label.config(text="USB: failed readouts %d | resyncs %d | readout %.0f us" % (
                st.polled_errors, cc.resyncs if cc else 0, st.measured_tau * 1e6),
                fg="red" if st.polled_errors else "gray")
        if ENABLE_SCALER:
            if st.list_mode:
                scaler_label.config(text="List mode: scaler not readable during autonomous acquisition", fg="gray")
            elif st.scaler_total > 0:
                dt_hw = max(0.0, (1.0 - st.trigger_count / st.scaler_total) * 100.0)
                scaler_label.config(text="Scaler N=%d: %d | DAQ: %d | HW DT: %.2f%%" % (
                    SCALER["slot"], st.scaler_total, st.trigger_count, dt_hw), fg="green")

    def gui_loop():
        try:
            st = app["st"]
            if st is not None and app["running"]:
                update_labels(st)
                dt_log_tick(st)
                draw_all()
                if st.state == "stopped" and not (app["acq"] and app["acq"].is_alive()):
                    finish_run()
            if app["closing"] and not app["running"]:
                root.quit()
                return
        except Exception:
            log("[GUI] update error:\n" + traceback.format_exc())
        root.after(DISPLAY_UPDATE_INTERVAL_MS, gui_loop)

    def on_close():
        if app["running"]:
            app["closing"] = True
            on_stop()
            usb_label.config(text="Stopping and saving before closing...", fg="red")
            root.after(30000, root.quit)  # hard limit
        else:
            root.quit()

    root.protocol("WM_DELETE_WINDOW", on_close)
    signal.signal(signal.SIGINT, lambda s, f: root.after(0, on_close))
    root.after(DISPLAY_UPDATE_INTERVAL_MS, gui_loop)
    root.mainloop()
    try:
        root.destroy()
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="CAMAC DAQ v10 (CC-USB)")
    ap.add_argument("--diag", action="store_true", help="run diagnostics in the console (no GUI)")
    ap.add_argument("--listtest", type=float, default=0.0, help="with --diag: seconds of list mode")
    ap.add_argument("--stress", type=int, default=3000, help="with --diag: number of direct reads")
    args = ap.parse_args()
    setup_logging("diag" if args.diag else "daq")
    if args.diag:
        run_diagnostics(args.listtest, args.stress)
        return
    environment_report()
    main_gui()


if __name__ == "__main__":
    main()
