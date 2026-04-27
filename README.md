# Python-Based CAMAC Data Acquisition System

A complete Python DAQ system for CAMAC (IEEE 583) nuclear instrumentation using the Wiener CC-USB controller. Supports autonomous list-mode acquisition with multi-event buffering, multi-module Q-stop readout, real-time dead time characterization, and offline coincidence analysis.

Built and tested at a 5.5 MV Van de Graaff accelerator laboratory for gamma/alpha spectroscopy and neutron time-of-flight experiments.

## Features

- **Autonomous list-mode acquisition** — the CC-USB FPGA executes the readout stack on every LAM trigger with no host intervention, achieving τ = 192 ± 5 µs dead time per event
- **Multi-event USB buffering** — packs ~13 events per USB transfer, sustaining up to 5200 triggers/s before saturation
- **Multi-module Q-stop readout** — drains the full sparse FIFO from each module in a single stack execution: two Phillips 7164 ADCs (32 channels) + one Phillips 7186 TDC (16 channels)
- **Real-time dead time model** — non-paralyzable model with live τ measurement, DT% display, and CSV logging for offline fits
- **Event-by-event list-mode storage** — NumPy structured arrays (.npy) with event ID, timestamp, slot, channel, and amplitude for every hit
- **Offline coincidence analysis** — Jupyter notebook with interactive Plotly plots: gated spectra, 2D correlations, KDE overlays, and multi-gate comparisons
- **Tkinter GUI** with live histograms, 4×4 neutron wall grid, channel selector, log/lin toggle, and auto-save

## Hardware

| Module | Slot | Role |
|--------|------|------|
| Phillips 7164 16-ch Peak-Sensing ADC | 17 | Gamma detectors |
| Phillips 7164 16-ch Peak-Sensing ADC | 19 | Neutron wall / alpha detector |
| Phillips 7186 16-ch TDC | 21 | Time-of-flight / coincidence timing |
| Polon 420A Scaler | 22 | Gate pulse counting (optional) |
| Wiener CC-USB | 24–25 | CAMAC controller (USB 2.0) |

### Controller

Wiener CC-USB (firmware ≥ 0x0600 recommended, 0x0603 for all features). Communicates via `libxxusb.dll` (32-bit) through `ctypes`. The DLL's `xxusb_stack_write` and `xxusb_register_write` return -116 on some firmware versions — replaced with raw `xxusb_bulk_write` packets throughout.

### Crate Compatibility

Tested on standard IEEE 583 CAMAC crates (25-station full-size and LeCroy 8013A 11-position mini crate). The CC-USB must occupy the control station (rightmost two positions). **Not compatible** with Data Design Corporation SCSICrate (internal hardwired SCSI controller occupies the control station).

## Software Requirements

- **Python 3.11 (32-bit)** — required because `libxxusb.dll` is a 32-bit PE binary
- **Windows** — the CC-USB driver and DLL are Windows-only
- **NumPy** — event storage and offline analysis
- **Plotly** — interactive coincidence plots (offline analysis only)
- **SciPy** — KDE smoothing for low-statistics spectra (optional)

## Files

| File | Description |
|------|-------------|
| `8list.py` | Main DAQ application — list-mode, multi-event, dual ADC + TDC, tkinter GUI |
| `coincidence.ipynb` | Jupyter notebook for offline coincidence analysis with gating |
| `restart.py` | Hardware rescue script — stops list mode, resets all registers, flushes USB FIFO |
| `slot_scanner.py` | Scans all CAMAC slots and identifies modules by type |
| `brute_scanner.py` | Aggressive scanner — tries N=1–30 with 6 CAMAC functions each |
| `web_daq.py` | Flask + SocketIO web-based DAQ server (replaces tkinter for remote access) |

## Quick Start

1. Install Python 3.11 (32-bit) and ensure `libxxusb.dll` is in `C:\Windows\SysWOW64\`
2. Connect the CC-USB via USB, power on the crate
3. Edit the configuration section at the top of `8list.py`:

```python
SERIAL      = b"CC0284"          # Your CC-USB serial number
ADC_GAMMA   = {"slot": 17, ...}  # Adjust slot numbers to your crate
ADC_NEUTRON = {"slot": 19, ...}
TDC_MAIN    = {"slot": 21, ...}
ENABLE_TDC  = True               # False if no TDC connected
```

4. Run: `python 8list.py`
5. Click **START** — the console shows live event rates, dead time, and buffer diagnostics
6. Click **STOP** — saves all events to `run_YYYYMMDD_HHMMSS_final.npy`

## List-Mode Architecture

The CC-USB autonomous stack executes on every LAM trigger without host intervention:

```
ADC_GAMMA Q-stop  →  N25 marker  →  ADC_NEUTRON Q-stop  →  N25 marker  →  TDC Q-stop
```

Each Q-stop drains the module's sparse FIFO completely (F(4)A(0) repeated until Q=0). The N25 firmware-ID read (`0x0600 0x7C00`) acts as a reliable section separator in the data stream. The TDC's sparse read self-clears its Hit Register — no separate clear command needed.

Multi-event mode (`Global Mode 0x0104`) packs multiple events per 512-word USB buffer. A 2.5 ms timeout flushes partial buffers at low rates. The host thread drains buffers via `xxusb_bulk_read` and parses them into per-event hit lists using marker-counting:

- Section 0 (before first marker) → gamma ADC
- Section 1 (between markers) → neutron ADC
- Section 2 (after second marker) → TDC

## Dead Time Characterization

Measured with a Berkeley precision pulse generator sweeping from 100 Hz to 20 kHz:

- **Below 3500 Hz**: zero dead time, trigger/input ratio = 1.02 ± 0.04
- **Above 4000 Hz**: non-paralyzable model, **τ = 192 ± 5 µs**
- **Channel multiplicity**: 14.01 ± 0.02 across all rates
- **Maximum sustainable rate**: 1/τ ≈ 5200 Hz (73,000 channel-reads/s)
- **Bottleneck**: USB 2.0 transport + CAMAC Q-stop execution (not Python)

## Coincidence Analysis

The `coincidence.ipynb` notebook loads `.npy` event files and provides:

- **Raw spectra** for all channels (alpha, gamma, TDC)
- **Single gates** — gate on any channel window, see gated spectra on all other channels
- **Double gates** — simultaneous alpha + gamma windows, see resulting TDC spectrum
- **2D correlations** — alpha vs gamma, energy vs timing heatmaps
- **KDE overlays** — smoothed spectra for low-statistics gated data
- **Multi-gate comparison** — overlay multiple alpha windows on the same gamma/TDC plot

Event matching uses the `event` field in the structured array. When loading multiple `.npy` files, event IDs are offset to prevent cross-file collisions.

## Known Hardware Quirks

- **Phillips 7164 ch0 duplicate**: the first two F(4) sparse reads return the same channel — caught by per-section `seen` sets in the parser
- **Phillips 7164 ch15 (A=15)**: not returned by F(4) sparse read — use direct F(0)A(15) in polled mode, or skip in list mode
- **Phillips 7186 TDC overflow**: channels without a stop signal return `0xFFF` (4095). With 16 channels read via Q-stop, `0xFFFF` (ch15 overflow) is valid data, not an event terminator
- **CC-USB Action Register**: `struct.pack('<3H', 5, 0, value)` writes `value` to register 0; the byte order `[5, 1, 0]` writes to register 1 instead (the original bricking bug)
- **XXUSBWin hex display**: slot numbers are shown in hexadecimal — hex 19 = decimal 25 = CC-USB internal registers

## Data Format

Events are saved as NumPy structured arrays:

```python
dtype = [
    ('event',   np.uint32),  # event ID (increments per trigger)
    ('time_s',  np.float64), # seconds since run start
    ('slot',    np.uint8),   # CAMAC slot number
    ('channel', np.uint8),   # CAMAC address (0-based; front panel = +1)
    ('value',   np.uint16),  # ADC/TDC value (0–4095)
]
```

## License

MIT

## Citation

If you use this software, please cite the Zenodo archive:

> DOI: [10.5281/zenodo.19157216](https://doi.org/10.5281/zenodo.19157216)
