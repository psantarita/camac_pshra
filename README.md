# CAMAC DAQ System

Multi-module CAMAC data acquisition system in Python via CC-USB. Reads Phillips 7164 ADCs and Phillips 7186 TDC with LAM polling, dead time monitoring (Gedcke-Hale model), live Tkinter histograms, and structured NumPy output. Designed for gamma-neutron coincidence experiments at a 5.5 MV Van de Graaff laboratory.

## Features

- **Multi-module readout** — simultaneous acquisition from Phillips 7164 ADCs (gamma detectors, neutron wall) and Phillips 7186 TDC (timing) via a CC-USB CAMAC controller
- **LAM polling** — optional Hit Register pre-check (F(6)A(1), ~50 µs) before full sparse read (~420 µs), dramatically reducing dead time at low-to-moderate count rates
- **Dead time monitoring** — real-time non-paralyzable (Gedcke-Hale) dead time correction with rolling τ estimate; color-coded display and CSV log output
- **Live GUI** — Tkinter interface with per-channel histogram display for gamma detectors, neutron wall (4×4 grid), and TDC; log/linear scale toggle
- **Structured NumPy output** — events saved as `.npy` files with fields: `event` (uint32), `time_s` (float64), `slot` (uint8), `channel` (uint8), `value` (uint16)
- **Auto-save** — configurable event-count threshold triggers automatic file segmentation during long runs
- **TDC values stored in raw channels** — time calibration performed externally using a hardware pulser

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| CC-USB | Wiener/ISEG CAMAC-USB controller |
| Phillips 7164 | 16-channel ADC (×2, gamma + neutron) |
| Phillips 7186 | 16-channel TDC |
| `libxxusb.dll` | CC-USB Windows driver library |

> The system runs on Windows due to the `libxxusb.dll` dependency.

## Installation

```bash
git clone https://github.com/yourusername/camac-daq.git
cd camac-daq
pip install numpy
```

No additional dependencies beyond the Python standard library and NumPy.

## Configuration

All settings are defined at the top of `daq.py`:

```python
SERIAL = b"CC0284"           # CC-USB serial number
ADC_GAMMA  = {"slot": 17, "channels": [14, 15], ...}
ADC_NEUTRON = {"slot": 19, "channels": list(range(16)), ...}
TDC_MAIN   = {"slot": 21, "channels": list(range(2)), ...}

ENABLE_TDC      = True       # set False to skip TDC readout
ENABLE_LAM_POLL = True       # enable Hit Register pre-check
LOWER_THRESHOLD = 50         # ADC/TDC lower threshold (raw ch)
AUTO_SAVE_INTERVAL = 1000000 # events between auto-saves
```

## Usage

```bash
python daq.py
```

1. Click **START** to initialize the CC-USB and begin acquisition
2. Click **Display: OFF (FAST)** to toggle live histograms (disable for maximum throughput)
3. Click **STOP** to end acquisition — a final `.npy` file is saved automatically
4. Use **LOG/LIN** to toggle histogram scale
5. Select the gamma or TDC channel to display using the spinbox controls

## Output Format

Each run produces one or more `.npy` files containing a structured NumPy array:

```python
import numpy as np
data = np.load("run_20250101_120000_final.npy")
# dtype: [('event', '<u4'), ('time_s', '<f8'), ('slot', 'u1'),
#         ('channel', 'u1'), ('value', '<u2')]
```

A dead time log CSV is also saved per run:

```
elapsed_s, rate_obs_cps, rate_true_cps, tau_us, deadtime_pct
```

## Dead Time Model

The system uses a **non-paralyzable (Type I / Gedcke-Hale)** dead time model:

$$\dot{m} = \frac{\dot{n}}{1 - \dot{n}\,\tau}$$

where $\dot{n}$ is the observed rate, $\tau$ is the mean readout cycle time (estimated from a rolling average), and $\dot{m}$ is the corrected true rate. The dead time percentage is displayed in real time and color-coded: green (<5%), orange (5–20%), red (>20%).

## Citation

If you use this software in your research, please cite:

```
[Paper citation — DOI to be added upon publication]
```

Software archive: [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.XXXXXXX.svg)](https://doi.org/10.5281/zenodo.XXXXXXX)

## License

MIT License — see [LICENSE](LICENSE) for details.
