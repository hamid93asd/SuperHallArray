# Cubby DeBry 2/13/2025 Via Codex 5.3
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import welch
from collections import Counter

# ---- user params ----
path = "log/4_sensor_28.4mT.txt"     # last column is timestamp
fs = 60.0                     # Hz
Bcal_mT = -31.2               # injected field
nperseg = 256
labels = {"Raw (Single Sensor)", "Spatial Average (4 Sensors)", "64x Temporal Average (Single Sensor)", "Combined Average (36 Sensors)"}

# Pick which column to use for calibration (name or index)
cal_col = 0   # use first signal column by default (0-based after dropping timestamp)

# If you only want to use a slice for calibration (e.g., field-on region), set these:
cal_start = None  # e.g. 1000
cal_stop  = None  # e.g. 5000
# ---------------------

def _split_fields(line):
    if "," in line:
        parts = [p.strip() for p in line.split(",")]
    else:
        parts = line.split()
    return [p for p in parts if p]


def load_numeric_log(file_path):
    rows = []
    field_counts = Counter()
    parse_errors = 0

    with open(file_path, "r", encoding="utf-8") as f:
        for line_num, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue

            fields = _split_fields(line)
            if not fields:
                continue

            try:
                values = [float(x) for x in fields]
            except ValueError:
                parse_errors += 1
                continue

            rows.append((line_num, values))
            field_counts[len(values)] += 1

    if not rows:
        raise ValueError("No numeric rows found in input file.")

    expected_cols = field_counts.most_common(1)[0][0]
    kept = [values for _, values in rows if len(values) == expected_cols]
    dropped = [ln for ln, values in rows if len(values) != expected_cols]

    if not kept:
        raise ValueError("No rows with consistent column count were found.")
    if expected_cols < 2:
        raise ValueError("Expected at least 2 columns (signals + timestamp).")

    if dropped or parse_errors:
        print(
            f"Parser warning: kept {len(kept)} rows with {expected_cols} columns, "
            f"dropped {len(dropped)} malformed-width rows and {parse_errors} parse-error rows."
        )
        if dropped:
            preview = ", ".join(str(n) for n in dropped[:10])
            more = "..." if len(dropped) > 10 else ""
            print(f"  Dropped line numbers (first 10): {preview}{more}")

    return np.asarray(kept, dtype=np.float64)


data = load_numeric_log(path)
sig = data[:, :-1]
num_channels = sig.shape[1]

if num_channels == 0:
    raise ValueError("No signal columns found after removing timestamp column.")
if not (0 <= cal_col < num_channels):
    raise IndexError(f"cal_col={cal_col} out of range for {num_channels} signal columns.")

# If your signals are actually *unsigned* ADC values, uncomment this to center them:
# midscale = 2**31  # or 2**15, etc. depending on your format
# sig = sig - midscale

# Calibration using mean value under applied field
xcal = sig[:, cal_col]
if cal_start is not None or cal_stop is not None:
    xcal = xcal[slice(cal_start, cal_stop)]

x_mean = np.nanmean(xcal)
if np.isclose(x_mean, 0.0, atol=1e-20):
    raise ValueError("Calibration mean is zero (or too close to zero), cannot compute scale factor.")

k_mT_per_count = Bcal_mT / x_mean  # mT per ADC unit

B = sig * k_mT_per_count  # mT

# ---- PSD in dB(mT^2/Hz) ----
plt.figure(figsize=(6.4, 4.8), dpi=100)
for i in range(num_channels):
    f, pxx = welch(B[:, i], fs=fs, nperseg=min(nperseg, len(B)))
    pxx_db = 10 * np.log10(np.maximum(pxx, np.finfo(float).tiny))  # units: dB(mT^2/Hz)
    plt.plot(f, pxx_db, label=f"ch{i}")

plt.xlabel("Frequency (Hz)")
plt.ylabel("PSD (dB mT$^2$/Hz)")
plt.title("Noise Reduction Methods (Flux Density Units)")
plt.legend(fontsize=7, loc="upper right", bbox_to_anchor=(1, 0.82))
plt.tight_layout()
plt.show()

# ---- Optional: ASD in nT/√Hz (often more intuitive) ----
plt.figure(figsize=(6.4, 4.8), dpi=100)
for i in range(num_channels):
    f, pxx = welch(B[:, i], fs=fs, nperseg=min(nperseg, len(B)))
    asd_nT = np.sqrt(pxx) * 1e6  # sqrt(mT^2/Hz)->mT/√Hz, then *1e6 => nT/√Hz
    plt.semilogy(f, asd_nT, label=f"ch{i}")

plt.xlabel("Frequency (Hz)")
plt.ylabel("ASD (nT/√Hz)")
plt.title("Noise Reduction Methods (Flux Density Units)")
plt.legend(fontsize=7, loc="upper right", bbox_to_anchor=(1, 0.82))
plt.tight_layout()
plt.show()

print("Calibration:")
print(f"  x_mean = {x_mean:g} (ADC units)")
print(f"  k = {k_mT_per_count:g} mT per ADC unit")
