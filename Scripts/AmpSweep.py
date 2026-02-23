#!/usr/bin/env python3
# Cubby DeBry 2/2026 via Codex 5.3
"""
Batch amplitude extraction from stepped-amplitude recordings at fixed frequency (e.g., 1 Hz).

Assumptions:
- Files are named like "0.1V.txt", "1V.txt", "2.5V.txt", "20V.txt" (case-insensitive),
  or split files like "..._2.5_Vpp.csv".
- By default, files use columns (1-based):
  raw=1, temporal=5, spatial=9, combined=10, timestamp=11.
  Other columns are ignored.
- If timestamps look like microseconds, they are converted to seconds automatically.
- Amplitude is extracted via coherent least-squares fit at fixed frequency f0:
    x(t) ≈ a*sin(2π f0 t) + b*cos(2π f0 t)
    A_peak = sqrt(a^2 + b^2)

Outputs:
- amplitude_summary_vs_inputV.csv
- amplitude_vs_inputV.png (300 DPI), with log x-axis

Usage:
  python amp_sweep.py --folder /path/to/folder --freq 1
  python amp_sweep.py --folder . --freq 1 --discard_frac 0.25
  python amp_sweep.py --folder . --freq 1 --pattern "*.csv" --fs 60   # only if no timestamp column
"""

from __future__ import annotations
import glob
import os
import re
from typing import Optional, Tuple, List, Dict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


V_RE = re.compile(r"(?P<v>\d+(?:\.\d+)?)(?=\s*v(?:pp)?\b)", re.IGNORECASE)
VPP_UNDERSCORE_RE = re.compile(r"_(?P<v>\d+(?:\.\d+)?)_vpp(?:\.[^.]+)?$", re.IGNORECASE)

# Config
FOLDER = "Scripts/Plot Data/Amplitude Sweep/Split"
FREQ_HZ = 10.0
PATTERN = "*.csv"
SIGNAL_COLS_1B = [1, 5, 9, 10]
TIME_COL_1B = 11
FS_FALLBACK: Optional[float] = None
EXPECTED_COLS: Optional[int] = None
MAX_REL_JUMP = 0.5
DISCARD_FRAC = 0.25
OUT_CSV = "amplitude_summary_vs_inputV.csv"
OUT_QC_CSV = "row_qc_summary_vs_inputV.csv"
OUT_PNG = "amplitude_vs_inputV.png"

# Measured Mag Field, Voltage
mag_field = np.array([  # Volts, mT
    [3.012,     0.21],
    [3.526,     0.24],
    [4.128,     0.26],
    [4.832,     0.33],
    [5.6,       0.39],
    [6.622,     0.44],
    [7.7,       0.52],
    [9.0,       0.62],
    [10.624,    0.72],
    [12.43,     0.85],
    [14.559,    0.99],
    [17.044,    1.15],
    [19.95,     1.35]
])

def fit_voltage_to_bfield(calibration_v_mt: np.ndarray) -> Tuple[float, float, float]:
    """Fit B[mT] = slope*V + intercept. Returns (slope, intercept, r2)."""
    cal = np.asarray(calibration_v_mt, dtype=np.float64)
    if cal.ndim != 2 or cal.shape[1] != 2 or cal.shape[0] < 2:
        raise ValueError("mag_field must be Nx2 with columns [V, mT] and at least 2 rows.")
    v = cal[:, 0]
    b = cal[:, 1]
    slope, intercept = np.polyfit(v, b, 1)
    b_hat = slope * v + intercept
    ss_res = float(np.sum((b - b_hat) ** 2))
    ss_tot = float(np.sum((b - np.mean(b)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan
    return float(slope), float(intercept), float(r2)

def parse_volts_from_filename(path: str) -> Optional[float]:
    base = os.path.basename(path)
    m = VPP_UNDERSCORE_RE.search(base)
    if not m:
        m = V_RE.search(base)
    if not m:
        return None
    return float(m.group("v"))


def load_table(
    path: str,
    selected_cols: List[int],
    expected_cols: Optional[int] = None,
    max_rel_jump: float = 0.5,
) -> Tuple[np.ndarray, Dict[str, int]]:
    rows: List[List[float]] = []
    prev_row: Optional[List[float]] = None
    max_col = max(selected_cols) if selected_cols else -1

    n_short = 0
    n_long = 0
    n_non_numeric = 0
    n_jump = 0

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    for line in lines:
        s = line.strip()
        if not s:
            continue

        fields = [v.strip() for v in s.split(",")] if "," in s else s.split()
        fields = [v for v in fields if v]
        if expected_cols is not None:
            if len(fields) < expected_cols:
                n_short += 1
                continue
            if len(fields) > expected_cols:
                n_long += 1
                continue

        try:
            row_all = [float(v) for v in fields]
        except ValueError:
            n_non_numeric += 1
            continue

        if len(row_all) <= max_col:
            n_short += 1
            continue

        row = [row_all[i] for i in selected_cols]

        if prev_row is not None:
            bad_jump = False
            for i in range(len(row)):
                denom = max(abs(prev_row[i]), 1.0)
                rel_delta = abs(row[i] - prev_row[i]) / denom
                if rel_delta > max_rel_jump:
                    bad_jump = True
                    break
            if bad_jump:
                n_jump += 1
                continue

        rows.append(row)
        prev_row = row

    if not rows:
        raise ValueError(f"No valid data rows after QC in {path}")

    qc = {
        "rejected_short_cols": n_short,
        "rejected_long_cols": n_long,
        "rejected_non_numeric": n_non_numeric,
        "rejected_jump": n_jump,
        "kept_rows": len(rows),
    }
    return np.asarray(rows, dtype=np.float64), qc


def infer_time_seconds(arr: np.ndarray, fs_fallback: Optional[float]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns:
      t (seconds), sig (Nx4)
    Logic:
      - If >=5 columns, assume last column is timestamp.
      - Else, require fs_fallback to synthesize t from sample index.
    """
    if arr.shape[1] < 4:
        raise ValueError(f"Need at least 4 columns. Got {arr.shape[1]}")

    sig = arr[:, :4].astype(np.float64)

    if arr.shape[1] >= 5:
        ts = arr[:, -1].astype(np.float64)

        d = np.diff(ts)
        if np.nanmedian(d) <= 0:
            if fs_fallback is None:
                raise ValueError("Timestamp not monotonic; provide --fs to use sample-index time.")
            t = np.arange(len(sig), dtype=np.float64) / fs_fallback
            return t, sig

        dt_med = float(np.nanmedian(d))

        # unit heuristic
        if dt_med > 1_000.0:
            scale = 1e-6  # us -> s
        elif dt_med > 1.0:
            scale = 1e-3  # ms -> s
        else:
            scale = 1.0   # s
        t = (ts - ts[0]) * scale
        return t, sig

    if fs_fallback is None:
        raise ValueError("Data has no timestamp column; provide --fs.")
    t = np.arange(len(sig), dtype=np.float64) / fs_fallback
    return t, sig


def coherent_amp_peak(t: np.ndarray, x: np.ndarray, f_hz: float) -> float:
    """Coherent least-squares peak amplitude at known frequency."""
    x = x.astype(np.float64)
    x = x - np.nanmean(x)

    w = 2.0 * np.pi * f_hz
    s = np.sin(w * t)
    c = np.cos(w * t)
    X = np.column_stack([s, c])

    coeffs, _, _, _ = np.linalg.lstsq(X, x, rcond=None)
    a, b = coeffs
    return float(np.sqrt(a * a + b * b))


def main() -> int:
    if TIME_COL_1B < 1:
        raise SystemExit("TIME_COL_1B must be >= 1")
    signal_cols_1b = SIGNAL_COLS_1B
    signal_cols_0b = [c - 1 for c in signal_cols_1b]
    time_col_0b = TIME_COL_1B - 1
    selected_cols = [*signal_cols_0b, time_col_0b]

    files = sorted(glob.glob(os.path.join(FOLDER, PATTERN)))
    if not files:
        raise SystemExit(f"No files matched: {os.path.join(FOLDER, PATTERN)}")

    rows = []
    skipped = []
    qc_by_file = {}

    for path in files:
        vin = parse_volts_from_filename(path)
        if vin is None:
            skipped.append(os.path.basename(path))
            continue

        arr, qc = load_table(
            path,
            selected_cols=selected_cols,
            expected_cols=EXPECTED_COLS,
            max_rel_jump=MAX_REL_JUMP,
        )
        qc_by_file[os.path.basename(path)] = qc
        t, sig = infer_time_seconds(arr, FS_FALLBACK)

        # discard settling
        if DISCARD_FRAC > 0:
            n0 = int(len(t) * DISCARD_FRAC)
            if n0 >= len(t) - 20:
                continue
            t = t[n0:] - t[n0]
            sig = sig[n0:, :]

        if len(t) < 20:
            continue

        amps = [coherent_amp_peak(t, sig[:, ch], FREQ_HZ) for ch in range(4)]
        rows.append([vin, *amps])

    if not rows:
        raise SystemExit("No usable files found (did filenames include 'V'?)")

    rows.sort(key=lambda r: r[0])
    out = pd.DataFrame(rows, columns=["input_v", "amp1_peak", "amp2_peak", "amp3_peak", "amp4_peak"])
    b_slope, b_intercept, b_r2 = fit_voltage_to_bfield(mag_field)
    out["input_b_mT"] = b_slope * out["input_v"] + b_intercept

    out_csv_path = os.path.join(FOLDER, OUT_CSV)
    out.to_csv(out_csv_path, index=False)

    qc_rows = []
    for name in sorted(qc_by_file.keys()):
        q = qc_by_file[name]
        dropped = q["rejected_short_cols"] + q["rejected_long_cols"] + q["rejected_non_numeric"] + q["rejected_jump"]
        total_seen = q["kept_rows"] + dropped
        drop_frac = (dropped / total_seen) if total_seen > 0 else np.nan
        qc_rows.append(
            {
                "file": name,
                "kept_rows": q["kept_rows"],
                "dropped_rows": dropped,
                "dropped_frac": drop_frac,
                "rejected_short_cols": q["rejected_short_cols"],
                "rejected_long_cols": q["rejected_long_cols"],
                "rejected_non_numeric": q["rejected_non_numeric"],
                "rejected_jump": q["rejected_jump"],
            }
        )

    qc_df = pd.DataFrame(qc_rows)
    out_qc_csv_path = os.path.join(FOLDER, OUT_QC_CSV)
    qc_df.to_csv(out_qc_csv_path, index=False)

    # Plot: amplitude vs injected voltage (log x)
    plt.figure(figsize=(8, 6), dpi=300)
    plt.plot(out["input_b_mT"], out["amp1_peak"], marker="o", linewidth=1, label="Raw Output (Single Sensor)")
    plt.plot(out["input_b_mT"], out["amp2_peak"], marker="o", linewidth=1, label="Time Averaged (Single Sensor)")
    plt.plot(out["input_b_mT"], out["amp3_peak"], marker="o", linewidth=1, label="Spatial Average (Four Sensors)")
    plt.plot(out["input_b_mT"], out["amp4_peak"], marker="o", linewidth=1, label="Combined Average (Four Sensors)")

    plt.xscale("log")
    plt.xlabel("Magnetic Flux Density (mT)")
    plt.yscale("log")
    plt.ylabel("Measured Peak Amplitude (ADC counts)")
    plt.title(
        f"Measured Amplitude vs Magnetic Flux Density @ {FREQ_HZ:g} Hz (Coherent Fit)\n"
        f"B[mT] = {b_slope:.5g}*V + {b_intercept:.5g} (R^2={b_r2:.4f}) (Measured linear fit)"
    )
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()

    out_png_path = os.path.join(FOLDER, OUT_PNG)
    plt.savefig(out_png_path, dpi=300)
    plt.close()

    if skipped:
        print("Skipped (couldn't parse input volts from filename):")
        for s in skipped:
            print("  ", s)

    print("Row QC summary:")
    for name in sorted(qc_by_file.keys()):
        q = qc_by_file[name]
        rejected_total = q["rejected_short_cols"] + q["rejected_long_cols"] + q["rejected_non_numeric"] + q["rejected_jump"]
        if rejected_total == 0:
            continue
        print(
            f"  {name}: kept={q['kept_rows']} dropped={rejected_total} "
            f"(short={q['rejected_short_cols']}, long={q['rejected_long_cols']}, "
            f"nonnum={q['rejected_non_numeric']}, jump={q['rejected_jump']})"
        )

    print("Wrote:", out_csv_path)
    print("Wrote:", out_qc_csv_path)
    print("Wrote:", out_png_path)
    print(f"Voltage->B fit: B[mT] = {b_slope:.6g}*V + {b_intercept:.6g} (R^2={b_r2:.6f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
