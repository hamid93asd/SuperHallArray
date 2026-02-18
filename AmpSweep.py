#!/usr/bin/env python3
"""
Batch amplitude extraction from stepped-amplitude recordings at fixed frequency (e.g., 1 Hz).

Assumptions:
- Files are named like "0.1V.txt", "1V.txt", "2.5V.txt", "20V.txt" (case-insensitive).
- Each file has at least 4 signal columns (we use the first 4).
- If a timestamp column exists, it's the LAST column.
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
  python amp_sweep.py --folder . --freq 1 --pattern "*.txt" --fs 60   # only if no timestamp column
"""

from __future__ import annotations
import argparse
import glob
import os
import re
from typing import Optional, Tuple, List, Dict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


V_RE = re.compile(r"(?P<v>\d+(?:\.\d+)?)\s*v", re.IGNORECASE)


def parse_volts_from_filename(path: str) -> Optional[float]:
    base = os.path.basename(path)
    m = V_RE.search(base)
    if not m:
        return None
    return float(m.group("v"))


def load_table(path: str, expected_cols: int = 5, max_rel_jump: float = 0.5) -> Tuple[np.ndarray, Dict[str, int]]:
    rows: List[List[float]] = []
    prev_row: Optional[List[float]] = None

    n_short = 0
    n_long = 0
    n_non_numeric = 0
    n_jump = 0

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    # Recordings often have short metadata/footer blocks.
    if len(lines) > 8:
        lines = lines[4:-4]

    for line in lines:
        s = line.strip()
        if not s:
            continue

        fields = [v.strip() for v in s.split(",")] if "," in s else s.split()
        fields = [v for v in fields if v]
        if len(fields) < expected_cols:
            n_short += 1
            continue
        if len(fields) > expected_cols:
            n_long += 1
            continue

        try:
            row = [float(fields[i]) for i in range(expected_cols)]
        except ValueError:
            n_non_numeric += 1
            continue

        if prev_row is not None:
            bad_jump = False
            for i in range(expected_cols):
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--folder", required=True, help="Folder containing recordings like 0.1V.txt ... 20V.txt")
    ap.add_argument("--freq", type=float, required=True, help="Injected sine frequency in Hz (e.g., 1)")
    ap.add_argument("--pattern", default="*.txt", help="Glob pattern inside folder (default: *.txt)")
    ap.add_argument("--fs", type=float, default=None, help="Fallback sample rate (Hz) if no timestamp column.")
    ap.add_argument("--expected_cols", type=int, default=5,
                    help="Expected column count. Rows not matching are dropped (default: 5).")
    ap.add_argument("--max_rel_jump", type=float, default=0.5,
                    help="Row is dropped if any value changes by > this fraction vs previous accepted row.")
    ap.add_argument("--discard_frac", type=float, default=0.25,
                    help="Discard initial fraction for settling (default 0.25). Set 0 to disable.")
    ap.add_argument("--out_csv", default="amplitude_summary_vs_inputV.csv", help="Output CSV filename")
    ap.add_argument("--out_qc_csv", default="row_qc_summary_vs_inputV.csv", help="Output QC CSV filename")
    ap.add_argument("--out_png", default="amplitude_vs_inputV.png", help="Output plot filename (PNG)")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.folder, args.pattern)))
    if not files:
        raise SystemExit(f"No files matched: {os.path.join(args.folder, args.pattern)}")

    rows = []
    skipped = []
    qc_by_file = {}

    for path in files:
        vin = parse_volts_from_filename(path)
        if vin is None:
            skipped.append(os.path.basename(path))
            continue

        arr, qc = load_table(path, expected_cols=args.expected_cols, max_rel_jump=args.max_rel_jump)
        qc_by_file[os.path.basename(path)] = qc
        t, sig = infer_time_seconds(arr, args.fs)

        # discard settling
        if args.discard_frac > 0:
            n0 = int(len(t) * args.discard_frac)
            if n0 >= len(t) - 20:
                continue
            t = t[n0:] - t[n0]
            sig = sig[n0:, :]

        if len(t) < 20:
            continue

        amps = [coherent_amp_peak(t, sig[:, ch], args.freq) for ch in range(4)]
        rows.append([vin, *amps])

    if not rows:
        raise SystemExit("No usable files found (did filenames include 'V'?)")

    rows.sort(key=lambda r: r[0])
    out = pd.DataFrame(rows, columns=["input_v", "amp1_peak", "amp2_peak", "amp3_peak", "amp4_peak"])

    out_csv_path = os.path.join(args.folder, args.out_csv)
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
    out_qc_csv_path = os.path.join(args.folder, args.out_qc_csv)
    qc_df.to_csv(out_qc_csv_path, index=False)

    # Plot: amplitude vs injected voltage (log x)
    plt.figure(figsize=(8, 6), dpi=300)
    plt.plot(out["input_v"], out["amp1_peak"], marker="o", linewidth=1, label="Response 1")
    plt.plot(out["input_v"], out["amp2_peak"], marker="o", linewidth=1, label="Response 2")
    plt.plot(out["input_v"], out["amp3_peak"], marker="o", linewidth=1, label="Response 3")
    plt.plot(out["input_v"], out["amp4_peak"], marker="o", linewidth=1, label="Response 4")

    plt.xscale("log")
    plt.xlabel("Injected Amplitude (V)")
    plt.yscale("log")
    plt.ylabel("Measured Peak Amplitude (ADC counts)")
    plt.title(f"Measured Amplitude vs Injected Amplitude @ {args.freq:g} Hz (Coherent Fit)")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()

    out_png_path = os.path.join(args.folder, args.out_png)
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
