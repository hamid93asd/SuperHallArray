#!/usr/bin/env python3
"""
Batch amplitude extraction from stepped-sine recordings.

Assumptions:
- Files are named like "0.1Hz.txt", "1.25Hz.txt", etc. (case-insensitive).
- Each file has at least 4 signal columns (we use the first 4).
- If a timestamp column exists, it's the LAST column.
  - If timestamps look like microseconds, they are converted to seconds automatically.
- Amplitude is extracted via coherent least-squares fit:
    x(t) ≈ a*sin(2π f t) + b*cos(2π f t)
    A_peak = sqrt(a^2 + b^2)

Outputs:
- A CSV summary (amplitude_peak per channel vs frequency)
- A 300 DPI plot PNG

Usage:
  python sweep_amp.py --folder /path/to/folder
  python sweep_amp.py --folder . --pattern "*.txt"
  python sweep_amp.py --folder . --fs 60     # only needed if files have NO timestamp column
"""

from __future__ import annotations
import argparse
import glob
import os
import re
from typing import Optional, Tuple, List

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


FREQ_RE = re.compile(r"(?P<f>\d+(?:\.\d+)?)\s*hz", re.IGNORECASE)


def parse_freq_hz_from_filename(path: str) -> Optional[float]:
    base = os.path.basename(path)
    m = FREQ_RE.search(base)
    if not m:
        return None
    return float(m.group("f"))


def load_table(path: str) -> np.ndarray:
    # Try comma first, then whitespace
    try:
        arr = pd.read_csv(path, header=None).values
        if arr.shape[1] == 1:
            raise ValueError("single-col parse")
        return arr
    except Exception:
        arr = pd.read_csv(path, header=None, delim_whitespace=True).values
        return arr


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

        # If it's monotonic-ish, treat as timebase. Otherwise fall back.
        d = np.diff(ts)
        if np.nanmedian(d) <= 0:
            if fs_fallback is None:
                raise ValueError("Timestamp column not monotonic; provide --fs to use sample index time.")
            t = np.arange(len(sig), dtype=np.float64) / fs_fallback
            return t, sig

        dt_med = float(np.nanmedian(d))

        # Heuristic for units:
        # - If dt is huge (e.g., 16667), likely microseconds.
        # - If dt is moderately large (e.g., 16.7), could be milliseconds.
        # - Else assume seconds.
        if dt_med > 1_000.0:
            scale = 1e-6  # us -> s
        elif dt_med > 1.0:
            scale = 1e-3  # ms -> s (best guess)
        else:
            scale = 1.0

        t = (ts - ts[0]) * scale
        return t, sig

    # No timestamp column
    if fs_fallback is None:
        raise ValueError(f"{os.path.basename(path)} has no timestamp col; provide --fs.")
    t = np.arange(len(sig), dtype=np.float64) / fs_fallback
    return t, sig


def coherent_amp_peak(t: np.ndarray, x: np.ndarray, f_hz: float) -> float:
    """
    Coherent least-squares amplitude at known frequency.
    Removes DC before fitting.
    Returns peak amplitude (same units as x).
    """
    x = x.astype(np.float64)
    x = x - np.nanmean(x)

    w = 2.0 * np.pi * f_hz
    s = np.sin(w * t)
    c = np.cos(w * t)
    X = np.column_stack([s, c])

    # Solve min ||X*[a,b]-x||
    coeffs, _, _, _ = np.linalg.lstsq(X, x, rcond=None)
    a, b = coeffs
    return float(np.sqrt(a * a + b * b))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--folder", required=True, help="Folder containing recordings like 0.1Hz.txt")
    ap.add_argument("--pattern", default="*.txt", help="Glob pattern inside folder (default: *.txt)")
    ap.add_argument("--fs", type=float, default=None, help="Fallback sample rate (Hz) if no timestamp column.")
    ap.add_argument("--discard_frac", type=float, default=0.0,
                    help="Optionally discard first fraction of each file (e.g., 0.25 to drop first 25%%).")
    ap.add_argument("--out_csv", default="amplitude_summary.csv", help="Output CSV filename")
    ap.add_argument("--out_png", default="amplitude_vs_frequency.png", help="Output plot filename (PNG)")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(args.folder, args.pattern)))
    if not files:
        raise SystemExit(f"No files matched: {os.path.join(args.folder, args.pattern)}")

    rows = []
    skipped = []

    for path in files:
        f_hz = parse_freq_hz_from_filename(path)
        if f_hz is None:
            skipped.append(os.path.basename(path))
            continue

        arr = load_table(path)
        t, sig = infer_time_seconds(arr, args.fs)

        # Optional discard of early portion (settling)
        if args.discard_frac > 0:
            n0 = int(len(t) * args.discard_frac)
            t = t[n0:] - t[n0]
            sig = sig[n0:, :]

        # Guard against too-short traces
        if len(t) < 20:
            continue

        amps = [coherent_amp_peak(t, sig[:, ch], f_hz) for ch in range(4)]
        rows.append([f_hz, *amps])

    if not rows:
        raise SystemExit("No usable files found (did filenames include 'Hz'?)")

    rows.sort(key=lambda r: r[0])
    out = pd.DataFrame(rows, columns=["freq_hz", "amp1_peak", "amp2_peak", "amp3_peak", "amp4_peak"])

    out_csv_path = os.path.join(args.folder, args.out_csv)
    out.to_csv(out_csv_path, index=False)

    # Plot
    plt.figure(figsize=(8, 6), dpi=300)
    plt.plot(out["freq_hz"], out["amp1_peak"], marker="o", linewidth=1, label="Response 1")
    plt.plot(out["freq_hz"], out["amp2_peak"], marker="o", linewidth=1, label="Response 2")
    plt.plot(out["freq_hz"], out["amp3_peak"], marker="o", linewidth=1, label="Response 3")
    plt.plot(out["freq_hz"], out["amp4_peak"], marker="o", linewidth=1, label="Response 4")

    plt.xscale("log")
    plt.xlabel("Injected Frequency (Hz)")
    plt.ylabel("Peak Amplitude (ADC counts)")
    plt.title("Extracted Amplitude vs Frequency (Coherent Fit)")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()

    out_png_path = os.path.join(args.folder, args.out_png)
    plt.savefig(out_png_path, dpi=300)
    plt.close()

    if skipped:
        print("Skipped (couldn't parse frequency from filename):")
        for s in skipped:
            print("  ", s)

    print("Wrote:", out_csv_path)
    print("Wrote:", out_png_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
