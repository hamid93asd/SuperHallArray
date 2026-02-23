#!/usr/bin/env python3
"""
Batch amplitude extraction from stepped-sine recordings.

Assumptions:
- Input files can be classic names like "1.25Hz.txt" or split names like "..._1.25_Hz_20_Vpp.csv".
- Data uses columns (1-based): raw=1, temporal=5, spatial=9, combined=10, timestamp=11.
- Amplitude is extracted via coherent least-squares fit:
    x(t) ≈ a*sin(2π f t) + b*cos(2π f t)
    A_peak = sqrt(a^2 + b^2)
"""

from __future__ import annotations
import glob
import os
import re
from typing import Optional, Tuple, List, Dict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

try:
    from scipy.signal import welch as _scipy_welch
except ImportError:  # scipy is optional
    _scipy_welch = None


FREQ_SPLIT_RE = re.compile(r"_(?P<f>\d+(?:\.\d+)?)_hz(?:_|\.|$)", re.IGNORECASE)
FREQ_RE = re.compile(r"(?P<f>\d+(?:\.\d+)?)(?:[\s_]*hz)(?=$|[^a-z0-9])", re.IGNORECASE)

# Config
FOLDER = "Scripts/Plot Data/Frequency Sweep/Split"
PATTERN = "*.csv"
SIGNAL_COLS_1B = [1, 5, 9, 10]
TIME_COL_1B = 11
FS_FALLBACK: Optional[float] = None
EXPECTED_COLS: Optional[int] = None
MAX_REL_JUMP = 0.5
NOISE_BW_HZ = 1.0
WELCH_NPERSEG = 1024
DISCARD_FRAC = 0.0
OUT_CSV = "amplitude_summary_vs_frequency.csv"
OUT_QC_CSV = "row_qc_summary_vs_frequency.csv"
OUT_SNR_CSV = "snr_summary_vs_frequency.csv"
OUT_SNR_PNG = "snr_vs_frequency.png"
OUT_PNG = "amplitude_vs_frequency.png"


def parse_freq_hz_from_filename(path: str) -> Optional[float]:
    base = os.path.basename(path)
    m = FREQ_SPLIT_RE.search(base)
    if not m:
        m = FREQ_RE.search(base)
    if not m:
        return None
    return float(m.group("f"))


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
        raise ValueError("Data has no timestamp col; provide --fs.")
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


def coherent_amp_peak_and_residual(t: np.ndarray, x: np.ndarray, f_hz: float) -> Tuple[float, np.ndarray]:
    """
    Coherent least-squares amplitude at known frequency.
    Returns:
      amp_peak, residual (mean-removed, tone-subtracted)
    """
    x = x.astype(np.float64)
    x = x - np.nanmean(x)

    w = 2.0 * np.pi * f_hz
    s = np.sin(w * t)
    c = np.cos(w * t)
    X = np.column_stack([s, c])

    coeffs, _, _, _ = np.linalg.lstsq(X, x, rcond=None)
    yhat = X @ coeffs
    resid = x - yhat

    a, b = coeffs
    amp_peak = float(np.sqrt(a * a + b * b))
    return amp_peak, resid


def estimate_fs_hz(t: np.ndarray) -> float:
    if t.size < 2:
        return float("nan")
    dt = float(np.nanmedian(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return float("nan")
    return float(1.0 / dt)


def welch_psd(x: np.ndarray, fs: float, nperseg: int, noverlap: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    Welch PSD estimate (density, units x^2/Hz).
    Uses SciPy if available, otherwise a minimal NumPy implementation.
    """
    x = np.asarray(x, dtype=np.float64)
    if x.size < 16 or not np.isfinite(fs) or fs <= 0:
        raise ValueError("Not enough samples or invalid fs for PSD.")

    nperseg_eff = int(min(max(16, nperseg), x.size))
    noverlap_eff = int(min(max(0, noverlap), nperseg_eff - 1))

    if _scipy_welch is not None:
        f, pxx = _scipy_welch(
            x,
            fs=fs,
            nperseg=nperseg_eff,
            noverlap=noverlap_eff,
            detrend="constant",
            scaling="density",
            return_onesided=True,
        )
        return f.astype(np.float64), pxx.astype(np.float64)

    # Minimal Welch (Hann window, 50% overlap by default in caller).
    step = nperseg_eff - noverlap_eff
    if step <= 0:
        raise ValueError("Invalid noverlap (must be < nperseg).")

    w = np.hanning(nperseg_eff)
    w_norm = float(np.sum(w * w))
    if w_norm <= 0:
        raise ValueError("Bad window normalization.")

    freqs = np.fft.rfftfreq(nperseg_eff, d=1.0 / fs)
    psd_accum = np.zeros(freqs.shape, dtype=np.float64)
    k = 0
    for start in range(0, x.size - nperseg_eff + 1, step):
        seg = x[start:start + nperseg_eff]
        seg = seg - float(np.mean(seg))
        seg = seg * w
        X = np.fft.rfft(seg)
        P = (np.abs(X) ** 2) / (fs * w_norm)
        # one-sided correction
        if nperseg_eff % 2 == 0:
            P[1:-1] *= 2.0
        else:
            P[1:] *= 2.0
        psd_accum += P
        k += 1

    psd = psd_accum / max(k, 1)
    return freqs.astype(np.float64), psd.astype(np.float64)


def band_rms_from_psd(freqs: np.ndarray, psd: np.ndarray, f_lo: float, f_hi: float) -> float:
    mask = (freqs >= f_lo) & (freqs <= f_hi)
    if int(np.count_nonzero(mask)) < 2:
        return float("nan")
    p = float(np.trapz(psd[mask], freqs[mask]))
    if not np.isfinite(p) or p < 0:
        return float("nan")
    return float(np.sqrt(p))


def main() -> int:
    if TIME_COL_1B < 1:
        raise SystemExit("TIME_COL_1B must be >= 1")
    if any(c < 1 for c in SIGNAL_COLS_1B):
        raise SystemExit("SIGNAL_COLS_1B values must be >= 1")
    signal_cols_0b = [c - 1 for c in SIGNAL_COLS_1B]
    time_col_0b = TIME_COL_1B - 1
    selected_cols = [*signal_cols_0b, time_col_0b]

    files = sorted(glob.glob(os.path.join(FOLDER, PATTERN)))
    if not files:
        raise SystemExit(f"No files matched: {os.path.join(FOLDER, PATTERN)}")

    rows = []
    skipped = []
    qc_by_file = {}
    snr_rows = []

    for path in files:
        f_hz = parse_freq_hz_from_filename(path)
        if f_hz is None:
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

        # Optional discard of early portion (settling)
        if DISCARD_FRAC > 0:
            n0 = int(len(t) * DISCARD_FRAC)
            t = t[n0:] - t[n0]
            sig = sig[n0:, :]

        # Guard against too-short traces
        if len(t) < 20:
            continue

        amps = [coherent_amp_peak(t, sig[:, ch], f_hz) for ch in range(4)]
        rows.append([f_hz, *amps])

        fs_est = estimate_fs_hz(t)
        if np.isfinite(fs_est) and fs_est > 0 and np.isfinite(NOISE_BW_HZ) and NOISE_BW_HZ > 0:
            nyq = 0.5 * fs_est
            bw = float(NOISE_BW_HZ)
            f_lo = max(0.0, f_hz - 0.5 * bw)
            f_hi = min(nyq, f_hz + 0.5 * bw)
        else:
            fs_est = float("nan")
            f_lo = float("nan")
            f_hi = float("nan")

        snr_row = {"freq_hz": f_hz, "fs_est_hz": fs_est, "noise_band_lo_hz": f_lo, "noise_band_hi_hz": f_hi}
        for ch in range(4):
            try:
                amp_peak, resid = coherent_amp_peak_and_residual(t, sig[:, ch], f_hz)
                if np.isfinite(f_lo) and np.isfinite(f_hi) and f_hi > f_lo and np.isfinite(fs_est) and fs_est > 0:
                    nperseg = int(min(max(16, WELCH_NPERSEG), resid.size))
                    noverlap = int(nperseg // 2)
                    freqs, psd = welch_psd(resid, fs=fs_est, nperseg=nperseg, noverlap=noverlap)
                    noise_rms = band_rms_from_psd(freqs, psd, f_lo, f_hi)
                else:
                    noise_rms = float("nan")

                tone_rms = amp_peak / float(np.sqrt(2.0))
                snr_db = 20.0 * float(np.log10(tone_rms / noise_rms)) if (noise_rms > 0 and tone_rms > 0) else float("nan")
            except Exception:
                amp_peak = float("nan")
                noise_rms = float("nan")
                snr_db = float("nan")

            snr_row[f"amp{ch + 1}_peak"] = amp_peak
            snr_row[f"noise{ch + 1}_rms_band"] = noise_rms
            snr_row[f"snr{ch + 1}_db"] = snr_db

        snr_rows.append(snr_row)

    if not rows:
        raise SystemExit("No usable files found (did filenames include 'Hz'?)")

    rows.sort(key=lambda r: r[0])
    out = pd.DataFrame(rows, columns=["freq_hz", "amp1_peak", "amp2_peak", "amp3_peak", "amp4_peak"])

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

    snr_out = pd.DataFrame(snr_rows).sort_values("freq_hz")
    out_snr_csv_path = os.path.join(FOLDER, OUT_SNR_CSV)
    snr_out.to_csv(out_snr_csv_path, index=False)

    # Plot
    plt.figure(figsize=(8, 6), dpi=300)
    plt.plot(out["freq_hz"], out["amp1_peak"], marker="o", linewidth=1, label="Raw Output (Single Sensor)")
    plt.plot(out["freq_hz"], out["amp2_peak"], marker="o", linewidth=1, label="Temporal Average (Single Sensor)")
    plt.plot(out["freq_hz"], out["amp3_peak"], marker="o", linewidth=1, label="Spatial Average (Four Sensors)")
    plt.plot(out["freq_hz"], out["amp4_peak"], marker="o", linewidth=1, label="Combined Average (Four Sensors)")


    plt.xscale("log")
    plt.xlabel("Signal Frequency (Hz)")
    plt.ylabel("Peak Amplitude (ADC counts)")
    plt.title("Extracted Amplitude vs Frequency (Coherent Fit)")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()

    out_png_path = os.path.join(FOLDER, OUT_PNG)
    plt.savefig(out_png_path, dpi=300)
    plt.close()

    # Plot SNR (tone RMS vs in-band noise RMS)
    plt.figure(figsize=(8, 6), dpi=300)
    plt.plot(snr_out["freq_hz"], snr_out["snr1_db"], marker="o", linewidth=1, label="Raw Output (Single Sensor)")
    plt.plot(snr_out["freq_hz"], snr_out["snr2_db"], marker="o", linewidth=1, label="Temporal Average (Single Sensor)")
    plt.plot(snr_out["freq_hz"], snr_out["snr3_db"], marker="o", linewidth=1, label="Spatial Average (Four Sensors)")
    plt.plot(snr_out["freq_hz"], snr_out["snr4_db"], marker="o", linewidth=1, label="Combined Average (Four Sensors)")
    plt.xscale("log")
    plt.xlabel("Signal Frequency (Hz)")
    plt.ylabel("SNR (dB) in Noise Band")
    plt.title(f"SNR vs Frequency (Noise BW={NOISE_BW_HZ:g} Hz)")
    plt.grid(True, which="both")
    plt.legend()
    plt.tight_layout()

    out_snr_png_path = os.path.join(FOLDER, OUT_SNR_PNG)
    plt.savefig(out_snr_png_path, dpi=300)
    plt.close()

    if skipped:
        print("Skipped (couldn't parse frequency from filename):")
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
    print("Wrote:", out_snr_csv_path)
    print("Wrote:", out_png_path)
    print("Wrote:", out_snr_png_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
