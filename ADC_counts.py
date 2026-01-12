import numpy as np
import pandas as pd
import argparse


Q20 = 2**20

def rms_noise_counts(x_counts: np.array, detrend: bool = False) -> float:
    x = np.asarray(x_counts, dtype = float)
    if x.size < 2:
        return float("nan")
    
    if detrend:
        t = np.arange(x.size, dtype=float)
        # least squares
        A = np.vstack([t, np.ones_like(t)]).T
        m, b = np.linalg.lstsq(A, x, rcond=None)[0]
        r = x - (m * t + b)
    else:
        r = x - np.mean(x)

    return float(np.sqrt(np.mean(r * r)))   # counts RMS

def main():
    ap = argparse.ArgumentParser(description="Compute RMS noise (counts) for Hall array streams.")
    ap.add_argument("file", help="CSV log file, 4 col: raw, spatial, temporal, combined")
    ap.add_argument("--detrend", action="store_true", help="Remove Best fit before RMS")
    args = ap.parse_args()

    data_q20 = pd.read_csv(args.file, header=None)
    if data_q20.shape[1] < 4:
        raise ValueError("Expected >= 4 cols")
    
    # convert Q20 -> ADC counts
    data_counts = data_q20.iloc[:, :4].to_numpy(dtype=float) / Q20

    # Columns: raw, spatial, temporal, combined
    labels = ["raw", "spatial", "temporal", "combined"]
    sigmas = {}
    for i, name in enumerate(labels):
        sigmas[name] = rms_noise_counts(data_counts[:,i], detrend=args.detrend)

    base = sigmas["raw"]
    print(f"File: {args.file}")
    print(f"Mode: {'detrend+RMS' if args.detrend else 'mean-remove+RMS'}")
    print("")
    for name in labels:
        s = sigmas[name]
        ratio = base / s if (s > 0 and np.isfinite(s)) else float("nan")
        gain_db = 20 * np.log10(ratio) if (ratio > 0 and np.isfinite(ratio)) else float("nan")
        print(f"{name:8s}: std_RMS = {s:10.6f} counts    improvement vs raw {ratio:9.3f}x  ({gain_db:7.2f} dB)")


if __name__ == "__main__":
    main()
