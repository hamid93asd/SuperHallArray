import os
import pandas as pd
import matplotlib.pyplot as plt

FOLDER = "Scripts/Plot Data/Velocity Sweep"
SIMPLE_FILE = "Simp_test.csv"
NCC_FILE = "NCC_test.csv"
OUT_CSV = "velocity_vy_means_comparison.csv"
OUT_PNG = "velocity_vy_means_comparison.png"


def grouped_vy_stats(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    vy = df.iloc[:, 4].astype(float)
    tmp = pd.DataFrame(
        {
            "target_mm_s": df.iloc[:, 0].astype(float),
            "vy_mm_s": vy,
        }
    )
    out = (
        tmp.groupby("target_mm_s", as_index=False)["vy_mm_s"]
        .agg(vy_sum="sum", vy_count="count", vy_mean="mean")
        .sort_values("target_mm_s")
        .reset_index(drop=True)
    )
    return out


def main() -> int:
    simple_path = os.path.join(FOLDER, SIMPLE_FILE)
    ncc_path = os.path.join(FOLDER, NCC_FILE)

    simple = grouped_vy_stats(simple_path).rename(
        columns={
            "vy_sum": "simple_vy_sum",
            "vy_count": "simple_vy_count",
            "vy_mean": "simple_vy_mean",
        }
    )
    ncc = grouped_vy_stats(ncc_path).rename(
        columns={
            "vy_sum": "ncc_vy_sum",
            "vy_count": "ncc_vy_count",
            "vy_mean": "ncc_vy_mean",
        }
    )

    merged = simple.merge(ncc, on="target_mm_s", how="inner")

    out_csv_path = os.path.join(FOLDER, OUT_CSV)
    merged.to_csv(out_csv_path, index=False)

    plt.figure(figsize=(8, 5), dpi=200)
    plt.plot(merged["target_mm_s"], merged["simple_vy_mean"], marker="o", label="Simple vy mean")
    plt.plot(merged["target_mm_s"], -merged["ncc_vy_mean"], marker="o", label="NCC vy mean")
    plt.xlabel("Target Velocity (mm/s)")
    plt.ylabel("Average vy (mm/s)")
    plt.title("Velocity Sweep Comparison (Averaged vy per Target)")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    out_png_path = os.path.join(FOLDER, OUT_PNG)
    plt.savefig(out_png_path, dpi=300)
    plt.close()

    print("Wrote:", out_csv_path)
    print("Wrote:", out_png_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
