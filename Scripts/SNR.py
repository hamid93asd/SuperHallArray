import numpy as np
import pandas as pd

FILE = '/Users/jacobdebry/Documents/HallArray/Data/North_2.txt'

data = pd.read_csv(FILE, header=None)

def calculate_snr(signal_data):
    mean_signal = np.mean(signal_data)
    std_noise = np.std(signal_data)
    snr_linear = mean_signal / std_noise
    snr_db = 20 * np.log10(snr_linear)
    return snr_linear, snr_db

snr_raw, snr_raw_db = calculate_snr(data[0])
snr_temporal, snr_temporal_db = calculate_snr(data[2])
snr_ensemble, snr_ensemble_db = calculate_snr(data[1])
snr_both, snr_both_db = calculate_snr(data[3])

print(f"Raw SNR (Sensor 16) {snr_raw_db:.1f} dB")
print(f"Temporal Average (Sensor 16) {snr_temporal_db:.1f} dB (Improvement: {snr_temporal/snr_raw:.1f}x)")
print(f"Ensemble Average (Whole Array) {snr_ensemble_db:.1f} dB (Improvement: {snr_ensemble/snr_raw:.1f}x)")
print(f"Combined Average (Whole Array) {snr_both_db:.1f} dB (Improvement: {snr_both/snr_raw:.1f}x)")