import sys
import serial
import struct
import numpy as np

# --- CONFIGURATION ---
PORT = '/dev/ttyACM0'  
BAUDRATE = 921600
NUM_CHANNELS = 36
SYNC_BYTES = b'\xAA\x55\xAA\x55'
PAYLOAD_SIZE = NUM_CHANNELS * 2

# --- TEST SETTINGS ---
FRAMES_TO_RECORD = 1000
QUANTIZATION_UT = 10.25  # microtesla per count

# --- ADC VOLTAGE SETTINGS ---
V_REF = 5  # Change to 5.0 if your microcontroller uses 5V logic
BIT_RESOLUTION = 16
TOTAL_LEVELS = 2 ** BIT_RESOLUTION

def run_noise_test():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening port {PORT}: {e}")
        sys.exit(1)

    print("Gathering 1,000 frames of static data...")
    print("DO NOT MOVE. Keep all metal and electronics away from the array.")
    
    raw_frames = []
    
    # Flush buffer
    ser.reset_input_buffer()
    
    while len(raw_frames) < FRAMES_TO_RECORD:
        if ser.in_waiting >= (4 + PAYLOAD_SIZE):
            sync_buffer = ser.read(4)
            if sync_buffer != SYNC_BYTES:
                ser.read(1)
                continue
                
            payload = ser.read(PAYLOAD_SIZE)
            if len(payload) == PAYLOAD_SIZE:
                raw_data = np.array(struct.unpack('<36H', payload))
                raw_frames.append(raw_data)
                
                if len(raw_frames) % 200 == 0:
                    print(f"Recorded {len(raw_frames)}/1000 frames...")

    print("\nProcessing data with Common-Mode Rejection (Ensemble Average)...")
    raw_frames = np.array(raw_frames)
    
    # 1. Calculate static baseline (mean of each pad over time)
    baseline = np.mean(raw_frames, axis=0)
    
    # 2. Subtract baseline
    zeroed_frames = raw_frames - baseline
    
    # 3. Apply CMR (subtract the ensemble average of the board from each frame)
    cmr_frames = []
    for frame in zeroed_frames:
        ensemble_avg = np.mean(frame)
        cmr_frames.append(frame - ensemble_avg)
        
    cmr_frames = np.array(cmr_frames)
    
    # 4. Calculate the RMS noise (Standard Deviation) of the fully filtered system
    system_noise_sigma = np.std(cmr_frames)
    
    # 5. Calculate Minimum Detectable Field (3-Sigma Threshold)
    min_detectable_counts = 3 * system_noise_sigma
    min_detectable_ut = min_detectable_counts * QUANTIZATION_UT
    
    # 6. Voltage Conversions
    noise_voltage = system_noise_sigma * (V_REF / TOTAL_LEVELS)
    threshold_voltage = min_detectable_counts * (V_REF / TOTAL_LEVELS)
    
    print("\n" + "="*65)
    print("      BARE BOARD SENSITIVITY REPORT (VOLTAGE ENABLED)")
    print("="*65)
    print(f"System RMS Noise (1-Sigma):    {system_noise_sigma:.3f} counts ({noise_voltage:.6f} V)")
    print(f"Detection Threshold (3-Sigma): {min_detectable_counts:.3f} counts ({threshold_voltage:.6f} V)")
    print("-" * 65)
    print(f"Minimum Detectable Field:      {min_detectable_ut:.2f} µT")
    print("="*65)

if __name__ == '__main__':
    run_noise_test()