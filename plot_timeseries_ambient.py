import sys
import serial
import struct
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QApplication

# --- CONFIGURATION ---
PORT = '/dev/ttyACM0'  # Update if your Ubuntu port changed
BAUDRATE = 921600
NUM_CHANNELS = 36
SYNC_BYTES = b'\xAA\x55\xAA\x55'
PAYLOAD_SIZE = NUM_CHANNELS * 2  # 36 channels * 2 bytes (uint16)

# --- CALIBRATION & FILTERING ---
CALIBRATION_FRAMES = 50
SMOOTHING_FACTOR = 0.15  # Lower = smoother but slower response (0.0 to 1.0)
HISTORY_LEN = 400        # Number of data points to show on screen

# --- GUI SETUP ---
app = QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="SuperHallArray Ambient Magnetic Tracking")
win.resize(1200, 700)
plot = win.addPlot(title="Zero-Baseline Filtered Output (Ambient Fields)")
plot.showGrid(x=True, y=True)
plot.setLabel('left', 'Relative Magnetic Flux (Filtered)', units='Counts')
plot.setLabel('bottom', 'Time', units='Frames')
plot.addLegend()

# Create 36 curves with distinct colors
curves = []
for i in range(NUM_CHANNELS):
    color = pg.intColor(i, hues=NUM_CHANNELS)
    curves.append(plot.plot(pen=color, name=f'Pad {i+1}'))

# Initialize data arrays
data_history = np.zeros((NUM_CHANNELS, HISTORY_LEN))
baseline = np.zeros(NUM_CHANNELS)
current_filtered = np.zeros(NUM_CHANNELS)
calibration_buffer = []
is_calibrated = False

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
except serial.SerialException as e:
    print(f"Error opening port {PORT}: {e}")
    sys.exit(1)

def update():
    global is_calibrated, baseline, current_filtered, data_history
    
    # Read from serial buffer
    while ser.in_waiting >= (4 + PAYLOAD_SIZE):
        # Look for the sync bytes
        sync_buffer = ser.read(4)
        if sync_buffer != SYNC_BYTES:
            # If out of sync, read 1 byte at a time until we catch the pattern
            ser.read(1)
            continue
            
        # Read the actual payload
        payload = ser.read(PAYLOAD_SIZE)
        if len(payload) == PAYLOAD_SIZE:
            # Unpack raw 16-bit unsigned integers
            raw_data = np.array(struct.unpack('<36H', payload))
            
            # --- PHASE 1: CALIBRATION ---
            if not is_calibrated:
                calibration_buffer.append(raw_data)
                if len(calibration_buffer) >= CALIBRATION_FRAMES:
                    baseline = np.mean(calibration_buffer, axis=0)
                    current_filtered = np.zeros(NUM_CHANNELS)
                    is_calibrated = True
                    print("\n[SUCCESS] Calibration complete! Baseline zeroed.")
                    print("--> Now smoothly slide the array across the desk/floor.")
                return  # Skip plotting until calibrated
            
            # --- PHASE 2: FILTERING & ZEROING ---
            # 1. Subtract the baseline to zero out the natural manufacturing tolerances
            zeroed_data = raw_data - baseline
            
            # 2. Apply Exponential Smoothing to filter out hardware noise
            current_filtered = (SMOOTHING_FACTOR * zeroed_data) + ((1 - SMOOTHING_FACTOR) * current_filtered)
            
            # --- PHASE 3: UPDATE PLOT ---
            # Shift the history matrix left and add the newest readings to the right
            data_history = np.roll(data_history, -1, axis=1)
            data_history[:, -1] = current_filtered
            
            # Update the GUI lines
            for i in range(NUM_CHANNELS):
                curves[i].setData(data_history[i])

# Set up Qt Timer to run the update loop
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)  # Check for serial data every 10ms

if __name__ == '__main__':
    print("Gathering baseline... PLEASE KEEP THE BOARD STATIONARY AND MAGNETS FAR AWAY.")
    sys.exit(app.exec())