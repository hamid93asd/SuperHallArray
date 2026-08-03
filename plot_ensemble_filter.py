import sys
import serial
import struct
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QApplication

# --- CONFIGURATION ---
PORT = '/dev/ttyACM0'  # Update if necessary
BAUDRATE = 921600
NUM_CHANNELS = 36
SYNC_BYTES = b'\xAA\x55\xAA\x55'
PAYLOAD_SIZE = NUM_CHANNELS * 2

# --- CALIBRATION & FILTERING ---
CALIBRATION_FRAMES = 50
SMOOTHING_FACTOR = 0.15  
HISTORY_LEN = 400        

# --- GUI SETUP ---
app = QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="SuperHallArray Ensemble Filter Test")
win.resize(1200, 700)
plot = win.addPlot(title="Common-Mode Rejected Output (Advisors Method)")
plot.showGrid(x=True, y=True)
plot.setLabel('left', 'Relative Magnetic Flux (Filtered)', units='Counts')
plot.setLabel('bottom', 'Time', units='Frames')
plot.addLegend()

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
    
    while ser.in_waiting >= (4 + PAYLOAD_SIZE):
        sync_buffer = ser.read(4)
        if sync_buffer != SYNC_BYTES:
            ser.read(1)
            continue
            
        payload = ser.read(PAYLOAD_SIZE)
        if len(payload) == PAYLOAD_SIZE:
            raw_data = np.array(struct.unpack('<36H', payload))
            
            # --- PHASE 1: STATIC CALIBRATION ---
            if not is_calibrated:
                calibration_buffer.append(raw_data)
                if len(calibration_buffer) >= CALIBRATION_FRAMES:
                    baseline = np.mean(calibration_buffer, axis=0)
                    current_filtered = np.zeros(NUM_CHANNELS)
                    is_calibrated = True
                    print("\n[SUCCESS] Baseline zeroed! Ready for testing.")
                return 
            
            # --- PHASE 2: ADVISOR'S ENSEMBLE FILTER ---
            # 1. Subtract the static resting baseline
            zeroed_data = raw_data - baseline
            
            # 2. Calculate the live ensemble average (the common global drift)
            ensemble_average = np.mean(zeroed_data)
            
            # 3. Subtract the common drift from the individual sensors
            cmr_data = zeroed_data - ensemble_average
            
            # 4. Apply exponential smoothing to clean up the electrical fuzz
            current_filtered = (SMOOTHING_FACTOR * cmr_data) + ((1 - SMOOTHING_FACTOR) * current_filtered)
            
            # --- PHASE 3: UPDATE PLOT ---
            data_history = np.roll(data_history, -1, axis=1)
            data_history[:, -1] = current_filtered
            
            for i in range(NUM_CHANNELS):
                curves[i].setData(data_history[i])

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

if __name__ == '__main__':
    print("Gathering static baseline... Keep array still.")
    sys.exit(app.exec())