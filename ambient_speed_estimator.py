import sys
import time
import serial
import struct
import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QApplication

# --- CONFIGURATION ---
PORT = '/dev/ttyACM0'  
BAUDRATE = 921600
NUM_CHANNELS = 36
SYNC_BYTES = b'\xAA\x55\xAA\x55'
PAYLOAD_SIZE = NUM_CHANNELS * 2

# --- ADVISOR CONSTRAINTS & SETTINGS ---
DISTANCE_CM = 10.0
TRIGGER_THRESHOLD = 3  # Starts recording when signal crosses this
COL_A_INDICES = [0, 6, 12, 18, 24, 30]  # Column 1
COL_B_INDICES = [5, 11, 17, 23, 29, 35] # Column 6

# --- FILTERING ---
CALIBRATION_FRAMES = 100
SMOOTHING_FACTOR = 0.3

# --- STATE TRACKING ---
tracking_active = False
swipe_buffer = []

# --- GUI SETUP ---
app = QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="SuperHallArray Peak-Detection Tracking")
win.resize(1200, 700)
plot = win.addPlot(title="Waiting for magnet swipe...")
plot.showGrid(x=True, y=True)
plot.setLabel('left', 'Filtered Magnetic Flux', units='Counts')
plot.setLabel('bottom', 'Time', units='Frames')

curves = []
for i in range(NUM_CHANNELS):
    color = pg.intColor(i, hues=NUM_CHANNELS)
    curves.append(plot.plot(pen=color))

data_history = np.zeros((NUM_CHANNELS, 400))
baseline = np.zeros(NUM_CHANNELS)
current_filtered = np.zeros(NUM_CHANNELS)
calibration_buffer = []
is_calibrated = False

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
except serial.SerialException as e:
    print(f"Error opening port {PORT}: {e}")
    sys.exit(1)

def update():
    global is_calibrated, baseline, current_filtered, data_history, tracking_active, swipe_buffer
    
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
                    is_calibrated = True
                    print("\n[READY] Baseline zeroed. Ready for magnet tracking!")
                return
            
            # --- PHASE 2: COMMON-MODE REJECTION FILTER (All 36 Pads) ---
            zeroed_data = raw_data - baseline
            ensemble_average = np.mean(zeroed_data)
            cmr_data = zeroed_data - ensemble_average
            current_filtered = (SMOOTHING_FACTOR * cmr_data) + ((1 - SMOOTHING_FACTOR) * current_filtered)
            
            # --- PHASE 3: PEAK-DETECTION SPEED ESTIMATION (Columns 1 and 6) ---
            val_A = np.mean(current_filtered[COL_A_INDICES])
            val_B = np.mean(current_filtered[COL_B_INDICES])
            current_time = time.time()
            
            if not tracking_active:
                if val_A > TRIGGER_THRESHOLD or val_B > TRIGGER_THRESHOLD:
                    tracking_active = True
                    swipe_buffer = [(current_time, val_A, val_B)]
                    plot.setTitle("Recording swipe... Keep moving!", color="y")
            else:
                # Append live data to buffer while tracking
                swipe_buffer.append((current_time, val_A, val_B))
                
                # Check if the magnet has completely left the board
                if val_A < (TRIGGER_THRESHOLD / 2) and val_B < (TRIGGER_THRESHOLD / 2):
                    tracking_active = False
                    
                    if len(swipe_buffer) > 5:
                        # Extract data arrays from buffer
                        times = [row[0] for row in swipe_buffer]
                        col_A_vals = [row[1] for row in swipe_buffer]
                        col_B_vals = [row[2] for row in swipe_buffer]
                        
                        # Find the exact timestamp where each column hit its maximum peak
                        max_A_idx = np.argmax(col_A_vals)
                        max_B_idx = np.argmax(col_B_vals)
                        
                        time_A_peak = times[max_A_idx]
                        time_B_peak = times[max_B_idx]
                        
                        dt = time_B_peak - time_A_peak
                        
                        # Prevent division by zero and filter out instant noise glitches
                        if abs(dt) > 0.05:
                            speed = DISTANCE_CM / abs(dt)
                            direction = "1->6" if dt > 0 else "6->1"
                            msg = f"Magnet Swipe ({direction})! Speed: {speed:.2f} cm/s"
                            print(f"\n[SUCCESS] {msg}")
                            plot.setTitle(msg, color="g", size="16pt")
                        else:
                            plot.setTitle("Swipe too messy or parallel to calculate.", color="r")
                    else:
                        plot.setTitle("Ready for next swipe...", color="w")

            # --- PHASE 4: UPDATE PLOT ---
            data_history = np.roll(data_history, -1, axis=1)
            data_history[:, -1] = current_filtered
            
            for i in range(NUM_CHANNELS):
                curves[i].setData(data_history[i])

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

if __name__ == '__main__':

    print("Gathering static baseline... Keep array still and magnet away.")
    sys.exit(app.exec())