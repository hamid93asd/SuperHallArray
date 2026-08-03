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

# --- SPEED ESTIMATION SETTINGS ---
DISTANCE_CM = 10.0
TRIGGER_THRESHOLD = 500  # Counts above baseline to register the magnet
PAD_A = 0  # Array index for Pad 1
PAD_B = 5  # Array index for Pad 6

# --- STATE MACHINE ---
state = 'IDLE'
time_A = 0.0
time_B = 0.0
cooldown_timer = 0.0

# --- CALIBRATION ---
CALIBRATION_FRAMES = 50
baseline = np.zeros(NUM_CHANNELS)
calibration_buffer = []
is_calibrated = False

# --- GUI SETUP ---
app = QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="SuperHallArray Speed Estimator")
win.resize(1200, 700)
plot = win.addPlot(title="Waiting for swipe...")
plot.showGrid(x=True, y=True)
plot.setLabel('left', 'Magnetic Flux', units='Counts')
plot.setLabel('bottom', 'Time', units='Frames')

curves = []
for i in range(NUM_CHANNELS):
    color = pg.intColor(i, hues=NUM_CHANNELS)
    curves.append(plot.plot(pen=color))

data_history = np.zeros((NUM_CHANNELS, 400))

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0)
except serial.SerialException as e:
    print(f"Error opening port {PORT}: {e}")
    sys.exit(1)

def update():
    global is_calibrated, baseline, data_history, state, time_A, time_B, cooldown_timer
    
    while ser.in_waiting >= (4 + PAYLOAD_SIZE):
        sync_buffer = ser.read(4)
        if sync_buffer != SYNC_BYTES:
            ser.read(1)
            continue
            
        payload = ser.read(PAYLOAD_SIZE)
        if len(payload) == PAYLOAD_SIZE:
            raw_data = np.array(struct.unpack('<36H', payload))
            
            # --- CALIBRATION ---
            if not is_calibrated:
                calibration_buffer.append(raw_data)
                if len(calibration_buffer) >= CALIBRATION_FRAMES:
                    baseline = np.mean(calibration_buffer, axis=0)
                    is_calibrated = True
                    print("\n[READY] Baseline zeroed. Swipe the magnet from Pad 1 to Pad 6!")
                return
            
            zeroed_data = raw_data - baseline
            current_time = time.time()
            
            # --- SPEED ESTIMATION LOGIC ---
            val_A = zeroed_data[PAD_A]
            val_B = zeroed_data[PAD_B]
            
            if state == 'IDLE':
                if val_A > TRIGGER_THRESHOLD:
                    time_A = current_time
                    state = 'WAIT_B'
                    plot.setTitle("Pad 1 Triggered! Waiting for Pad 6...", color="y")
                elif val_B > TRIGGER_THRESHOLD:
                    time_B = current_time
                    state = 'WAIT_A'
                    plot.setTitle("Pad 6 Triggered! Waiting for Pad 1...", color="y")
                    
            elif state == 'WAIT_B':
                if val_B > TRIGGER_THRESHOLD:
                    time_B = current_time
                    dt = time_B - time_A
                    speed = DISTANCE_CM / dt
                    msg = f"Swipe Detected (1->6)! Speed: {speed:.2f} cm/s"
                    print(f"[SUCCESS] {msg}")
                    plot.setTitle(msg, color="g", size="16pt")
                    state = 'COOLDOWN'
                    cooldown_timer = current_time
                    
            elif state == 'WAIT_A':
                if val_A > TRIGGER_THRESHOLD:
                    time_A = current_time
                    dt = time_A - time_B
                    speed = DISTANCE_CM / dt
                    msg = f"Swipe Detected (6->1)! Speed: {speed:.2f} cm/s"
                    print(f"[SUCCESS] {msg}")
                    plot.setTitle(msg, color="g", size="16pt")
                    state = 'COOLDOWN'
                    cooldown_timer = current_time
                    
            elif state == 'COOLDOWN':
                if current_time - cooldown_timer > 1.5:  # 1.5 second reset
                    state = 'IDLE'
                    plot.setTitle("Ready for next swipe...", color="w")

            # --- UPDATE PLOT ---
            data_history = np.roll(data_history, -1, axis=1)
            data_history[:, -1] = zeroed_data
            
            for i in range(NUM_CHANNELS):
                curves[i].setData(data_history[i])

timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

if __name__ == '__main__':
    print("Gathering baseline... Keep magnet away.")
    sys.exit(app.exec())