import sys
import serial
import struct
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

# ==========================================
# CONFIGURATION
# ==========================================
PORT = '/dev/ttyACM0' 
BAUD = 921600 
ROWS = 6
COLS = 6
FRAME_WORDS = ROWS * COLS  # 36 sensors
SYNC = b"\xAA\x55\xAA\x55"
SYNC_LEN = len(SYNC)
PAYLOAD_BYTES = FRAME_WORDS * 2
FRAME_BYTES = PAYLOAD_BYTES + SYNC_LEN

WINDOW_SIZE = 300  # Number of frames to show on screen

# Buffer for raw serial bytes
rx = bytearray()

# ==========================================
# PARSING LOGIC (From mag_cam.py)
# ==========================================
def read_frame(ser, max_scan_frames=2, max_buffer=2048):
    global rx
    chunk = ser.read(max(1, ser.in_waiting))
    if not chunk:
        return None
    rx.extend(chunk)

    if len(rx) > max_buffer:
        rx[:] = rx[-max_buffer:]

    attempts = 0
    while attempts < max_scan_frames:
        i = rx.find(SYNC)

        if i < 0:  # No sync found, discard all but last byte
            keep = len(SYNC) - 1
            if len(rx) > keep:
                rx[:] = rx[-keep:]
            return None
        
        if i > 0:  # Discard data before sync
            del rx[:i]

        if rx[:SYNC_LEN] != SYNC:
            del rx[:1]
            return None

        if len(rx) < FRAME_BYTES:  # Not enough data yet
            return None
        
        payload = bytes(rx[SYNC_LEN:SYNC_LEN + PAYLOAD_BYTES])
        del rx[:FRAME_BYTES]
        vals = struct.unpack(f'<{FRAME_WORDS}H', payload)

        if any(v > 65535 for v in vals):
            attempts += 1
            continue

        return np.array(vals, dtype=float)
    return None

# ==========================================
# PYQTGRAPH SETUP
# ==========================================
app = QtWidgets.QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="SuperHallArray Time-Series")
win.resize(1000, 600)
win.setBackground('k')

plot = win.addPlot(title="36-Channel Magnetic Flux Density")
plot.setLabel('left', 'ADC Counts (Scaled)')
plot.setLabel('bottom', 'Time (Frames)')
plot.addLegend(offset=(10, 10))

# Automatically scale the Y-axis based on the data
plot.enableAutoRange(axis=pg.ViewBox.YAxis)

curves = []
for i in range(FRAME_WORDS):
    color = pg.intColor(i, hues=FRAME_WORDS)
    curve = plot.plot(pen=pg.mkPen(color, width=1.5), name=f"Pad {i+1}")
    curves.append(curve)

# Initialize buffer with the known firmware baseline (32768)
data_buffer = np.full((FRAME_WORDS, WINDOW_SIZE), 32768.0)

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.0001)
    ser.reset_input_buffer()
    print(f"Connected to {PORT} at {BAUD} baud. Plotting data...")
except serial.SerialException as e:
    print(f"Failed to open port {PORT}: {e}")
    sys.exit(1)

# ==========================================
# UPDATE LOOP
# ==========================================
def update_plot():
    global data_buffer
    
    updated = False
    # Pull ALL available frames from the serial buffer to prevent lag
    while True:
        frame = read_frame(ser)
        if frame is None:
            break
            
        data_buffer = np.roll(data_buffer, -1, axis=1)
        data_buffer[:, -1] = frame
        updated = True

    if updated:
        for i in range(FRAME_WORDS):
            curves[i].setData(data_buffer[i])

# Update UI at ~33 FPS
timer = QtCore.QTimer()
timer.timeout.connect(update_plot)
timer.start(30)

if __name__ == '__main__':
    sys.exit(app.exec())