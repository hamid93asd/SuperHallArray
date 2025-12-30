import sys
import serial
import signal
import struct
import numpy as np
import pyqtgraph as pg
from scipy.ndimage import gaussian_filter

PORT = '/dev/tty.usbmodem14201'  # Serial port for Pico
BAUD = 115200                   # Baud rate
ROWS = 6
COLS = 6
NCH = ROWS * COLS
ALPHA = 0.98
DEVIATION_RANGE = 20

_running = True
baseline = np.full((ROWS, COLS), 2048.0)

def handle_exit(signum, frame):
    global _running
    _running = False

signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

def main():
    global _running, baseline

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print("ERROR: Could not open serial port: ", e)
        sys.exit(1)

    app = pg.mkQApp("Mag Camera")
    win = pg.GraphicsLayoutWidget(show=True, title="Magnetic Camera (6x6 Hall Array)")
    win.resize(600, 600)

    view = win.addViewBox(lockAspect=True)
    img = pg.ImageItem()
    view.addItem(img)

    cmap = pg.colormap.get('CET-D1A')
    img.setLookupTable(cmap.getLookupTable())
    img.setLevels([-DEVIATION_RANGE, DEVIATION_RANGE])
    img.setAutoDownsample(True)

    print("Listening for frames.... Press Ctrl+C to exit.")

    while _running and win.isVisible():
        try:
            # Look for start word
            if ser.read(1) != b'\xFF':
                continue

            # Read frame
            data = ser.read(72)  # 36 channels * 2 bytes each
            if len(data) != 72:
                continue

            vals = struct.unpack('<36H', data)
            arr = np.array(vals, dtype=float).reshape((ROWS, COLS))

            # Update baseline and compute deviation
            baseline = ALPHA * baseline + (1 - ALPHA) * arr
            deviation = arr - baseline

            # Gaussian smoothing
            deviation = gaussian_filter(deviation, sigma=0.8)

            img.setImage(deviation.T, autoLevels=False)
            app.processEvents()

        except serial.SerialException:
            print("ERROR: Serial port disconnected: ", e)

        except Exception as e:
            print("Unexpected error: ", e)

    print("Exiting...")
    try:
        ser.close()
    except:
        pass
    app.quit()
    print("Done.")

if __name__ == "__main__":
    main()