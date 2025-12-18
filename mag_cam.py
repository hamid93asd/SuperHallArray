import sys
import serial
import numpy as np
import matplotlib.pyplot as plt
import signal
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

PORT = '/dev/tty.usbmodem14101'  # Serial port for Pico
BAUD = 115200                   # Baud rate

ROWS = 6
COLS = 6
NCH = ROWS * COLS

LOG_EPS = 1.0

_running = True
def handle_exit(signum, frame):
    global _running
    _running = False
signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

def main():
    global _running

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print("ERROR: Could not open serial port:", e)
        sys.exit(1)

    data = np.zeros((ROWS, COLS))

    app = pg.mkQApp("Mag Camera")

    win = pg.GraphicsLayoutWidget(show=True, title="Magnetic Camera (6x6 Hall Array)")
    win.resize(600, 600)

    view = win.addViewBox(lockAspect=True)
    img = pg.ImageItem()
    view.addItem(img)
    cmap = pg.colormap.get('CET-L4')
    img.setLookupTable(cmap.getLookupTable())

    # img.setLevels([np.log10(LOG_EPS), np.log10(4095 + LOG_EPS)])
    img.setLevels([1024, 3072])
    img.setAutoDownsample(True)

    print("Listening for frames.... Press Ctrl+C to exit.")

    while _running:
        try:
            while True:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                parts = line.split(",")
                if parts[0] != 'F':
                    continue
                if len(parts) != NCH + 1:
                    continue

                try:
                    vals = [int(p) for p in parts[1:]]
                except ValueError:
                    continue

                arr = np.array(vals, dtype=float).reshape((ROWS, COLS))

                log_arr = np.log10(arr + LOG_EPS)
                # img.setImage(log_arr.T, autoLevels=False)
                img.setImage(arr.T, autoLevels=False)
                app.processEvents()

        except serial.SerialException:
            print("ERROR: Serial port disconnected:")

        except Exception as e:
            print("Unexpected error:", e)

    print("Exiting...")
    try:
        ser.close()
    except:
        pass

    plt.close('all')
    print("Done.")

if __name__ == "__main__":
    main()