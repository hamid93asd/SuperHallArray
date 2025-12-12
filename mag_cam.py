import sys
import serial
import numpy as np
import matplotlib.pyplot as plt

PORT = '/dev/tty.usbmodem14101'  # Serial port for Pico
BAUD = 115200                   # Baud rate

ROWS = 6
COLS = 6
NCH = ROWS * COLS

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)

    data = np.zeros((ROWS, COLS))

    plt.ion()
    fig, ax = plt.subplots()
    im = ax.imshow(data, vmin=0, vmax=4095, origin="lower", interpolation="nearest")
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("Sensor value")

    ax.set_title("magnetic Camera (6x6 Hall Array)")
    plt.show()

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

            im.set_data(arr)

            plt.pause(0.001)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()