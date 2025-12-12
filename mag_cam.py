import sys
import serial
import numpy as np
import matplotlib.pyplot as plt
import signal

PORT = '/dev/tty.usbmodem14201'  # Serial port for Pico
BAUD = 115200                   # Baud rate

ROWS = 6
COLS = 6
NCH = ROWS * COLS

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

    plt.ion()
    fig, ax = plt.subplots()
    im = ax.imshow(data, vmin=0, vmax=4095, origin="lower", interpolation="nearest")
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("Sensor value")
    ax.set_title("magnetic Camera (6x6 Hall Array)")
    plt.show()

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

                im.set_data(arr)

                fig.canvas.draw_idle()
                plt.pause(0.001)

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