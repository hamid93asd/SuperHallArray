import sys
import serial
import signal
import struct
import time as time_module
import numpy as np
import pyqtgraph as pg
from time import time
from scipy.ndimage import gaussian_filter

PORT = '/dev/tty.usbmodemPICO1'  # Serial port for Pico
BAUD = 921600                   # Baud rate
ROWS = 6
COLS = 6
FRAME_WORDS = ROWS * COLS
SYNC = b"\xAA\x55\xAA\x55"
SYNC_LEN = len(SYNC)
FRAME_BYTES = FRAME_WORDS * 2 + SYNC_LEN
PAYLOAD_BYTES = FRAME_WORDS * 2
TIME_CONSTANT = .5  # seconds
DEVIATION_RANGE = 200
UI_FPS = 90
ALPHA = 1 - (1 / (TIME_CONSTANT * UI_FPS))
SIGMA = 0.0
_UI_DT = 1.0 / UI_FPS


rx = bytearray()
_running = True
baseline = np.full((ROWS, COLS), 32768.0)

_last_ui = 0.0
_last_ev_idle = 0.0
_n_ui = 0
t_last_report = time()
_n_frames = 0
_acc_read = 0.0
_acc_math = 0.0
_acc_filt = 0.0
_acc_img = 0.0
_acc_ev = 0.0
_reads = 0

def handle_exit(signum, frame):
    global _running
    _running = False

signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)

def open_serial():
    while _running:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.0001)
            ser.reset_input_buffer()
            print("Serial connected.")
            return ser
        except Exception as e:
            print("Waiting for serial connection...")
            time_module.sleep(1)
        
def read_frame(ser, max_scan_frames=2, max_buffer=2048):
    """Read one frame with validation"""
    global rx, _reads

    chunk = ser.read(max(1, ser.in_waiting))

    if not chunk:
        return None
    rx.extend(chunk)

    if len(rx) > max_buffer:
        rx[:] = rx[-max_buffer:]

    attempts = 0
    while attempts < max_scan_frames:
        i = rx.find(SYNC)

        if i < 0:           # No sync found, discard all but last byte
            keep = len(SYNC) - 1
            if len(rx) > keep:
                rx[:] = rx[-keep:]
            return None
    
        if i > 0:           # Discard data before sync
            del rx[:i]

        if rx[:SYNC_LEN] != SYNC:
            del rx[:1]
            return None


        if len(rx) < FRAME_BYTES:       # Not enough data yet
            return None

        
        payload = bytes(rx[SYNC_LEN:SYNC_LEN + PAYLOAD_BYTES])
        _reads += 1
        del rx[:FRAME_BYTES]
        vals = struct.unpack(f'<{FRAME_WORDS}H', payload)

        if any(v > 65535 for v in vals):
            attempts += 1
            print("Passing rejected frame with invalid data.")
            # continue

        return np.array(vals, dtype=float).reshape((ROWS, COLS))
    return None


def main():
    global _running, baseline, _acc_math, _acc_read, _acc_filt, _acc_img, _acc_ev, _n_frames, t_last_report, _last_ui, _n_ui, _last_ev_idle, _reads
    t_i0 = t_i1 = t_ev0 = t_ev1 = 0.0

    ser = open_serial()
    if ser is None:
        return

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

    while _running & win.isVisible():
        try:
            t0 = time()
            arr = read_frame(ser)
            t1 = time()
            if arr is None:
                now = time()
                if now - _last_ev_idle >= 3/UI_FPS:
                    t_ev0 = time()
                    app.processEvents()
                    t_ev1 = time()
                    _acc_ev += (t_ev1 - t_ev0)
                    _last_ev_idle = now
                continue

            # Update baseline and compute deviation
            t_m0 = time()
            deviation = arr - baseline
            baseline = ALPHA * baseline + (1 - ALPHA) * arr
            t_m1 = time()

            # Gaussian smoothing
            t_f0 = time()
            # deviation = gaussian_filter(deviation, sigma=SIGMA)
            t_f1 = time()

            now = time()

            if now - _last_ui >= _UI_DT:
                t_i0 = time()
                img.setImage(deviation.T, autoLevels=False)
                t_i1 = time()

                t_ev0 = time()
                app.processEvents()
                t_ev1 = time()
                _last_ui = now
                _n_ui += 1

            # Timing stats
            _n_frames += 1
            _acc_read += (t1 - t0)
            _acc_math += (t_m1 - t_m0)
            _acc_filt += (t_f1 - t_f0)
            _acc_img += (t_i1 - t_i0)
            _acc_ev += (t_ev1 - t_ev0)

            now = time()
            if now - t_last_report >= 1.0:
                fps = _n_frames / (now - t_last_report)
                print(
                    f"FPS={fps:6.1f} "
                    f"read={(_acc_read/_n_frames)*1000:5.2f}ms "
                    f"reads/sec={(_reads/_n_frames)*1000:5.2f} "
                    f"math={(_acc_math/_n_frames)*1000:5.2f}ms "
                    f"gauss={(_acc_filt/_n_frames)*1000:5.2f}ms "
                    f"img={(_acc_img/_n_frames)*1000:5.2f}ms "
                    f"events={(_acc_ev/max(1,_n_ui))*1000:5.2f}ms/ui"
                )
                t_last_report = now
                _n_frames = _n_ui = _reads = 0
                _acc_read = _acc_math = _acc_filt = _acc_img = _acc_ev = 0.0

        except serial.SerialException as e:
            print("ERROR: Serial port disconnected: ", e)
            try:
                ser.close()
            except:
                pass
            ser = open_serial()
            if ser is None:
                break

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