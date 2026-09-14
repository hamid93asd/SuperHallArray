#!/usr/bin/env python3
"""
SuperHallArray live viewer
==========================
Streams the 36-channel Hall array, removes the DC baseline, and plots the
result in real physical volts with a scientific-notation Y axis.

Changes relative to the previous version:
  * ADC scaling fixed to the actual hardware (12-bit ADC1283, AVCC = 5 V).
  * Scientific-notation axis is now correct in log mode as well.
  * Log/linear toggle plots |signal| with a defined noise floor instead of
    a 1e-9 epsilon.
  * Serial sync recovery is byte-accurate, so a dropped byte no longer
    desynchronises the stream permanently.
  * Rolling mean uses an incremental sum (O(1) per frame) with a window
    short enough to preserve real field transients.
  * Optional microtesla display, toggleable legend, and live recalibration.
"""

import sys
import numpy as np
import serial
import pyqtgraph as pg
from collections import deque
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel,
)

# --- LINK CONFIGURATION ---
PORT = '/dev/ttyACM0'
BAUDRATE = 921600
NUM_CHANNELS = 36
SYNC_BYTES = b'\xAA\x55\xAA\x55'
PAYLOAD_SIZE = NUM_CHANNELS * 2          # 36 little-endian uint16

# --- ADC / SENSOR SCALING ---
# The array uses the ADC1283 (12-bit SAR) running at AVCC = 5 V, and the
# DRV5055A1 responds at 100 mV/mT. One LSB is therefore 5/4096 = 1.221 mV,
# which is the 12.2 uT/LSB figure quoted in the paper. Using 3.3 V and 2**16
# understated every reading by a factor of 24.
V_REF = 5.0
ADC_BITS = 12
ADC_FULL_SCALE = 1 << ADC_BITS
RAW_SHIFT = 4                            # set to 4 if the firmware left-aligns
VOLTS_PER_COUNT = V_REF / ADC_FULL_SCALE
SENSOR_V_PER_TESLA = 100.0               # 100 mV/mT
MICROTESLA_PER_VOLT = 1e6 / SENSOR_V_PER_TESLA

# --- FILTERING / DISPLAY ---
CALIBRATION_FRAMES = 500
WINDOW_SIZE = 64                         # frames in the moving average
HISTORY_LEN = 800                        # frames shown on screen
LOG_FLOOR_V = 1e-7                       # sub-LSB floor for the log view
UPDATE_MS = 10


class SciNotationAxis(pg.AxisItem):
    """Forces ticks such as 1.2e-03, and un-logs the value in log mode."""

    def tickStrings(self, values, scale, spacing):
        if self.logMode:
            return [f"{10.0 ** v:.1e}" for v in values]
        return [f"{v:.1e}" for v in values]


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SuperHallArray Ambient Magnetic Tracking")
        self.resize(1200, 700)

        self.is_log = False
        self.in_tesla = False
        self.legend_visible = True

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        controls = QHBoxLayout()
        self.btn_log = QPushButton("Switch to log-linear (Y axis)")
        self.btn_log.clicked.connect(self.toggle_log)
        controls.addWidget(self.btn_log)

        self.btn_units = QPushButton("Show microtesla")
        self.btn_units.clicked.connect(self.toggle_units)
        controls.addWidget(self.btn_units)

        self.btn_legend = QPushButton("Hide legend")
        self.btn_legend.clicked.connect(self.toggle_legend)
        controls.addWidget(self.btn_legend)

        self.btn_recal = QPushButton("Recalibrate baseline")
        controls.addWidget(self.btn_recal)
        layout.addLayout(controls)

        self.status = QLabel("Gathering baseline, keep the board stationary.")
        layout.addWidget(self.status)

        self.axis = SciNotationAxis(orientation='left')
        self.plot_widget = pg.PlotWidget(axisItems={'left': self.axis})
        layout.addWidget(self.plot_widget)

        self.plot_widget.setTitle("Zero-baseline filtered output")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setLabel('bottom', 'Time', units='Frames')
        # Auto SI prefixing would relabel the axis "uV" while the custom ticks
        # still printed raw volts, which is what produced the mismatched
        # screenshot. Disable it and state the unit in the label instead.
        self.axis.enableAutoSIPrefix(False)
        self.apply_axis_label()

        # Single column, anchored inside the top-left corner, as before.
        self.legend = self.plot_widget.addLegend(offset=(10, 10))
        self.curves = []
        for i in range(NUM_CHANNELS):
            color = pg.intColor(i, hues=NUM_CHANNELS)
            self.curves.append(
                self.plot_widget.plot(pen=color, name=f'Pad {i + 1}')
            )

    def apply_axis_label(self):
        unit = 'uT' if self.in_tesla else 'V'
        self.plot_widget.setLabel(
            'left', f"Baseline-referenced signal [{unit}]"
        )

    def toggle_log(self):
        self.is_log = not self.is_log
        self.plot_widget.setLogMode(x=False, y=self.is_log)
        self.btn_log.setText(
            "Switch to linear-linear (Y axis)" if self.is_log
            else "Switch to log-linear (Y axis)"
        )

    def toggle_units(self):
        self.in_tesla = not self.in_tesla
        self.btn_units.setText(
            "Show volts" if self.in_tesla else "Show microtesla"
        )
        self.apply_axis_label()

    def toggle_legend(self):
        self.legend_visible = not self.legend_visible
        self.legend.setVisible(self.legend_visible)
        self.btn_legend.setText(
            "Hide legend" if self.legend_visible else "Show legend"
        )


class HallArrayReader:
    """Byte-accurate framing over the USB CDC link."""

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0)
        self.buf = bytearray()

    def read_frames(self):
        pending = self.ser.in_waiting
        if pending:
            self.buf.extend(self.ser.read(pending))

        frames = []
        while True:
            idx = self.buf.find(SYNC_BYTES)
            if idx < 0:
                # Keep only a possible partial sync word.
                del self.buf[:max(0, len(self.buf) - (len(SYNC_BYTES) - 1))]
                break
            if len(self.buf) - idx < len(SYNC_BYTES) + PAYLOAD_SIZE:
                del self.buf[:idx]          # wait for the rest of the frame
                break
            start = idx + len(SYNC_BYTES)
            payload = bytes(self.buf[start:start + PAYLOAD_SIZE])
            del self.buf[:start + PAYLOAD_SIZE]
            counts = np.frombuffer(payload, dtype='<u2').astype(np.float64)
            frames.append(counts / (1 << RAW_SHIFT))
        return frames

    def close(self):
        self.ser.close()


class Acquisition:
    def __init__(self, window, reader):
        self.window = window
        self.reader = reader
        self.history = np.zeros((NUM_CHANNELS, HISTORY_LEN))
        self.reset_calibration()
        window.btn_recal.clicked.connect(self.reset_calibration)

    def reset_calibration(self):
        self.is_calibrated = False
        self.baseline = np.zeros(NUM_CHANNELS)
        self.cal_buffer = []
        self.rolling = deque(maxlen=WINDOW_SIZE)
        self.rolling_sum = np.zeros(NUM_CHANNELS)
        self.window.status.setText(
            "Gathering baseline, keep the board stationary."
        )

    def calibrate(self, raw):
        self.cal_buffer.append(raw)
        if len(self.cal_buffer) < CALIBRATION_FRAMES:
            return
        stack = np.array(self.cal_buffer)
        self.baseline = stack.mean(axis=0)
        self.is_calibrated = True
        self.cal_buffer = []

        peak = stack.max()
        note = ""
        if peak >= ADC_FULL_SCALE:
            note = (f" Warning: peak raw count {peak:.0f} exceeds 12-bit "
                    f"full scale, set RAW_SHIFT (try 4) or check ADC_BITS.")
        self.window.status.setText(
            f"Calibrated. 1 LSB = {VOLTS_PER_COUNT * 1e3:.3f} mV "
            f"({VOLTS_PER_COUNT * MICROTESLA_PER_VOLT:.1f} uT). "
            f"Moving average window = {WINDOW_SIZE} frames.{note}"
        )

    def filtered_volts(self, raw):
        zeroed = raw - self.baseline
        if len(self.rolling) == WINDOW_SIZE:
            self.rolling_sum -= self.rolling[0]
        self.rolling.append(zeroed)
        self.rolling_sum += zeroed
        mean_counts = self.rolling_sum / len(self.rolling)
        return mean_counts * VOLTS_PER_COUNT

    def update(self):
        new_columns = []
        for raw in self.reader.read_frames():
            if not self.is_calibrated:
                self.calibrate(raw)
                continue
            new_columns.append(self.filtered_volts(raw))

        if not new_columns:
            return

        block = np.array(new_columns[-HISTORY_LEN:]).T
        n = block.shape[1]
        self.history = np.roll(self.history, -n, axis=1)
        self.history[:, -n:] = block
        self.redraw()

    def redraw(self):
        scale = MICROTESLA_PER_VOLT if self.window.in_tesla else 1.0
        floor = LOG_FLOOR_V * scale
        data = self.history * scale
        if self.window.is_log:
            # A log axis cannot show sign. Plot magnitude, clamped at a floor
            # below the averaged noise level so quiet channels stay on screen.
            data = np.maximum(np.abs(data), floor)
        for i in range(NUM_CHANNELS):
            self.window.curves[i].setData(data[i])


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    try:
        reader = HallArrayReader(PORT, BAUDRATE)
    except serial.SerialException as exc:
        print(f"Error opening port {PORT}: {exc}")
        return 1

    acq = Acquisition(window, reader)
    timer = QTimer()
    timer.timeout.connect(acq.update)
    timer.start(UPDATE_MS)

    try:
        return app.exec()
    finally:
        reader.close()


if __name__ == '__main__':
    print("Gathering baseline. Please keep the board stationary.")
    sys.exit(main())