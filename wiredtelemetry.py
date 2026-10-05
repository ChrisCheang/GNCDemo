import sys
import csv
import serial
import numpy as np
import pandas as pd
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# CSV Column Header Definitions (17 variables in exact stream order)
COLUMN_NAMES = [
    "qw", "qx", "qy", "qz",
    "pitch_deg", "yaw_deg", "roll_deg",
    "pitch_rate", "yaw_rate", "roll_rate",
    "altitude_z",
    "esc_top_us", "esc_bot_us",
    "servo_pitch_us", "servo_yaw_us",
    "pos_x", "pos_y"
]

class FlightTelemetryGUI(QtWidgets.QMainWindow):
    def __init__(self, serial_port='COM7', baud_rate=115200):
        super().__init__()
        self.setWindowTitle("Coaxial TVC Flight Telemetry & Position Monitor")
        self.resize(1400, 900)

        # Serial & Data Storage Initializations
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        
        # Buffer for live or loaded CSV data
        self.data_df = pd.DataFrame(columns=COLUMN_NAMES)

        # Build UI Layout
        self._init_ui()

        # Serial thread / timer setup for live streaming
        if self.serial_port:
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.05)
                self.timer = QtCore.QTimer()
                self.timer.timeout.connect(self._read_serial_data)
                self.timer.start(20)  # 50 Hz UI update rate
            except Exception as e:
                print(f"[ERROR] Failed to open serial port {self.serial_port}: {e}")

    def _init_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)

        # Config PyQTGraph settings
        pg.setConfigOptions(antialias=True)

        # Left Side Layout: Orientation, Actuators, Altitude
        left_layout = QtWidgets.QVBoxLayout()
        
        # Plot 1: Attitudinal Euler Angles
        self.plot_euler = pg.PlotWidget(title="Orientation Angles (NWU)")
        self.plot_euler.addLegend()
        self.plot_euler.setLabel('left', 'Angle', units='deg')
        self.curve_pitch = self.plot_euler.plot(pen=pg.mkPen('r', width=1.5), name="Pitch (Y)")
        self.curve_yaw   = self.plot_euler.plot(pen=pg.mkPen('g', width=1.5), name="Yaw (X)")
        self.curve_roll  = self.plot_euler.plot(pen=pg.mkPen('b', width=1.5), name="Roll (Z)")
        left_layout.addWidget(self.plot_euler)

        # Plot 2: Actuators (ESCs & Servos)
        self.plot_actuators = pg.PlotWidget(title="Motor & TVC Servo Microseconds")
        self.plot_actuators.addLegend()
        self.plot_actuators.setLabel('left', 'Pulse Width', units='us')
        self.curve_esc_top    = self.plot_actuators.plot(pen=pg.mkPen('c', width=1.5), name="ESC Top")
        self.curve_esc_bot    = self.plot_actuators.plot(pen=pg.mkPen('m', width=1.5), name="ESC Bot")
        self.curve_servo_p    = self.plot_actuators.plot(pen=pg.mkPen('y', width=1.5), name="Servo Pitch")
        self.curve_servo_y    = self.plot_actuators.plot(pen=pg.mkPen('w', width=1.5), name="Servo Yaw")
        left_layout.addWidget(self.plot_actuators)

        # Plot 3: 1D Altitude Trajectory
        self.plot_alt = pg.PlotWidget(title="Kalman Filtered Altitude (Z)")
        self.plot_alt.setLabel('left', 'Altitude', units='m')
        self.plot_alt.setLabel('bottom', 'Sample Index')
        self.curve_alt = self.plot_alt.plot(pen=pg.mkPen('g', width=2), name="Alt Z")
        left_layout.addWidget(self.plot_alt)

        # Right Side Layout: Top-Down 2D Position Plot (X Vertical Up, Y Horizontal Left)
        right_layout = QtWidgets.QVBoxLayout()

        self.plot_pos_2d = pg.PlotWidget(title="2D Earth-Fixed Position (Top-Down View from Z)")
        self.plot_pos_2d.setLabel('left', 'X Position / North', units='m')      # Vertical axis
        self.plot_pos_2d.setLabel('bottom', 'Y Position / West', units='m')    # Horizontal axis
        self.plot_pos_2d.showGrid(x=True, y=True, alpha=0.3)
        self.plot_pos_2d.setAspectLocked(True, ratio=1.0)                       # Maintain 1:1 spatial aspect ratio
        
        # Invert X-axis so positive Y points Left according to NWU standard top-down view
        self.plot_pos_2d.getPlotItem().invertX(True)

        # Trajectory curve line and live position marker
        self.curve_pos_2d = self.plot_pos_2d.plot(pen=pg.mkPen('y', width=2), name="Path")
        self.point_current_pos = self.plot_pos_2d.plot(
            pen=None, symbol='o', symbolSize=10, symbolBrush='r', name="Current Pos"
        )
        right_layout.addWidget(self.plot_pos_2d)

        # Control panel buttons (e.g., CSV Load)
        btn_layout = QtWidgets.QHBoxLayout()
        self.btn_load_csv = QtWidgets.QPushButton("Load CSV Log")
        self.btn_load_csv.clicked.connect(self.load_csv_dialog)
        btn_layout.addWidget(self.btn_load_csv)
        right_layout.addLayout(btn_layout)

        # Combine main layouts
        main_layout.addLayout(left_layout, stretch=1)
        main_layout.addLayout(right_layout, stretch=1)

    def load_csv_file(self, filepath):
        """Loads and parses static extended 17-variable CSV log files."""
        try:
            df = pd.read_csv(filepath, names=COLUMN_NAMES, header=None)
            self.data_df = df.apply(pd.to_numeric, errors='coerce').dropna()
            self.update_plots()
            print(f"[INFO] Successfully loaded {len(self.data_df)} telemetry samples from {filepath}")
        except Exception as e:
            print(f"[ERROR] Failed to load CSV file: {e}")

    def load_csv_dialog(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open CSV Telemetry Log", "", "CSV Files (*.csv);;Text Files (*.txt)")
        if filename:
            self.load_csv_file(filename)

    def _read_serial_data(self):
        """Parses real-time serial streams containing 17 comma-separated floats."""
        if not self.ser or not self.ser.in_waiting:
            return

        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                parts = line.split(',')
                if len(parts) == 17:
                    values = [float(p) for p in parts]
                    new_row = pd.DataFrame([values], columns=COLUMN_NAMES)
                    self.data_df = pd.concat([self.data_df, new_row], ignore_index=True)
            except ValueError:
                continue

        # Keep buffer bounded for performance during long live runs
        if len(self.data_df) > 500:
            self.data_df = self.data_df.iloc[-5000:].reset_index(drop=True)

        self.update_plots()

    def update_plots(self):
        if self.data_df.empty:
            return

        idx = np.arange(len(self.data_df))

        # 1. Update Orientation Angles
        self.curve_pitch.setData(idx, self.data_df["pitch_deg"].values)
        self.curve_yaw.setData(idx, self.data_df["yaw_deg"].values)
        self.curve_roll.setData(idx, self.data_df["roll_deg"].values)

        # 2. Update Actuator Microseconds
        self.curve_esc_top.setData(idx, self.data_df["esc_top_us"].values)
        self.curve_esc_bot.setData(idx, self.data_df["esc_bot_us"].values)
        self.curve_servo_p.setData(idx, self.data_df["servo_pitch_us"].values)
        self.curve_servo_y.setData(idx, self.data_df["servo_yaw_us"].values)

        # 3. Update Altitude (Z)
        self.curve_alt.setData(idx, self.data_df["altitude_z"].values)

        # 4. Update 2D NWU Top-Down Position Plot
        pos_x = self.data_df["pos_x"].values  # Vertical axis on graph
        pos_y = self.data_df["pos_y"].values  # Horizontal axis on graph
        
        # Plot (Y, X) so Y position maps to horizontal axis and X position maps to vertical axis
        self.curve_pos_2d.setData(pos_y, pos_x)
        if len(pos_x) > 0:
            self.point_current_pos.setData([pos_y[-1]], [pos_x[-1]])

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    # Defaults to 'COM7' if no port is passed via command-line arguments
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    gui = FlightTelemetryGUI(serial_port=port)
    gui.show()

    sys.exit(app.exec_())
