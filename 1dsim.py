import sys
import collections
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

class AltitudeSimGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Coaxial TVC 1D Altitude Controller Simulation (0 - 10s)")
        self.resize(1400, 850)

        # --- Vehicle & Physical Constants ---
        self.m = 1.232                    # Mass (kg)
        self.CoG_height = 0.57            # Rest height / base CoG height (m)
        self.g = 9.81                     # Gravitational acceleration (m/s^2)

        # --- ESC Dynamics (First-order model) ---
        self.esc_tau = 0.05               # ESC time constant (s)
        self.actual_throttle_fraction = 0.0 # Filtered throttle fraction

        # --- PID Gains (Default matching altkalman.ino) ---
        self.alt_kp = 820.0
        self.alt_ki = 250.0
        self.alt_kd = 100.0

        # --- PID State Variables ---
        self.alt_integral = 0.0
        self.prev_alt_error = 0.0

        # --- Simulation States [z, vz] ---
        self.state = np.array([self.CoG_height, 0.0])
        self.sim_time = 0.0
        self.running = False
        self.max_sim_time = 10.0          # Run simulation from 0s to 10s

        # --- Telemetry Buffers for Plots ---
        self.buf_t = []
        self.buf_z = []
        self.buf_target_z = []
        self.buf_esc = []

        # --- Battery & Throttle States ---
        self.battery_voltage = 12.6
        self.hover_throttle_us = 1500

        # --- UI Setup ---
        self._init_ui()

        # --- Simulation Timer (Target ~5ms average dt, jitter 0.7ms to 7ms) ---
        self.sim_timer = QtCore.QTimer()
        self.sim_timer.timeout.connect(self._step_simulation)
        self.sim_timer.start(5)  # 5ms base timer interval

    def _init_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)

        pg.setConfigOptions(antialias=True)

        # ==========================================
        # LEFT SIDE: Controls, Gain Sliders & Readouts
        # ==========================================
        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget)
        left_widget.setMaximumWidth(400)

        # Simulation Control Buttons
        btn_layout = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("Start (t=0)")
        self.btn_start.clicked.connect(self.start_simulation)
        self.btn_start.setStyleSheet("background-color: #2ecc71; color: white; font-weight: bold; font-size: 14px; padding: 8px;")
        
        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_pause.clicked.connect(self.pause_simulation)
        self.btn_pause.setStyleSheet("background-color: #e74c3c; color: white; font-weight: bold; font-size: 14px; padding: 8px;")
        
        btn_layout.addWidget(self.btn_start)
        btn_layout.addWidget(self.btn_pause)
        left_layout.addLayout(btn_layout)

        # Adjustable Altitude PID Gains Group Box
        gains_group = QtWidgets.QGroupBox("Adjustable Altitude PID Gains")
        gains_layout = QtWidgets.QFormLayout(gains_group)

        def create_gain_control(initial_val, min_val, max_val, step):
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(0, 1000)
            
            spinbox = QtWidgets.QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setSingleStep(step)
            spinbox.setValue(initial_val)

            def update_from_slider(val):
                mapped = min_val + (val / 1000.0) * (max_val - min_val)
                spinbox.blockSignals(True)
                spinbox.setValue(mapped)
                spinbox.blockSignals(False)

            def update_from_spinbox(val):
                mapped_slider = int(((val - min_val) / (max_val - min_val)) * 1000)
                slider.blockSignals(True)
                slider.setValue(max(0, min(1000, mapped_slider)))
                slider.blockSignals(False)

            slider.setValue(int(((initial_val - min_val) / (max_val - min_val)) * 1000))
            slider.valueChanged.connect(update_from_slider)
            spinbox.valueChanged.connect(update_from_spinbox)

            container = QtWidgets.QWidget()
            h_layout = QtWidgets.QHBoxLayout(container)
            h_layout.setContentsMargins(0, 0, 0, 0)
            h_layout.addWidget(slider)
            h_layout.addWidget(spinbox)
            return container, spinbox

        self.control_alt_kp, self.spin_alt_kp = create_gain_control(self.alt_kp, 0.0, 3000.0, 10.0)
        self.control_alt_ki, self.spin_alt_ki = create_gain_control(self.alt_ki, 0.0, 1000.0, 10.0)
        self.control_alt_kd, self.spin_alt_kd = create_gain_control(self.alt_kd, 0.0, 1000.0, 10.0)

        gains_layout.addRow("Altitude Kp:", self.control_alt_kp)
        gains_layout.addRow("Altitude Ki:", self.control_alt_ki)
        gains_layout.addRow("Altitude Kd:", self.control_alt_kd)
        left_layout.addWidget(gains_group)

        # Telemetry Readouts Group Box
        readout_group = QtWidgets.QGroupBox("Live Flight Data Readouts")
        readout_layout = QtWidgets.QFormLayout(readout_group)

        self.lbl_t = QtWidgets.QLabel("0.00 s")
        self.lbl_alt = QtWidgets.QLabel("0.57 m")
        self.lbl_target_alt = QtWidgets.QLabel("0.57 m")
        self.lbl_esc = QtWidgets.QLabel("1000 us")

        for lbl in (self.lbl_t, self.lbl_alt, self.lbl_target_alt, self.lbl_esc):
            lbl.setStyleSheet("font-weight: bold; font-size: 14px; color: #2980b9;")

        readout_layout.addRow("Simulation Time (t):", self.lbl_t)
        readout_layout.addRow("Altitude (Z):", self.lbl_alt)
        readout_layout.addRow("Target Altitude:", self.lbl_target_alt)
        readout_layout.addRow("ESC Pulse Width:", self.lbl_esc)
        
        left_layout.addWidget(readout_group)
        left_layout.addStretch()

        # ==========================================
        # RIGHT SIDE: Time-Series Plots
        # ==========================================
        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_widget)

        # Plot 1: Altitude Response
        self.plot_alt = pg.PlotWidget(title="1D Altitude Response (Z vs Target)")
        self.plot_alt.addLegend()
        self.plot_alt.setLabel('left', 'Altitude', units='m')
        self.plot_alt.showGrid(x=True, y=True, alpha=0.3)
        self.curve_alt = self.plot_alt.plot(pen=pg.mkPen('g', width=2), name="Actual Altitude")
        self.curve_target_alt = self.plot_alt.plot(pen=pg.mkPen('c', style=QtCore.Qt.DashLine, width=1.5), name="Target Altitude")
        right_layout.addWidget(self.plot_alt)

        # Plot 2: Control Input (ESC PWM)
        self.plot_esc = pg.PlotWidget(title="Control Input (ESC Throttle PWM)")
        self.plot_esc.addLegend()
        self.plot_esc.setLabel('left', 'PWM', units='us')
        self.plot_esc.setLabel('bottom', 'Time', units='s')
        self.plot_esc.showGrid(x=True, y=True, alpha=0.3)
        self.curve_esc = self.plot_esc.plot(pen=pg.mkPen('b', width=1.5), name="ESC PWM Command")
        right_layout.addWidget(self.plot_esc)

        self.plot_esc.setXLink(self.plot_alt)

        main_layout.addWidget(left_widget, stretch=1)
        main_layout.addWidget(right_widget, stretch=3)

    def start_simulation(self):
        self.sim_time = 0.0
        self.state = np.array([self.CoG_height, 0.0])  # [z, vz]
        self.alt_integral = 0.0
        self.prev_alt_error = 0.0
        self.actual_throttle_fraction = 0.0
        
        self.buf_t.clear()
        self.buf_z.clear()
        self.buf_target_z.clear()
        self.buf_esc.clear()
        
        self.running = True

    def pause_simulation(self):
        self.running = not self.running
        self.btn_pause.setText("Resume" if not self.running else "Pause")

    def _thrust_func(self, voltage, th_fraction):
        th = np.clip(th_fraction, 0.0, 1.0)
        return (voltage / 12.6) * (2.48 * th - 0.51)  # Thrust in kg (coaxial assembly)

    def _throttle_cmd(self, target_thrust_kg, voltage):
        if voltage < 5.0: voltage = 12.6
        th = ((target_thrust_kg * 12.6 / voltage) + 0.51) / 2.48
        return np.clip(th, 0.0, 1.0)

    def _step_simulation(self):
        if not self.running:
            return

        # Simulate dt between 0.7 ms and 7 ms with average ~5 ms (0.005 s)
        dt = np.random.uniform(0.0007, 0.007)
        self.sim_time += dt

        if self.sim_time >= self.max_sim_time:
            self.running = False
            return

        # --- Update PID Gains from UI Sliders/Spinboxes on the fly ---
        self.alt_kp = self.spin_alt_kp.value()
        self.alt_ki = self.spin_alt_ki.value()
        self.alt_kd = self.spin_alt_kd.value()

        t = self.sim_time

        # --- Target Altitude Profile (+1.0m increase trapezoidal profile) ---
        if t < 1.0:
            target_alt = self.CoG_height
        elif t < 3.0:
            target_alt = self.CoG_height + 1.0 * ((t - 1.0) / 2.0)
        elif t < 4.0:
            target_alt = self.CoG_height + 1.0
        elif t <= 6.0:
            target_alt = (self.CoG_height + 1.0) - 1.0 * ((t - 4.0) / 2.0)
        else:
            target_alt = self.CoG_height

        # --- Dynamic Hover Throttle Calculation ---
        total_weight_kg = self.m
        hover_th_frac = self._throttle_cmd(total_weight_kg, self.battery_voltage)
        self.hover_throttle_us = 1000 + int(hover_th_frac * 1000.0)

        # --- Altitude PID Controller ---
        z, vz = self.state
        measured_alt = z + np.random.normal(0, 0.002)  # Small sensor noise

        alt_error = target_alt - measured_alt
        self.alt_integral += alt_error * dt
        self.alt_integral = np.clip(self.alt_integral, -100.0, 100.0)
        alt_deriv = (alt_error - self.prev_alt_error) / dt if dt > 0 else 0.0
        self.prev_alt_error = alt_error

        alt_output = (self.alt_kp * alt_error) + (self.alt_ki * self.alt_integral) + (self.alt_kd * alt_deriv)
        target_base_throttle_us = self.hover_throttle_us + int(alt_output)
        target_base_throttle_us = np.clip(target_base_throttle_us, 1000, 1960)
        target_throttle_fraction = (target_base_throttle_us - 1000) / 1000.0

        # --- ESC First-Order Smooth Response Model ---
        self.actual_throttle_fraction += (dt / self.esc_tau) * (target_throttle_fraction - self.actual_throttle_fraction)
        base_throttle_us = 1000 + int(self.actual_throttle_fraction * 1000.0)

        # --- 1D Vertical Vehicle Physics ---
        total_thrust_kg = self._thrust_func(self.battery_voltage, self.actual_throttle_fraction)
        T_mag = total_thrust_kg * self.g  # Upward Thrust in Newtons

        # Vertical acceleration: F_z = T_mag - m*g
        az = (T_mag - (self.m * self.g)) / self.m

        # Integrate state
        vz += az * dt
        z += vz * dt

        # Ground interaction constraint (cannot fall below CoG rest height)
        if z <= self.CoG_height:
            z = self.CoG_height
            vz = 0.0
            az = 0.0

        self.state = np.array([z, vz])

        # --- Update Buffers & Plots ---
        self.buf_t.append(t)
        self.buf_z.append(z)
        self.buf_target_z.append(target_alt)
        self.buf_esc.append(base_throttle_us)

        self.curve_alt.setData(self.buf_t, self.buf_z)
        self.curve_target_alt.setData(self.buf_t, self.buf_target_z)
        self.curve_esc.setData(self.buf_t, self.buf_esc)

        # --- Update Readouts ---
        self.lbl_t.setText(f"{t:.2f} s")
        self.lbl_alt.setText(f"{z:.3f} m")
        self.lbl_target_alt.setText(f"{target_alt:.3f} m")
        self.lbl_esc.setText(f"{base_throttle_us} us")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    sim = AltitudeSimGUI()
    sim.show()
    sys.exit(app.exec_())
