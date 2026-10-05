import sys
import time
import numpy as np
import serial

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QVBoxLayout, QLabel, QMessageBox
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QQuaternion, QMatrix4x4

import pyqtgraph as pg
import pyqtgraph.opengl as gl

from OpenGL.GL import (
    GL_LINE_SMOOTH,
    GL_LINE_SMOOTH_HINT,
    GL_LINES,
    GL_NICEST,
    glBegin,
    glColor4f,
    glEnable,
    glEnd,
    glHint,
    glLineWidth,
    glVertex3f,
)

# ==========================================
# SET YOUR COM PORT HERE
# ==========================================
TARGET_PORT = "COM7"
BAUD_RATE = 115200
# ==========================================


# ---------------------------------------------------------
# Asynchronous Serial Processing Thread 
# (Using strict DTR/RTS logic for Teensy USB CDC)
# ---------------------------------------------------------
class SerialWorker(QThread):
    data_received = pyqtSignal(list)

    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.start_time = time.time()
        self.ser = None

    def run(self):
        try:
            # Initialize serial object
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = self.baudrate
            self.ser.timeout = 1
            
            # Open port and assert handshaking (CRITICAL FOR TEENSY)
            self.ser.open()
            self.ser.dtr = True
            self.ser.rts = True
            
            # Give the USB stack a moment to wake up, then clear old data
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            
        except Exception as e:
            print(f"Worker failed to open {self.port}: {e}")
            return

        while self.running:
            try:
                raw_bytes = self.ser.readline()
                if not raw_bytes: 
                    continue
                
                line_str = raw_bytes.decode('utf-8', errors='ignore').strip()
                parts = line_str.split(',')
                
                # Updated to expect 15 telemetry values
                if len(parts) == 15:
                    current_time = time.time() - self.start_time
                    data_tuple = [current_time] + [float(val) for val in parts]
                    self.data_received.emit(data_tuple)
            except Exception:
                pass 
                
        if self.ser and self.ser.is_open:
            self.ser.close()

    def stop(self):
        self.running = False
        self.wait()


# ---------------------------------------------------------
# Telemetry Main Dashboard
# ---------------------------------------------------------
class TelemetryDashboard(QMainWindow):
    def __init__(self, port):
        super().__init__()
        self.setWindowTitle(f"Teensy 4.1 Flight Telemetry - Connected: {port}")
        self.resize(1500, 950)

        # Apply global dark mode configuration
        pg.setConfigOption('background', '#121212')
        pg.setConfigOption('foreground', 'w')
        pg.setConfigOptions(antialias=True)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QGridLayout(central_widget)

        # Rolling Buffer Configuration
        self.max_points = 2000
        self.time_data = np.zeros(self.max_points)
        self.data_buffers = {
            'euler': np.zeros((3, self.max_points)), # Pitch, Yaw, Roll
            'rates': np.zeros((3, self.max_points)), # P_rate, Y_rate, R_rate
            'alt': np.zeros((1, self.max_points)),   # Kalman Alt
            'esc': np.zeros((2, self.max_points)),   # Top Motor, Bottom Motor
            'servo': np.zeros((2, self.max_points))  # Pitch Servo, Yaw Servo
        }
        self.ptr = 0 
        self.latest_q = [1.0, 0.0, 0.0, 0.0] # Store latest quaternion for the timer

        # Create PyQtGraph 2D Plots
        self.plot_euler = pg.PlotWidget(title="Euler Angles (deg)")
        self.plot_rates = pg.PlotWidget(title="Angular Rates (deg/s)")
        self.plot_alt = pg.PlotWidget(title="Kalman Filtered Altitude (m)")
        self.plot_esc = pg.PlotWidget(title="Coaxial ESC Outputs (µs)")
        self.plot_servo = pg.PlotWidget(title="TVC Servo Commands (µs)")

        # Link X-Axes across plots
        self.plot_rates.setXLink(self.plot_euler)
        self.plot_alt.setXLink(self.plot_euler)
        self.plot_esc.setXLink(self.plot_euler)
        self.plot_servo.setXLink(self.plot_euler)

        layout.addWidget(self.plot_euler, 0, 0)
        layout.addWidget(self.plot_rates, 1, 0)
        layout.addWidget(self.plot_alt, 2, 0)
        layout.addWidget(self.plot_esc, 3, 0)
        layout.addWidget(self.plot_servo, 4, 0)

        # Plot Curves
        self.curve_pitch = self.plot_euler.plot(pen=pg.mkPen('r', width=2), name="Pitch")
        self.curve_yaw = self.plot_euler.plot(pen=pg.mkPen('g', width=2), name="Yaw")
        self.curve_roll = self.plot_euler.plot(pen=pg.mkPen('b', width=2), name="Roll")

        self.curve_prate = self.plot_rates.plot(pen=pg.mkPen('r', width=2))
        self.curve_yrate = self.plot_rates.plot(pen=pg.mkPen('g', width=2))
        self.curve_rrate = self.plot_rates.plot(pen=pg.mkPen('b', width=2))

        self.curve_alt = self.plot_alt.plot(pen=pg.mkPen('c', width=2))

        self.curve_esc_top = self.plot_esc.plot(pen=pg.mkPen('#FFA500', width=2), name="Top Motor")
        self.curve_esc_bot = self.plot_esc.plot(pen=pg.mkPen('#00FFFF', width=2), name="Bottom Motor")

        self.curve_servo_pitch = self.plot_servo.plot(pen=pg.mkPen('#FF00FF', width=2), name="Pitch Servo")
        self.curve_servo_yaw = self.plot_servo.plot(pen=pg.mkPen('#FFFF00', width=2), name="Yaw Servo")

        # 3D OpenGL Viewport
        self.view_3d = gl.GLViewWidget()
        self.view_3d.opts['distance'] = 40
        self.view_3d.setBackgroundColor('#121212')
        
        grid = gl.GLGridItem()
        grid.scale(2, 2, 2)
        self.view_3d.addItem(grid)

        # RGB Frame Axes (Red=X, Green=Y, Blue=Z)
        self.axis_item = gl.GLAxisItem()
        self.axis_item.setSize(x=10, y=10, z=10)
        
        # Base transform: rotate -90 deg around Y to set X+ as local UP
        base_transform = QMatrix4x4()
        base_transform.rotate(-90, 0, 1, 0)
        self.axis_item.setTransform(base_transform)
        
        self.view_3d.addItem(self.axis_item)
        
        layout_3d = QVBoxLayout()
        label_3d = QLabel("Quaternion Orientation (X+ Up Base)")
        label_3d.setStyleSheet("color: white; font-weight: bold; font-size: 14px; text-align: center;")
        layout_3d.addWidget(label_3d)
        layout_3d.addWidget(self.view_3d)
        
        # Updated row span to 5 so the 3D widget stretches down alongside the new 5th plot
        layout.addLayout(layout_3d, 0, 1, 5, 1)
        layout.setColumnStretch(0, 2)
        layout.setColumnStretch(1, 1)

        # Start Async Worker Thread
        self.worker = SerialWorker(port=port, baudrate=115200)
        self.worker.data_received.connect(self.update_data)
        self.worker.start()

        # GUI Update Timer (~30 FPS)
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(33)

    def update_data(self, data):
        # Unpack telemetry list
        t = data[0]
        q = data[1:5]        # [qw, qx, qy, qz]
        euler = data[5:8]    # [pitch, yaw, roll]
        rates = data[8:11]   # [prate, yrate, rrate]
        alt = data[11]       # [kalman_alt]
        escs = data[12:14]   # [esc_top, esc_bot]
        servos = data[14:16] # [servo_pitch, servo_yaw]

        # Shift ring buffers
        self.time_data[:-1] = self.time_data[1:]
        self.time_data[-1] = t

        for key, vals in zip(['euler', 'rates', 'alt', 'esc', 'servo'], [euler, rates, [alt], escs, servos]):
            self.data_buffers[key][:, :-1] = self.data_buffers[key][:, 1:]
            for i in range(len(vals)):
                self.data_buffers[key][i, -1] = vals[i]

        if self.ptr < self.max_points:
            self.ptr += 1
            
        # Save quaternion to be updated on the next GUI timer tick
        self.latest_q = q 

    def update_gui(self):
        # Only attempt to update if we have actually received data
        if self.ptr > 0:
            self.update_plots()
            self.update_3d(self.latest_q)

    def update_plots(self):
        t_valid = self.time_data[-self.ptr:]
        
        # Update 2D Curves
        self.curve_pitch.setData(t_valid, self.data_buffers['euler'][0, -self.ptr:])
        self.curve_yaw.setData(t_valid, self.data_buffers['euler'][1, -self.ptr:])
        self.curve_roll.setData(t_valid, self.data_buffers['euler'][2, -self.ptr:])
        
        self.curve_prate.setData(t_valid, self.data_buffers['rates'][0, -self.ptr:])
        self.curve_yrate.setData(t_valid, self.data_buffers['rates'][1, -self.ptr:])
        self.curve_rrate.setData(t_valid, self.data_buffers['rates'][2, -self.ptr:])
        
        self.curve_alt.setData(t_valid, self.data_buffers['alt'][0, -self.ptr:])
        
        self.curve_esc_top.setData(t_valid, self.data_buffers['esc'][0, -self.ptr:])
        self.curve_esc_bot.setData(t_valid, self.data_buffers['esc'][1, -self.ptr:])

        self.curve_servo_pitch.setData(t_valid, self.data_buffers['servo'][0, -self.ptr:])
        self.curve_servo_yaw.setData(t_valid, self.data_buffers['servo'][1, -self.ptr:])

        # Oscilloscope-style 10-second Discrete Page Shift
        current_time = t_valid[-1]
        window_size = 10.0
        
        x_min = (current_time // window_size) * window_size
        x_max = x_min + window_size
        
        self.plot_euler.setXRange(x_min, x_max, padding=0)

    def update_3d(self, q):
        qw, qx, qy, qz = q

        # Override paint on first call to set OpenGL line width to 4x (default is 1.0)
        if not hasattr(self.axis_item, "_thickened"):
            orig_paint = self.axis_item.paint

            def thick_paint():
                glLineWidth(16.0)
                orig_paint()

            self.axis_item.paint = thick_paint
            self.axis_item._thickened = True

        # Construct Qt Quaternion object
        quat = QQuaternion(qw, qx, qy, qz)

        # Base Transform (X+ UP) + Live Rotation
        transform = QMatrix4x4()
        #transform.rotate(-90, 0, 1, 0)
        transform.rotate(quat)

        self.axis_item.setTransform(transform)

    def closeEvent(self, event):
        self.gui_timer.stop()
        self.worker.stop()
        event.accept()


# ---------------------------------------------------------
# Execution Entry Point
# ---------------------------------------------------------
if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # Pre-flight check to ensure the port is available
    try:
        test_ser = serial.Serial()
        test_ser.port = TARGET_PORT
        test_ser.baudrate = BAUD_RATE
        test_ser.timeout = 1
        
        test_ser.open()
        test_ser.dtr = True
        test_ser.rts = True
        test_ser.close()
        
    except serial.SerialException as e:
        msg = QMessageBox()
        msg.setIcon(QMessageBox.Critical)
        msg.setWindowTitle("Connection Error")
        msg.setText(f"Could not open {TARGET_PORT}.\n\nError: {e}\n\nEnsure the Arduino IDE Serial Monitor is closed and the board is plugged in.")
        msg.exec_()
        sys.exit()
        
    # Start main application
    window = TelemetryDashboard(TARGET_PORT)
    window.show()
    sys.exit(app.exec_())
