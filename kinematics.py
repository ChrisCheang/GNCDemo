from math import *
import numpy as np
from pyquaternion import Quaternion
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

import time
import datetime
import csv
from datetime import date
from datetime import datetime

import pyautogui as pyg

# --- NEW IMPORTS FOR PYQT5 & MATPLOTLIB 3D ---
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

# https://stackoverflow.com/questions/4870393/rotating-coordinate-system-via-a-quaternion
# https://kieranwynn.github.io/pyquaternion/

class TVCKinematics:

    def __init__(self, rEngine=32, hTopRing=23, hEngine=166.9, lPivot=106.9, 
                 hMount=5, rMount=203, aMax=7*pi/180, lead=2,
                 a=16, b=24, d=40, d_1=20, h_phi=100, 
                 h_theta=100+15, use_servo_offset=False):
        
        # Original Linear Actuator Geometry
        self.rEngine = rEngine
        self.hTopRing = hTopRing
        self.hEngine = hEngine
        self.lPivot = lPivot
        self.hMount = hMount
        self.rMount = rMount
        self.aMax = aMax
        self.lead = lead
        
        # New Servo Linkage Geometry
        self.a = a
        self.b = b
        self.d = d
        self.d_1 = d_1
        
        self.h_phi = h_phi
        self.offset_angle = use_servo_offset*np.arctan2(self.d_1, self.d)
        self.c_phi = sqrt((h_phi-b*sin(self.offset_angle)+d_1)**2+(d-(b*cos(self.offset_angle)+a))**2)

        self.h_theta = h_theta
        self.c_theta = sqrt((h_theta-b*sin(self.offset_angle)+d_1)**2+(d-(b*cos(self.offset_angle)+a))**2)
        self.use_servo_offset = use_servo_offset

    # rotate any point based on the two gimbal angles, defined positive for movement in the y or x direction respectively
    def rotate_gimbal_angles(self, gimbal_angles, point):
        outer = Quaternion(axis=np.array([1., 0., 0.]), angle=gimbal_angles[0]) #outer gimbal axis, colinear with x axis

        v_o = outer.rotate(point) # rotated from outer axis angle

        y = np.array([0.,1.,0.]) # original inner gimbal axis
        y_o = outer.rotate(y) # rotated inner gimbal axis

        inner = Quaternion(axis=y_o, angle=-gimbal_angles[1]) #outer gimbal axis
        return inner.rotate(v_o)

    # calculate required actuator lengths to achieve given gimbal angles (inverse kinematics)
    def actuator_lengths_gimbal(self, gimbal_angles, offset=False, unit_turns=False):
        # Actuator 1 is in the first quadrant, 2 in the second

        # actuator thrust plate mount points
        a1_thrust_plate_mount = np.array([self.rMount*cos(pi/4),self.rMount*sin(pi/4),self.hMount])
        a2_thrust_plate_mount = np.array([self.rMount*cos(3*pi/4),self.rMount*sin(3*pi/4),self.hMount])

        # Non-rotated engine actuator mount points
        a1_engine_mount_original = np.array([self.rEngine*cos(pi/4),self.rEngine*sin(pi/4),-self.lPivot])
        a2_engine_mount_original = np.array([self.rEngine*cos(3*pi/4),self.rEngine*sin(3*pi/4),-self.lPivot])

        # neutral actuator lengths from desired position
        a1length_orig = np.sqrt(np.sum((a1_engine_mount_original-a1_thrust_plate_mount)**2, axis=0))
        a2length_orig = np.sqrt(np.sum((a2_engine_mount_original-a2_thrust_plate_mount)**2, axis=0))

        # rotated actuator mount points
        a1_engine_mount = self.rotate_gimbal_angles(gimbal_angles, a1_engine_mount_original)
        a2_engine_mount = self.rotate_gimbal_angles(gimbal_angles, a2_engine_mount_original)

        # required actuator lengths from desired position
        a1length = np.sqrt(np.sum((a1_engine_mount-a1_thrust_plate_mount)**2, axis=0))
        a2length = np.sqrt(np.sum((a2_engine_mount-a2_thrust_plate_mount)**2, axis=0))

        if offset == True:
            a1length -= a1length_orig
            a2length -= a2length_orig

        if unit_turns == True:
            a1length = a1length/self.lead
            a2length = a2length/self.lead

        return [a1length,a2length]

    # calculate gimbal angles (outputs 1x2 np array) from given actuator lengths (forward kinematics)
    def gimbal_angles(self, alengths):

        def func(angles):
            # Note: Changed to pass list `angles` instead of `angles[0], angles[1]` to prevent unpacking into the boolean kwargs
            lengths = self.actuator_lengths_gimbal(angles) 
            return [lengths[0]-alengths[0],lengths[1]-alengths[1]]

        return fsolve(func,[0,0])
    
    def angle_between_vectors(self, u, v):
        dot_product = sum(i*j for i, j in zip(u, v))
        norm_u = sqrt(sum(i**2 for i in u))
        norm_v = sqrt(sum(i**2 for i in v))
        cos_theta = dot_product / (norm_u * norm_v)
        angle_rad = acos(cos_theta)
        return angle_rad

    # calculate xy component angles (angle between x perpendicular plane and thrust vector, vice versa for y) from gimbal angles
    def xy_component_angles(self, gimbal_angles):
        Tvec = self.rotate_gimbal_angles(gimbal_angles, np.array([0,0,-1]))
        xangle = pi/2-self.angle_between_vectors(Tvec, [1,0,0])
        yangle = pi/2-self.angle_between_vectors(Tvec, [0,1,0])
        return [xangle,yangle]
    
    # calculate gimbal angles from xy component angles
    def gimbal_angles_from_xy(self, xyangles):

        def func(gimbal_angles):
            xy = self.xy_component_angles(gimbal_angles)
            return [xy[0]-xyangles[0],xy[1]-xyangles[1]]

        return fsolve(func,xyangles)
    
    def spherical_angles(self, gimbal_angles): # makes programming circles (if needed) and range tests easier
        Tvec = self.rotate_gimbal_angles(gimbal_angles, [0,0,-1])
        xyproj = [Tvec[0],Tvec[1],0]  #projection of thrust vector on xy plane
        rollangle = self.angle_between_vectors(xyproj,[1,0,0])
        offsetangle = self.angle_between_vectors(Tvec,[0,0,-1])

        return [offsetangle,rollangle]
    
    def gimbal_angles_spherical(self, spherical_angles):
        
        def func(gimbal_angles):
            sphericals = self.spherical_angles(gimbal_angles)
            return [sphericals[0]-spherical_angles[0],sphericals[1]-spherical_angles[1]]
        
        return fsolve(func,spherical_angles)
    
    def actuator_lengths_spherical(self, spherical_angles, offset=False, unit_turns=False):
        gimbal_angles = self.gimbal_angles_spherical(spherical_angles)
        return self.actuator_lengths_gimbal(gimbal_angles, offset=offset, unit_turns=unit_turns)
    
    def actuator_lengths_xy(self, xyangles, offset=False, unit_turns=False):
        gimbal_angles = self.gimbal_angles_from_xy(xyangles)
        return self.actuator_lengths_gimbal(gimbal_angles, offset=offset, unit_turns=unit_turns)

    def servo_angles_gimbal(self, gimbal_angles):
        """
        Calculates the required servo angles (phi and theta) and linkage positions 
        for a 2-axis coupled rod-end linkage TVC system.
        """
        
        # 1. Unrotated motor mount points (assuming universal joint is at origin 0,0,0)
        mx_unrotated = np.array([self.d, 0.0, -self.d_1])
        my_unrotated = np.array([0.0, self.d, -self.d_1])

        # 2. 3D Rotated motor mount points based on gimbal input
        mx_rotated = self.rotate_gimbal_angles(gimbal_angles, mx_unrotated)
        my_rotated = self.rotate_gimbal_angles(gimbal_angles, my_unrotated)

        # ==========================================
        # 3. Calculate raw phi (for X-axis servo)
        # ==========================================
        m_xx, m_xy, m_xz = mx_rotated[0], mx_rotated[1], mx_rotated[2]
        
        A_phi = 2 * self.b * (m_xz - self.h_phi)
        B_phi = -2 * self.b * (m_xx - self.a)
        C_phi = self.b**2 + (m_xx - self.a)**2 + m_xy**2 + (m_xz - self.h_phi)**2 - self.c_phi**2

        disc_phi = A_phi**2 + B_phi**2 - C_phi**2
        if disc_phi < 0:
            raise ValueError(f"Kinematic limit exceeded for X servo at angles {gimbal_angles}. Linkage c_phi is too short.")
        
        # Raw angle (0 equals perfectly horizontal arm)
        phi_raw = 2 * np.arctan2(-A_phi - np.sqrt(disc_phi), C_phi - B_phi)

        # ==========================================
        # 4. Calculate raw theta (for Y-axis servo)
        # ==========================================
        m_yx, m_yy, m_yz = my_rotated[0], my_rotated[1], my_rotated[2]
        
        A_theta = 2 * self.b * (m_yz - self.h_theta)
        B_theta = -2 * self.b * (m_yy - self.a)
        C_theta = self.b**2 + m_yx**2 + (m_yy - self.a)**2 + (m_yz - self.h_theta)**2 - self.c_theta**2

        disc_theta = A_theta**2 + B_theta**2 - C_theta**2
        if disc_theta < 0:
            raise ValueError(f"Kinematic limit exceeded for Y servo at angles {gimbal_angles}. Linkage c_theta is too short.")
        
        theta_raw = 2 * np.arctan2(-A_theta - np.sqrt(disc_theta), C_theta - B_theta)

        # ==========================================
        # 5. Handle Offsets & Calculate 3D Positions
        # ==========================================
        if self.use_servo_offset:
            # Apply phase shift so output angle is 0 at neutral state
            phi = phi_raw - self.offset_angle
            theta = theta_raw - self.offset_angle
        else:
            phi = phi_raw
            theta = theta_raw

        # 3D location of the servo arm mount point (Uses raw angle to compute physical space)
        px_servo_arm = np.array([self.a + self.b * np.cos(phi_raw), 0.0, self.h_phi - self.b * np.sin(phi_raw)])
        py_servo_arm = np.array([0.0, self.a + self.b * np.cos(theta_raw), self.h_theta - self.b * np.sin(theta_raw)])

        return {
            'phi_rad': phi,
            'theta_rad': theta,
            'phi_deg': np.degrees(phi),
            'theta_deg': np.degrees(theta),
            'x_motor_mount_3d': mx_rotated,
            'y_motor_mount_3d': my_rotated,
            'x_servo_arm_3d': px_servo_arm,
            'y_servo_arm_3d': py_servo_arm
        }
    
    # ---------------------------------------------------------
    # NEW: PyQt5 Real-Time 3D Mockup
    # ---------------------------------------------------------
    def visualize_3d_mockup(self, mouse_sensitivity=1.0):
        """
        Launches a PyQt5 application with an embedded Matplotlib 3D axis.
        Tracks mouse position to simulate gimbal commands and plots real-time kinematics.
        """
        app = QApplication.instance()
        if app is None:
            app = QApplication(sys.argv)

        window = QMainWindow()
        window.setWindowTitle("TVC Servo Kinematics 3D Mockup")
        window.resize(900, 900)

        central_widget = QWidget()
        window.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Setup Matplotlib Figure
        fig = Figure(facecolor='#121212')
        canvas = FigureCanvas(fig)
        layout.addWidget(canvas)

        ax = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('#121212')
        
        # Style Axes & Make Labels 2x Larger
        ax.xaxis.label.set_color('white')
        ax.yaxis.label.set_color('white')
        ax.zaxis.label.set_color('white')
        ax.tick_params(colors='white', labelsize=20)
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        
        ax.set_xlim([-60, 60])
        ax.set_ylim([-60, 60])
        ax.set_zlim([-20, 140])
        ax.set_xlabel('X (mm)', fontsize=24)
        ax.set_ylabel('Y (mm)', fontsize=24)
        ax.set_zlabel('Z (mm)', fontsize=24)

        # Force perfectly 1:1:1 geometric scaling based on bounding box spans
        try:
            ax.set_box_aspect((120, 120, 160)) 
        except AttributeError:
            pass # Fallback for older matplotlib versions

        # Initialize Plot Artists (Lines are 2x thicker, markers 2x larger)
        arm_x_line, = ax.plot([], [], [], color='purple', linewidth=6, label="Servo Arms")
        arm_y_line, = ax.plot([], [], [], color='purple', linewidth=6)
        link_x_line, = ax.plot([], [], [], color='red', linewidth=4, label="Linkages")
        link_y_line, = ax.plot([], [], [], color='red', linewidth=4)
        
        mounts_points, = ax.plot([], [], [], 'bo', markersize=12, label="Motor Mounts")
        origin_point, = ax.plot([0], [0], [0], 'wo', markersize=12, label="Universal Joint")
        
        axis_x_line, = ax.plot([], [], [], color='red', linewidth=4)
        axis_y_line, = ax.plot([], [], [], color='green', linewidth=4)
        axis_z_line, = ax.plot([], [], [], color='blue', linewidth=4)

        ax.legend(facecolor='#222222', labelcolor='white', fontsize=20)

        # Fast real-time update function modifying artist data instead of clearing axes
        def update_plot(gx, gy):
            try:
                kin = self.servo_angles_gimbal([gx, gy])
            except ValueError:
                return # Ignore out-of-bounds mouse movements
            
            px, py = kin['x_servo_arm_3d'], kin['y_servo_arm_3d']
            mx, my = kin['x_motor_mount_3d'], kin['y_motor_mount_3d']
            
            pivot_x = [self.a, 0, self.h_phi]
            pivot_y = [0, self.a, self.h_theta]
            
            # Update Servo Arms
            arm_x_line.set_data([pivot_x[0], px[0]], [pivot_x[1], px[1]])
            arm_x_line.set_3d_properties([pivot_x[2], px[2]])
            arm_y_line.set_data([pivot_y[0], py[0]], [pivot_y[1], py[1]])
            arm_y_line.set_3d_properties([pivot_y[2], py[2]])
            
            # Update Linkages
            link_x_line.set_data([px[0], mx[0]], [px[1], mx[1]])
            link_x_line.set_3d_properties([px[2], mx[2]])
            link_y_line.set_data([py[0], my[0]], [py[1], my[1]])
            link_y_line.set_3d_properties([py[2], my[2]])
            
            # Update Engine Mount Points
            mounts_points.set_data([mx[0], my[0]], [mx[1], my[1]])
            mounts_points.set_3d_properties([mx[2], my[2]])
            
            # Update 20mm Rotated RGB Axes
            ax_x = self.rotate_gimbal_angles([gx, gy], [20, 0, 0])
            ax_y = self.rotate_gimbal_angles([gx, gy], [0, 20, 0])
            ax_z = self.rotate_gimbal_angles([gx, gy], [0, 0, 20])
            
            axis_x_line.set_data([0, ax_x[0]], [0, ax_x[1]])
            axis_x_line.set_3d_properties([0, ax_x[2]])
            axis_y_line.set_data([0, ax_y[0]], [0, ax_y[1]])
            axis_y_line.set_3d_properties([0, ax_y[2]])
            axis_z_line.set_data([0, ax_z[0]], [0, ax_z[1]])
            axis_z_line.set_3d_properties([0, ax_z[2]])
            
            canvas.draw_idle()

        # Canvas Mouse Event listener
        def on_mouse_move(event):
            if event.xdata is None or event.ydata is None: 
                return
            
            w = canvas.geometry().width()
            h = canvas.geometry().height()
            
            # Map mouse pixels to [-aMax, aMax] ranges with specified amplification parameter
            gx = ((event.x / w) - 0.5) * 2 * self.aMax * mouse_sensitivity
            gy = ((event.y / h) - 0.5) * 2 * self.aMax * mouse_sensitivity
            
            update_plot(gx, gy)

        canvas.mpl_connect('motion_notify_event', on_mouse_move)
        
        # Initialize at center
        update_plot(0, 0)
        window.show()
        app.exec_()


GNCTVCKinematics = TVCKinematics(use_servo_offset=False)

# Example call passing in an amplification factor of 1.5:
GNCTVCKinematics.visualize_3d_mockup(mouse_sensitivity=7)
