#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h" 
#include <Adafruit_DPS310.h> 
#include <Servo.h> 
#include <math.h>

#include <Arduino.h>
#include <MAVLink.h>

// ---------------------------------------------------------
// HARDWARE DEFINITIONS 
// ---------------------------------------------------------
#define BNO085_CS   10
#define BNO085_INT  9
#define BNO085_RST  8
#define DPS_CS      4   

// --- TVC SERVO DEFINITIONS ---
#define SERVO_PITCH_PIN 6  
#define SERVO_YAW_PIN   5  

// --- ESC DEFINITIONS ---
#define ESC_TOP_PIN     2
#define ESC_BOT_PIN     3

Servo pitchServo;
Servo yawServo;
Servo escTop;
Servo escBot;

// --- MISSION TIMER & ALTITUDE PROFILE ---
unsigned long loop_start_micros = 0;
bool loop_timer_started = false;
float t = 0.0f; // Time in seconds to microsecond precision

// --- MTF-01P Sensor Offset from Center of Mass (CoM) ---
// Defined in the Body-Fixed NWU Frame (North/Forward=X, West/Left=Y, Up=Z).
// A sensor mounted forward, left, and below the CoM:
const float SENSOR_OFFSET_X =  0.13f; // 13cm forward of CoM
const float SENSOR_OFFSET_Y =  0.15f; // 15cm left of CoM
const float SENSOR_OFFSET_Z = -0.36f; // 40cm below CoM (Z is UP in NWU, so below is negative)

// If the height of the CoM changes, remember to update below
float CoM_height = 0.57f;

// --- TVC KINEMATIC CONSTANTS (Ported from kinematics.py) ---
const float KIN_A = 12.0f;
const float KIN_B = 20.0f;
const float KIN_D = 32.0f;
const float KIN_D1 = 10.5f;
const float KIN_H = 43.5f;
// Pre-calculated linkage lengths based on offset_angle = 0
const float KIN_C = 54.0f;   

// --- OUTER POSITION PID TUNING & TARGETS ---
float pos_kp = 0.3f;        // Position proportional gain
float pos_ki = 0.0f;       // Position integral gain
float pos_kd = 0.2f;        // Position derivative gain

float target_px = 0.0f;     // Earth-fixed X target position (m)
float target_py = 0.0f;     // Earth-fixed Y target position (m)

float pos_x_integral = 0.0f;
float pos_y_integral = 0.0f;

float prev_px_c = 0.0f;
float prev_py_c = 0.0f;
float vx_c_filtered = 0.0f;
float vy_c_filtered = 0.0f;

float prev_pitch_err = 0.0f;
float prev_yaw_err = 0.0f;

// --- TVC PID TUNING & LIMITS ---
float tvc_kp = 0.4f;       // Proportional gain
float tvc_ki = 0.0f;       // Integral gain
float tvc_kd = 0.1f;      // Derivative gain
float d_lpf_alpha = 0.02f;  // Low pass filter factor for derivative (0.0 to 1.0)

int servo_center_us = 1500; // Center position in microseconds
int servo_limit_us = 333;   // Max deflection from center (~30 deg = 333us)

int pitch_us = servo_center_us;
int yaw_us = servo_center_us;

// TVC PID State Variables
float pitch_integral = 0.0f;
float yaw_integral = 0.0f;
float prev_pitch_deg = 0.0f;
float prev_yaw_deg = 0.0f;
float pitch_d_filtered = 0.0f;
float yaw_d_filtered = 0.0f;
bool pid_first_run = true;

// --- ROLL PID TUNING & LIMITS ---
float roll_kp = 2.0f;
float roll_ki = 0.0f;
float roll_kd = 0.4f;

float roll_integral = 0.0f;
float prev_roll_deg = 0.0f;
float roll_d_filtered = 0.0f;

float ax_cmd = 0.0f;
float ay_cmd = 0.0f;
float target_pitch_deg = 0.0f;
float target_yaw_deg = 0.0f;

// Initial orientation quaternions for gimbal-lock-free roll 
float qw_init = 1.0f;
float qx_init = 0.0f;
float qy_init = 0.0f;
float qz_init = 0.0f;

// --- ALTITUDE PID TUNING & LIMITS ---
float alt_kp = 500.0f;
float alt_ki = 150.0f;
float alt_kd = 250.0f;

bool calibrate_esc = false; // Set to true before uploading if you need to recalibrate the ESCs


float alt_integral = 0.0f;
float prev_alt_error = 0.0f;
float alt_d_filtered = 0.0f;

float target_altitude = 1.0f; // Target altitude in meters
int hover_throttle_us = 1800; // Base throttle needed to maintain hover
int max_throttle_us = 2000;   // Maximum throttle limit for safety during testing

unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_US = 20000; // 50 Hz (20 ms)

// --- Optical Flow Variables ---
float flow_vel_x_m_s = 0.0f;  // Velocity in X (m/s)
float flow_vel_y_m_s = 0.0f;  // Velocity in Y (m/s)
uint8_t flow_quality = 0;     // Optical flow confidence (0-255)

// --- ToF (Distance) Variables ---
float tof_distance_m = 0.0f;  // Distance to ground (meters)
uint8_t tof_strength = 0;     // Signal quality / strength (0-100)
uint8_t tof_precision = 0;    // Measurement covariance/precision (if populated by Micoair)

// Timestamps for failsafe checks
unsigned long last_flow_ms = 0;
unsigned long last_tof_ms = 0;

float R00 = 1.0f;
float R01 = 0.0f;
float R02 = 0.0f;
float R10 = 0.0f;
float R11 = 1.0f;
float R12 = 0.0f;
float R20 = 0.0f;
float R21 = 0.0f;
float R22 = 1.0f;
float delta_p_x = 0.0f;
float delta_p_y = 0.0f;

// post-kalman px py (use these instead of kalman.x[0] and kalman.x[1])
float px_c = 0.0f;
float py_c = 0.0f; 
float pos_lpf_alpha = 0.03f;   // Adjustable LPF alpha for px_c and py_c


// --- Global Origin & Latch State Variables ---
bool has_taken_off = false;
float px_bias = 0.0f;
float py_bias = 0.0f;

BNO080 myIMU;
Adafruit_DPS310 dps; 

class StateEstimator6D {
public:
  // State Vector: [px, py, pz, vx, vy, vz] in Earth-Fixed NWU Frame
  float x[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  // 6x6 Error Covariance Matrix
  float P[6][6] = {0.0f};

  // --- PROCESS NOISE VARIANCES (Q) ---
  float Q_p = 0.0001f; // Position process noise
  float Q_v = 0.02f;  // Velocity process noise

  // --- MEASUREMENT NOISE VARIANCES (R) ---
  float R_z    = 0.50f; // ToF Altitude measurement variance
  float R_flow = 0.2f; // Optical Flow velocity measurement variance

  StateEstimator6D() {
    for (int i = 0; i < 6; i++) P[i][i] = 1.0f;
  }

  // ---------------------------------------------------------
  // 1. PREDICT STEP (Using Earth-Frame Accelerations)
  // ---------------------------------------------------------
  void predict(float ax, float ay, float az, float dt, float qw, float qx, float qy, float qz) {
    float dt2 = dt * dt;

    // Compute Body-to-Earth 3x3 Rotation Matrix R_BW from Quaternion
    float qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz;

    float R00 = 1.0f - 2.0f * (qy2 + qz2);
    float R01 = 2.0f * (qx * qy - qw * qz);
    float R02 = 2.0f * (qx * qz + qw * qy);

    float R10 = 2.0f * (qx * qy + qw * qz);
    float R11 = 1.0f - 2.0f * (qx2 + qz2);
    float R12 = 2.0f * (qy * qz - qw * qx);

    float R20 = 2.0f * (qx * qz - qw * qy);
    float R21 = 2.0f * (qy * qz + qw * qx);
    float R22 = 1.0f - 2.0f * (qx2 + qy2);

    // Rotate body-frame accelerations [ax, ay, az] into Earth-fixed NWU frame
    float ax_e = R00 * ax + R01 * ay + R02 * az;
    float ay_e = R10 * ax + R11 * ay + R12 * az;
    float az_e = R20 * ax + R21 * ay + R22 * az;

    // State Prediction using Earth-Frame Accelerations
    float x_pred[6];
    x_pred[0] = x[0] + x[3] * dt + 0.5f * ax_e * dt2;
    x_pred[1] = x[1] + x[4] * dt + 0.5f * ay_e * dt2;
    x_pred[2] = x[2] + x[5] * dt + 0.5f * az_e * dt2;
    x_pred[3] = x[3] + ax_e * dt;
    x_pred[4] = x[4] + ay_e * dt;
    x_pred[5] = x[5] + az_e * dt;

    float F[6][6] = {0.0f};
    for (int i = 0; i < 6; i++) F[i][i] = 1.0f;
    F[0][3] = dt; 
    F[1][4] = dt; 
    F[2][5] = dt;

    float FP[6][6] = {0.0f};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            FP[i][j] = F[i][0]*P[0][j] + F[i][1]*P[1][j] + F[i][2]*P[2][j] + 
                      F[i][3]*P[3][j] + F[i][4]*P[4][j] + F[i][5]*P[5][j];
        }
    }
    
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        P[i][j] = FP[i][0]*F[j][0] + FP[i][1]*F[j][1] + FP[i][2]*F[j][2] + 
                  FP[i][3]*F[j][3] + FP[i][4]*F[j][4] + FP[i][5]*F[j][5];
      }
    }

    for (int i = 0; i < 3; i++) P[i][i] += Q_p;
    for (int i = 3; i < 6; i++) P[i][i] += Q_v;
    for (int i = 0; i < 6; i++) x[i] = x_pred[i];
  }

  // ---------------------------------------------------------
  // 2. UPDATE STEP A: Altitude (ToF Distance Sensor)
  // ---------------------------------------------------------
  void updateAltitude(float tof_distance_m, float qw, float qx, float qy, float qz) {
    // Unit Safety: convert cm to meters if sensor outputs raw cm values (> 50m ceiling check)
    if (tof_distance_m > 15.0f) {
      tof_distance_m /= 100.0f;
    }

    float R20 = 2.0f * (qx * qz - qw * qy);
    float R21 = 2.0f * (qy * qz + qw * qx);
    float R22 = 1.0f - 2.0f * (qx * qx + qy * qy);

    // Correct for slant beam and lever arm offset to find true vertical height
    float z_meas = (tof_distance_m * R22) - (R20 * SENSOR_OFFSET_X + R21 * SENSOR_OFFSET_Y + R22 * SENSOR_OFFSET_Z);

    float y = z_meas - x[2]; 
    float S = P[2][2] + R_z; 

    float K[6];
    for (int i = 0; i < 6; i++) K[i] = P[i][2] / S;
    for (int i = 0; i < 6; i++) x[i] += K[i] * y;

    float P_new[6][6];
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        P_new[i][j] = P[i][j] - K[i] * P[2][j];
      }
    }
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) P[i][j] = P_new[i][j];
    }
  }

  // ---------------------------------------------------------
  // 3. UPDATE STEP B: X/Y Planar Velocity (MAVLink #106 Optical Flow)
  // ---------------------------------------------------------
  // flow_rad_s_x: Angular flow rate around Body +X axis (rad/s, MAVLink OPTICAL_FLOW)
  // flow_rad_s_y: Angular flow rate around Body +Y axis (rad/s, MAVLink OPTICAL_FLOW)
  // wx, wy, wz: Gyro angular rates (rad/s) in Body NWU Frame
  // qw, qx, qy, qz: Body-to-Earth orientation quaternion
  // tof_distance_m: Slanted ground distance along optical axis (m)
  void updateFlow(float flow_rad_s_x, float flow_rad_s_y, 
                  float wx, float wy, float wz, 
                  float qw, float qx, float qy, float qz, 
                  float tof_distance_m = -1.0f) {
      
      // Unit Safety: convert cm to meters if sensor outputs raw cm values (> 50m ceiling check)
      if (tof_distance_m > 15.0f) {
          tof_distance_m /= 100.0f;
      }

      // Compute 3x3 Rotation Matrix (R_body_to_earth) from NWU Quaternion
      float qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz;

      float R00 = 1.0f - 2.0f * (qy2 + qz2);
      float R01 = 2.0f * (qx * qy - qw * qz);
      float R02 = 2.0f * (qx * qz + qw * qy);

      float R10 = 2.0f * (qx * qy + qw * qz);
      float R11 = 1.0f - 2.0f * (qx2 + qz2);
      float R12 = 2.0f * (qy * qz - qw * qx);

      float R20 = 2.0f * (qx * qz - qw * qy);
      float R21 = 2.0f * (qy * qz + qw * qx);
      float R22 = 1.0f - 2.0f * (qx2 + qy2);

      // Determine slanted optical axis distance D (meters)
      float D = tof_distance_m;
      if (D <= 0.05f) {
          float min_r22 = 0.2f; // Prevent division by zero during steep roll/pitch
          float effective_r22 = (R22 > min_r22) ? R22 : min_r22;
          // Fallback: reconstructed optical slant distance using state estimate x[2] (pz) and lever arm
          D = SENSOR_OFFSET_Z + (x[2] + R20 * SENSOR_OFFSET_X + R21 * SENSOR_OFFSET_Y) / effective_r22;
      }
      if (D < 0.10f) D = 0.10f; // Minimum safety floor (10 cm)

      // 1. Gyro and Lever-Arm Derotation to isolate pure translational angular flow (rad/s)
      // float w_trans_x = flow_rad_s_x - wy + (wy * SENSOR_OFFSET_Z - wz * SENSOR_OFFSET_Y) / D;
      // float w_trans_y = flow_rad_s_y + wx - (wx * SENSOR_OFFSET_Z - wz * SENSOR_OFFSET_X) / D;

      float v_trans_x = flow_vel_x_m_s;// - wy*D + (wy * SENSOR_OFFSET_Z - wz * SENSOR_OFFSET_Y);
      float v_trans_y = flow_vel_y_m_s;// + wx*D - (wx * SENSOR_OFFSET_Z - wz * SENSOR_OFFSET_X);

      // 2. Exact 2D linear system matrix inversion to recover Earth-Frame horizontal velocities (Vx, Vy)
      float rhs_x = -v_trans_x - R20 * x[5];
      float rhs_y = -v_trans_y - R21 * x[5];

      float det_M = R00 * R11 - R01 * R10;
      if (fabsf(det_M) < 1e-6f) return; 

      float v_meas_earth_x = ( R11 * rhs_x - R10 * rhs_y) / det_M;
      float v_meas_earth_y = (-R01 * rhs_x + R00 * rhs_y) / det_M;

      // 3. 2D Extended Kalman Filter Measurement Update for Vx (x[3]) and Vy (x[4])
      float y_innov[2];
      y_innov[0] = 3*v_meas_earth_x - x[3]; // 3 is artifical scaling factor (temporary)
      y_innov[1] = 3*v_meas_earth_y - x[4]; // 3 is artifical scaling factor (temporary)

      float S[2][2];
      S[0][0] = P[3][3] + R_flow;
      S[0][1] = P[3][4];
      S[1][0] = P[4][3];
      S[1][1] = P[4][4] + R_flow;

      float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
      if (fabsf(det) < 1e-6f) return; 
      
      float S_inv[2][2];
      S_inv[0][0] =  S[1][1] / det;
      S_inv[0][1] = -S[0][1] / det;
      S_inv[1][0] = -S[0][1] / det;
      S_inv[1][1] =  S[0][0] / det;

      float K[6][2];
      for (int i = 0; i < 6; i++) {
          K[i][0] = P[i][3] * S_inv[0][0] + P[i][4] * S_inv[1][0];
          K[i][1] = P[i][3] * S_inv[0][1] + P[i][4] * S_inv[1][1];
      }

      for (int i = 0; i < 6; i++) {
          x[i] += K[i][0] * y_innov[0] + K[i][1] * y_innov[1];
      }

      float P_new[6][6];
      for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
              P_new[i][j] = P[i][j] - (K[i][0] * P[3][j] + K[i][1] * P[4][j]);
          }
      }
      for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) P[i][j] = P_new[i][j];
      }
  }
};

// AltitudeKalman kalman;

StateEstimator6D kalman;
// ---------------------------------------------

// State Variables
float imu_vel = 0.0f;
float imu_alt = 0.0f;
float baro_alt = 0.0f;
float baselinePressure = 101325.0f;
float baselineTemp = 0.0f;

// Thermal Drift Compensation
float tempCompCoef = 0.0f; 

unsigned long prev_imu_time = 0;

// ---------------------------------------------------------
// INVERSE KINEMATICS SOLVER (Ported from kinematics.py)
// ---------------------------------------------------------
void calculateInverseKinematics(float gimbal_pitch_rad, float gimbal_yaw_rad, float &servo_phi_deg, float &servo_theta_deg) {
  // 1. Rotation Matrix Generation (Rx * Ry) matching rotate_gimbal_angles intrinsic sequence
  float cp = cos(gimbal_pitch_rad);
  float sp = sin(gimbal_pitch_rad);
  float cy = cos(gimbal_yaw_rad);  
  float sy = -sin(gimbal_yaw_rad); 

  // Combined Rotation Matrix Components
  float R00 = cy;               float R01 = 0.0f;  float R02 = sy;
  float R10 = sp * -sy;         float R11 = cp;    float R12 = sp * cy;
  float R20 = cp * -sy;         float R21 = -sp;   float R22 = cp * cy;
  
  // 2. Rotated motor mount points
  float m_xx = R00 * KIN_D + R02 * (-KIN_D1);
  float m_xy = R10 * KIN_D + R12 * (-KIN_D1);
  float m_xz = R20 * KIN_D + R22 * (-KIN_D1);

  float m_yx = R01 * KIN_D + R02 * (-KIN_D1);
  float m_yy = R11 * KIN_D + R12 * (-KIN_D1);
  float m_yz = R21 * KIN_D + R22 * (-KIN_D1);

  // 3. Solve X Servo Angle (Phi)
  float A_phi = 2.0f * KIN_B * (m_xz - KIN_H);
  float B_phi = -2.0f * KIN_B * (m_xx - KIN_A);
  float C_phi = (KIN_B*KIN_B) + (m_xx - KIN_A)*(m_xx - KIN_A) + (m_xy*m_xy) + (m_xz - KIN_H)*(m_xz - KIN_H) - (KIN_C*KIN_C);
  
  // Clamp C_phi to reachable limit to prevent negative discriminant and servo flipping
  float max_C_phi = sqrt((A_phi * A_phi) + (B_phi * B_phi));
  if (C_phi > max_C_phi) C_phi = max_C_phi;
  if (C_phi < -max_C_phi) C_phi = -max_C_phi;

  float disc_phi = (A_phi*A_phi) + (B_phi*B_phi) - (C_phi*C_phi);
  if (disc_phi < 0) disc_phi = 0.0f; 
  float phi_rad = 2.0f * atan2(-A_phi - sqrt(disc_phi), C_phi - B_phi);

  // 4. Solve Y Servo Angle (Theta)
  float A_theta = 2.0f * KIN_B * (m_yz - KIN_H);
  float B_theta = -2.0f * KIN_B * (m_yy - KIN_A);
  float C_theta = (KIN_B*KIN_B) + (m_yx*m_yx) + (m_yy - KIN_A)*(m_yy - KIN_A) + (m_yz - KIN_H)*(m_yz - KIN_H) - (KIN_C*KIN_C);
  
  // Clamp C_theta to reachable limit to prevent negative discriminant and servo flipping
  float max_C_theta = sqrt((A_theta * A_theta) + (B_theta * B_theta));
  if (C_theta > max_C_theta) C_theta = max_C_theta;
  if (C_theta < -max_C_theta) C_theta = -max_C_theta;

  float disc_theta = (A_theta*A_theta) + (B_theta*B_theta) - (C_theta*C_theta);
  if (disc_theta < 0) disc_theta = 0.0f; 
  float theta_rad = 2.0f * atan2(-A_theta - sqrt(disc_theta), C_theta - B_theta);

  // Convert to degrees
  servo_phi_deg = phi_rad * (180.0f / PI);
  servo_theta_deg = theta_rad * (180.0f / PI);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // 1. Setup TVC Servos
  pitchServo.attach(SERVO_PITCH_PIN);
  yawServo.attach(SERVO_YAW_PIN);
  pitchServo.writeMicroseconds(servo_center_us);
  yawServo.writeMicroseconds(servo_center_us);

  // 2. Setup ESCs (Arming sequence)
  // --- ESC CALIBRATION & SETUP PROCESS ---

  // --- ESC CALIBRATION & SETUP PROCESS ---
  if (calibrate_esc) {
    // Note: Power the microcontroller via USB first to start the code BEFORE connecting the main LiPo battery.

    Serial.println("Step 1: Moving throttle stick to max position (2000 us).");
    // Simulate moving the throttle stick to the top position[cite: 1]
    escTop.writeMicroseconds(2000); 
    escBot.writeMicroseconds(2000);

    Serial.println("Step 2: CONNECT BATTERY NOW! Waiting for max throttle beep...");
    // 20-second countdown loop to connect XT60/battery and wait for acceptance beeps[cite: 1]
    for (int i = 10; i > 0; i--) {
      Serial.print("  Throttle MAX - Time remaining: ");
      Serial.print(i);
      Serial.println(" s");
      delay(1000);
    }

    Serial.println("Step 3: Moving throttle stick to minimum position (1000 us).");
    // Move the throttle stick to the bottom position after the two short beeps[cite: 1]
    escTop.writeMicroseconds(1000);
    escBot.writeMicroseconds(1000);

    Serial.println("Listening for LiPo cell count beeps & final ready tone...");
    // 4-second countdown loop to allow cell count beeps and final long arming tone[cite: 1]
    for (int i = 4; i > 0; i--) {
      Serial.print("  Throttle MIN - Time remaining: ");
      Serial.print(i);
      Serial.println(" s");
      delay(1000);
    }

    Serial.println(">>> ESC Calibration Complete! <<<");
  } else {
    // Standard arming sequence for normal flight
    escTop.writeMicroseconds(1000); 
    escBot.writeMicroseconds(1000);
    delay(4000); // Give ESCs time to initialize normally on standard boot
  }


  escTop.attach(ESC_TOP_PIN);
  escBot.attach(ESC_BOT_PIN);
  escTop.writeMicroseconds(1000); // 1000us standard minimum PWM for arming ESCs
  escBot.writeMicroseconds(1000);

  // Initialize PID history states
  pitch_integral = 0.0f;
  yaw_integral = 0.0f;
  roll_integral = 0.0f;
  alt_integral = 0.0f;
  prev_pitch_deg = 0.0f;
  prev_yaw_deg = 0.0f;
  prev_roll_deg = 0.0f;
  prev_alt_error = 0.0f;
  pitch_d_filtered = 0.0f;
  yaw_d_filtered = 0.0f;
  roll_d_filtered = 0.0f;
  alt_d_filtered = 0.0f;
  pid_first_run = true;
  
  // 3. SPI Bus Preparation
  pinMode(BNO085_CS, OUTPUT);
  pinMode(DPS_CS, OUTPUT);
  digitalWrite(BNO085_CS, HIGH);
  digitalWrite(DPS_CS, HIGH);
  
  pinMode(BNO085_RST, OUTPUT);
  digitalWrite(BNO085_RST, HIGH);

  pinMode(BNO085_INT, INPUT_PULLUP); 

  SPI.begin();

  // 4. Initialize DPS310 (SPI)
  if (!dps.begin_SPI(DPS_CS)) {
    Serial.println("WARNING: DPS310 not detected!");
  } else {
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  }

  // 5. Initialize BNO085 (SPI)
  if (myIMU.beginSPI(BNO085_CS, 255, BNO085_INT, BNO085_RST, 3000000) == false) {
    Serial.println("WARNING: BNO085 not detected!");
  } else {
    myIMU.enableLinearAccelerometer(2); 
    myIMU.enableGameRotationVector(2); 
    myIMU.enableGyro(2);
  }

  // 6. Let IMU Settle
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (myIMU.dataAvailable()) { 
      prev_imu_time = micros(); 
    }
  }

  // 7. Capture Initial Orientation as Zero Reference for Roll
  if (myIMU.dataAvailable()) {
    qw_init = myIMU.getQuatReal();
    qx_init = myIMU.getQuatI();
    qy_init = myIMU.getQuatJ();
    qz_init = myIMU.getQuatK();
  }

  // 8. Capture Baseline Pressure & Temperature
  unsigned long pressureTimeout = millis();
  sensors_event_t temp_event, pressure_event;
  while (!dps.getEvents(&temp_event, &pressure_event)) {
    if (millis() - pressureTimeout > 3000) {
      pressure_event.pressure = 1013.25; 
      temp_event.temperature = 25.0;
      break;
    }
    delay(10);
  }
  baselinePressure = pressure_event.pressure;
  baselineTemp = temp_event.temperature;

  // Teensy 4.1 RX4 (Pin 16) / TX4 (Pin 17)
  // The MTF-01P defaults to 115200 baud for MAVLink
  Serial4.begin(115200); 
  
  // Wait for USB Serial to connect (optional, for debugging)
  while (!Serial && millis() < 3000);
  Serial.println("MTF-01P MAVLink Parser Initialized on Serial4");

  delay(2000);
  Serial.print("setup complete");
  delay(2000);
}

void loop() {
  if (myIMU.hasReset()) {
    myIMU.enableLinearAccelerometer(2);
    myIMU.enableGameRotationVector(2);
    myIMU.enableGyro(2);
  }

  // 1. Initialize microsecond-precision timer on the first loop iteration
  if (!loop_timer_started) {
    loop_start_micros = micros();
    loop_timer_started = true;
  }
  t = (micros() - loop_start_micros) * 1e-6f;

  // 2. Map target_altitude to time profile h(t)
  if (t < 2.0f) {
    target_altitude = CoM_height;
  } 
  else if (t < 4.0f) {
    // Increases linearly from CoM_height to CoM_height + 0.5 over 2 seconds (t from 2 to 4)
    target_altitude = CoM_height + 0.5f * ((t - 2.0f) / 2.0f);
  } 
  else if (t < 5.0f) {
    // Holds for one second (t from 4 to 5) at CoM_height + 0.5
    target_altitude = CoM_height + 0.5f;
  } 
  else if (t <= 7.0f) {
    // Decreases linearly back to CoM_height over another 2 seconds (t from 5 to 7)
    target_altitude = (CoM_height + 0.5f) - 0.5f * ((t - 5.0f) / 2.0f);
  } 
  else {
    // Hold at CoM_height after profile completion
    target_altitude = CoM_height;
  }

  // 1. Read BNO085 IMU 
  if (myIMU.dataAvailable() == true) {
    unsigned long now_micros = micros();
    
    if (prev_imu_time == 0) prev_imu_time = now_micros;
    
    float dt = (now_micros - prev_imu_time) * 1e-6f;
    prev_imu_time = now_micros;

    // --- ACCELERATION & KALMAN 
    float accel_x = -myIMU.getLinAccelZ(); 
    float accel_y = myIMU.getLinAccelY();
    float accel_z = myIMU.getLinAccelX();

    imu_vel += accel_z * dt;
    imu_alt += imu_vel * dt + 0.5f * accel_z * dt * dt;

    // 1. Fetch raw quaternions from BNO085
    float raw_qw = myIMU.getQuatReal();
    float raw_qx = myIMU.getQuatI();
    float raw_qy = myIMU.getQuatJ();
    float raw_qz = myIMU.getQuatK();

    // 2. Define the mounting correction quaternion inverse (q_mount^-1)
    // Corrects the static -90 deg Y-axis pitch offset from vertical mounting
    const float m_w =  0.7071068f;
    const float m_x =  0.0f;
    const float m_y =  0.7071068f;
    const float m_z =  0.0f;

    // 3. Perform Right-Multiplication (Local Body Frame Rotation): q_body = q_raw * q_mount
    // Optimized Hamilton product where m_x = 0 and m_z = 0
    float qw = raw_qw * m_w - raw_qy * m_y;
    float qx = raw_qx * m_w - raw_qz * m_y;
    float qy = raw_qw * m_y + raw_qy * m_w;
    float qz = raw_qx * m_y + raw_qz * m_w;

    // 4. Normalize the result to prevent numerical drift in PID
    float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (norm > 0.0f) {
        qw /= norm;
        qx /= norm;
        qy /= norm;
        qz /= norm;
    }

    kalman.predict(accel_x, accel_y, accel_z, dt, qw, qx, qy, qz);

    // 5. Retrieve gyro rates from imu, NWU correction from board mount orientation applied
    float wx = -myIMU.getGyroZ();
    float wy = myIMU.getGyroY();
    float wz = myIMU.getGyroX();

    // With the mounting correction applied, the quaternion (qw, qx, qy, qz)
    // now represents a sensor sitting perfectly flat. Z+ is the vertical axis.
    
    float sin_roll = 2.0f * (qw * qz + qx * qy);
    float cos_roll = 1.0f - 2.0f * (qy * qy + qz * qz);
    float roll_rad = atan2(sin_roll, cos_roll);

    float sin_pitch = 2.0f * (qw * qy - qz * qx);
    float pitch_rad;
    if (abs(sin_pitch) >= 1.0f) {
        pitch_rad = copysign(PI / 2.0f, sin_pitch); // Clamping failsafe for floating-point bounds
    } else {
        pitch_rad = asin(sin_pitch);
    }

    float sin_yaw = 2.0f * (qw * qx + qy * qz);
    float cos_yaw = 1.0f - 2.0f * (qx * qx + qy * qy);
    float yaw_rad = atan2(sin_yaw, cos_yaw); 

    // Convert to degrees for telemetry and PID controller inputs
    float roll_deg  = roll_rad  * (180.0f / PI); // Z
    float pitch_deg = pitch_rad * (180.0f / PI); // Y
    float yaw_deg   = yaw_rad   * (180.0f / PI); // X


    // optical flow data parsing
    mavlink_message_t msg;
    mavlink_status_t status;

    // Declare optical flow rate variables (static so their values persist across serial reads)
    static float flow_rad_s_x = 0.0f;
    static float flow_rad_s_y = 0.0f;

    // Read all available data on Serial4 without blocking
    while (Serial4.available() > 0) {
      uint8_t c = Serial4.read();

      // Pass each byte to the MAVLink decoder
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        
        // Optional filter: If you specifically want to ignore other sensors 
        // and only listen to your Component ID 88, uncomment the line below:
        // if (msg.compid != 88) continue; 

        switch (msg.msgid) {
            
          // 1. Parse Optical Flow Velocity & Quality (#100)
          case MAVLINK_MSG_ID_OPTICAL_FLOW: {
            mavlink_optical_flow_t flow;
            mavlink_msg_optical_flow_decode(&msg, &flow);
            
            flow_quality = flow.quality; // Flow confidence (0 - 255)

            // Fallback logic: If float flow_comp_m_x is 0, read integer flow_x / flow_y
            // mapping from sensor axes to NWU body axes: X+ --> Y-, Y+ --> X+
            if (abs(flow.flow_comp_m_x) > 0.001f || abs(flow.flow_comp_m_y) > 0.001f) {
                flow_vel_x_m_s = flow.flow_comp_m_y;
                flow_vel_y_m_s = flow.flow_comp_m_x;
            } else {
                // Convert integer flow delta to velocity (scale factor of 1000.0f)
                flow_vel_x_m_s = (float)flow.flow_y / 1000.0f; 
                flow_vel_y_m_s = (float)flow.flow_x / 1000.0f;
            }
            // Convert ground linear velocity (m/s) to angular flow rate (rad/s):
            // omega_flow = v / D
            // if (tof_distance_m > 0.05f) {
            //     flow_rad_s_x = flow_vel_x_m_s / tof_distance_m;
            //     flow_rad_s_y = flow_vel_y_m_s / tof_distance_m;
            // } else {
            //     flow_rad_s_x = 0.0f;
            //     flow_rad_s_y = 0.0f;
            // }
            kalman.updateFlow(flow_vel_x_m_s, flow_vel_y_m_s, 
                    wx, wy, wz, 
                    qw, qx, qy, qz, 
                    tof_distance_m);
            break;
          }

          // 2. Parse ToF Distance, Strength & Precision (#132)
          case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
            mavlink_distance_sensor_t dist;
            mavlink_msg_distance_sensor_decode(&msg, &dist);
            
            // Convert current_distance from cm to meters
            tof_distance_m = (float)dist.current_distance / 100.0f; 
            
            // Extract ToF Signal Quality & Precision
            tof_strength  = dist.signal_quality; // Valid if MAVLink v2 packet
            tof_precision = dist.covariance;     // 0 if module does not transmit variance
            kalman.updateAltitude(tof_distance_m, qw, qx, qy, qz);
            break;
          }
        }
      }


      

    }

    int escTop_us = 1000;
    int escBot_us = 1000;

    // --- PID CONTROLLER IMPLEMENTATION ---
    if (pid_first_run) {
      prev_pitch_deg = pitch_deg;
      prev_yaw_deg = yaw_deg;
      prev_roll_deg = roll_deg;
      prev_alt_error = target_altitude - kalman.x[2];
      pid_first_run = false;
    }

    if (dt > 0.0001f) {
      // 1. Compute current Body-to-Earth Rotation Matrix components from active quaternions
      float qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz;

      R00 = 1.0f - 2.0f * (qy2 + qz2);
      R01 = 2.0f * (qx * qy - qw * qz);
      R02 = 2.0f * (qx * qz + qw * qy);

      R10 = 2.0f * (qx * qy + qw * qz);
      R11 = 1.0f - 2.0f * (qx2 + qz2);
      R12 = 2.0f * (qy * qz - qw * qx);

      // Check ESC command threshold to detect initial launch
      bool throttle_active = (escTop_us > 1100 || escBot_us > 1100);
      if (kalman.x[2] > CoM_height + 0.03) {
        has_taken_off = true; // Permanently latch takeoff state
      }

      // Pre-flight clamp: ONLY runs prior to the very first takeoff
      if (!has_taken_off) {
        // 1. Hold Kalman planar position and velocity states at zero while on launchpad
        kalman.x[0] = 0.0f; // px
        kalman.x[1] = 0.0f; // py
        kalman.x[3] = 0.0f; // vx
        kalman.x[4] = 0.0f; // vy

        // 2. Latch resting ground tilt offset as baseline bias
        px_bias = -delta_p_x;
        py_bias = -delta_p_y;
      }

      // Ground intersection point of the tilted optical axis relative to CoM:
      float p_x_tilted = R00 * SENSOR_OFFSET_X + R01 * SENSOR_OFFSET_Y + R02 * (SENSOR_OFFSET_Z - tof_distance_m);
      float p_y_tilted = R10 * SENSOR_OFFSET_X + R11 * SENSOR_OFFSET_Y + R12 * (SENSOR_OFFSET_Z - tof_distance_m);

      // Attitude-induced position shift relative to zero-tilt level attitude
      delta_p_x = p_x_tilted - SENSOR_OFFSET_X;
      delta_p_y = p_y_tilted - SENSOR_OFFSET_Y;

      //px_c = kalman.x[0] - delta_p_x - px_bias;
      //py_c = kalman.x[1] - delta_p_y - py_bias;

      // 2. Corrected Earth-fixed Position States with Direct In-Place LPF
      float raw_px_c = kalman.x[0] - delta_p_x - px_bias;
      float raw_py_c = kalman.x[1] - delta_p_y - py_bias;

      px_c += pos_lpf_alpha * (raw_px_c - px_c);
      py_c += pos_lpf_alpha * (raw_py_c - py_c);

      // 3. Numerical Derivative of px_c and py_c (avoiding kalman.x[3] / x[4])
      float vx_c_raw = (px_c - prev_px_c) / dt;
      float vy_c_raw = (py_c - prev_py_c) / dt;
      prev_px_c = px_c;
      prev_py_c = py_c;

      vx_c_filtered += d_lpf_alpha * (vx_c_raw - vx_c_filtered);
      vy_c_filtered += d_lpf_alpha * (vy_c_raw - vy_c_filtered);

      // 4. Outer Position PID Control Loops (Earth NWU Frame)
      float err_px = target_px - px_c;
      float err_py = target_py - py_c;

      if (has_taken_off) {
        pos_x_integral += err_px * dt;
        pos_y_integral += err_py * dt;
        pos_x_integral = constrain(pos_x_integral, -3.0f, 3.0f);
        pos_y_integral = constrain(pos_y_integral, -3.0f, 3.0f);
      } else {
        pos_x_integral = 0.0f;
        pos_y_integral = 0.0f;
      }

      ax_cmd = (pos_kp * err_px) + (pos_ki * pos_x_integral) - (pos_kd * vx_c_filtered);
      ay_cmd = (pos_kp * err_py) + (pos_ki * pos_y_integral) - (pos_kd * vy_c_filtered);

      // 5. Form Target Vehicle Attitude Vector in Earth NWU Frame
      const float g = 9.81f;
      float t_e_x = ax_cmd;
      float t_e_y = ay_cmd;
      float t_e_z = g;

      float norm_t_e = sqrtf(t_e_x * t_e_x + t_e_y * t_e_y + t_e_z * t_e_z);
      if (norm_t_e > 0.0f) {
        t_e_x /= norm_t_e;
        t_e_y /= norm_t_e;
        t_e_z /= norm_t_e;
      }

      // 6. Rotate Target Thrust Vector into Body-Fixed Frame via R_BW^T
      float t_b_x = R00 * t_e_x + R10 * t_e_y + R20 * t_e_z;
      float t_b_y = R01 * t_e_x + R11 * t_e_y + R21 * t_e_z;
      float t_b_z = R02 * t_e_x + R12 * t_e_y + R22 * t_e_z;

      // 7. Calculate Body-Fixed Target Attitude Angles (degrees)
      target_pitch_deg = atan2f(t_b_x, t_b_z) * (180.0f / PI);
      target_yaw_deg   = - atan2f(t_b_y, t_b_z) * (180.0f / PI);

      // 8. Inner Attitude Loop Error & Derivative Terms
      float pitch_err = target_pitch_deg - pitch_deg;
      float yaw_err   = target_yaw_deg - yaw_deg;

      pitch_integral += pitch_err * dt;
      yaw_integral   += yaw_err * dt;
      pitch_integral = constrain(pitch_integral, -15.0f, 15.0f);
      yaw_integral   = constrain(yaw_integral, -15.0f, 15.0f);

      float pitch_d_raw = (pitch_err - prev_pitch_err) / dt;
      float yaw_d_raw   = (yaw_err - prev_yaw_err) / dt;
      prev_pitch_err = pitch_err;
      prev_yaw_err   = yaw_err;

      pitch_d_filtered += d_lpf_alpha * (pitch_d_raw - pitch_d_filtered);
      yaw_d_filtered   += d_lpf_alpha * (yaw_d_raw - yaw_d_filtered);

      // 9. Inner TVC PID Commands
      float pitch_output_deg = (tvc_kp * pitch_err) + (tvc_ki * pitch_integral) + (tvc_kd * pitch_d_filtered);
      float yaw_output_deg   = (tvc_kp * yaw_err)   + (tvc_ki * yaw_integral)   + (tvc_kd * yaw_d_filtered);

      float max_gimbal_deg = 20.0f;
      pitch_output_deg = constrain(pitch_output_deg, -max_gimbal_deg, max_gimbal_deg);
      yaw_output_deg   = constrain(yaw_output_deg, -max_gimbal_deg, max_gimbal_deg);

      // 10. Inverse Kinematics & Servo Actuation
      float desired_gimbal_pitch_rad = - pitch_output_deg * (PI / 180.0f);
      float desired_gimbal_yaw_rad   = yaw_output_deg * (PI / 180.0f);

      float servo_phi_deg = 0.0f;
      float servo_theta_deg = 0.0f;
      calculateInverseKinematics(desired_gimbal_pitch_rad, desired_gimbal_yaw_rad, servo_phi_deg, servo_theta_deg);

      int pitch_us = servo_center_us + (int)(servo_phi_deg * 11.111f);
      int yaw_us   = servo_center_us - (int)(servo_theta_deg * 11.111f);

      // Stop/center servo commands if altitude is below CoM_height + 0.03m
      if (kalman.x[2] < (CoM_height + 0.05f)) {
        pitch_us = servo_center_us;
        yaw_us = servo_center_us;
      }

      pitchServo.writeMicroseconds(pitch_us);
      yawServo.writeMicroseconds(yaw_us);

      // 11. Altitude & Roll PID Pipeline (Differential ESC Throttle)
      float current_alt = kalman.x[2];
      float alt_error = target_altitude - current_alt;

      roll_integral += roll_deg * dt;
      alt_integral  += alt_error * dt;
      roll_integral = constrain(roll_integral, -15.0f, 15.0f);
      alt_integral  = constrain(alt_integral, -100.0f, 100.0f);

      float roll_d_raw = (roll_deg - prev_roll_deg) / dt;
      float alt_d_raw  = (alt_error - prev_alt_error) / dt;
      prev_roll_deg = roll_deg;
      prev_alt_error = alt_error;

      roll_d_filtered += d_lpf_alpha * (roll_d_raw - roll_d_filtered);
      alt_d_filtered  += d_lpf_alpha * (alt_d_raw - alt_d_filtered);

      float roll_output = (roll_kp * roll_deg) + (roll_ki * roll_integral) + (roll_kd * roll_d_filtered);
      float alt_output  = (alt_kp * alt_error) + (alt_ki * alt_integral)  + (alt_kd * alt_d_filtered);

      int base_throttle_us = hover_throttle_us + (int)alt_output;
      base_throttle_us = constrain(base_throttle_us, 1000, max_throttle_us);

      escTop_us = constrain(base_throttle_us - (int)roll_output, 1000, max_throttle_us);
      escBot_us = constrain(base_throttle_us + (int)roll_output, 1000, max_throttle_us);

      if (t > 7.0f) {
        escTop_us = 1000;
        escBot_us = 1000;
      } 

      escTop.writeMicroseconds(escTop_us);
      escBot.writeMicroseconds(escBot_us);

    }

    // 2. Read DPS310 Barometer
    sensors_event_t temp_event, pressure_event;
    if (dps.getEvents(&temp_event, &pressure_event)) {
      
      float delta_temp = temp_event.temperature - baselineTemp;
      float comp_pressure = pressure_event.pressure + (delta_temp * tempCompCoef);
      
      baro_alt = 44330.0f * (1.0f - pow(comp_pressure / baselinePressure, 0.190295f));
      
      //kalman.updateAltitude(baro_alt);
    }

    // 2. Gate telemetry output to 50 Hz non-blocking
    unsigned long current_micros = micros();
    if (current_micros - last_telemetry_time >= TELEMETRY_INTERVAL_US) {
      last_telemetry_time = current_micros;
      
      // Only attempt to transmit if buffer is clear
      if (Serial && Serial.availableForWrite() >= 128) {
        // --- TELEMETRY SERIAL PRINT (CSV FORMAT) ---
        Serial.print(t);                Serial.print(",");

        // Quaternions (w, x, y, z, NWU body fixed frame)
        Serial.print(qw);                Serial.print(",");
        Serial.print(qx);                Serial.print(",");
        Serial.print(qy);                Serial.print(",");
        Serial.print(qz);                Serial.print(",");

        // Orientation Angles (Pitch, Yaw, Roll in deg, note that pitch is around Y, yaw around X and roll around Z)
        Serial.print(pitch_deg);         Serial.print(",");
        Serial.print(yaw_deg);           Serial.print(",");
        Serial.print(roll_deg);          Serial.print(",");

        // Angular Rates (Pitch, Yaw, Roll rates in deg/s)
        Serial.print(pitch_d_filtered);  Serial.print(",");
        Serial.print(yaw_d_filtered);    Serial.print(",");
        Serial.print(roll_d_filtered);   Serial.print(",");

        // Kalman Filtered 1D Altitude (meters)
        Serial.print(kalman.x[2], 4);       Serial.print(",");

        // Top & Bottom Motor Pulse Widths (microseconds)
        Serial.print(escTop_us);            Serial.print(",");
        Serial.print(escBot_us);            Serial.print(",");

        // Servo Command Angles (deg)
        Serial.print(pitchServo.readMicroseconds());     Serial.print(",");
        Serial.print(yawServo.readMicroseconds());     Serial.print(",");

        // X and Y positions in earth fixed NWU frame (aligns with body frame at startup)
        Serial.print(px_c, 4);       Serial.print(",");
        Serial.print(py_c, 4);       Serial.print(",");

        //Serial.print(px_bias, 4);     Serial.print(",");
        //Serial.println(py_bias, 4);     Serial.print(",");

        Serial.print(target_pitch_deg, 4);       Serial.print(",");
        Serial.print(target_yaw_deg, 4);       Serial.print(",");
        Serial.println(target_altitude, 4);

        // Serial.print(",");
        // Serial.println(has_taken_off);
        // // wx wy wz, verified to output data
        // Serial.print(wx);       Serial.print(",");
        // Serial.print(wy);       Serial.print(",");   
        // Serial.println(wz);       

        // MTF01P
        Serial.printf("Dist: %.2fm (Str: %d, Prec: %d) | VelX: %.3fm/s, VelY: %.3fm/s (Flow Qual: %d)\n",
                    tof_distance_m, tof_strength, tof_precision, 
                    flow_vel_x_m_s, flow_vel_y_m_s, flow_quality);

      }
    }

  }
}
