#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h" 
#include <Adafruit_DPS310.h> 
#include <Servo.h> 
#include <math.h>

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

// --- TVC KINEMATIC CONSTANTS (Ported from kinematics.py) ---
const float KIN_A = 12.0f;
const float KIN_B = 20.0f;
const float KIN_D = 32.0f;
const float KIN_D1 = 10.5f;
const float KIN_H = 43.5f;
// Pre-calculated linkage lengths based on offset_angle = 0
const float KIN_C = 54.0f;   

// --- TVC PID TUNING & LIMITS ---
float tvc_kp = 1.5f;       // Proportional gain
float tvc_ki = 0.0f;       // Integral gain
float tvc_kd = 0.2f;      // Derivative gain
float d_lpf_alpha = 0.03f;  // Low pass filter factor for derivative (0.0 to 1.0)

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
float roll_kp = 0.0f;
float roll_ki = 0.0f;
float roll_kd = 0.00f;

float roll_integral = 0.0f;
float prev_roll_deg = 0.0f;
float roll_d_filtered = 0.0f;

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
bool altitude_lock = false; // forces landing when altitude exceeds a threshold for initial testing
bool altitude_locked = false;

float alt_integral = 0.0f;
float prev_alt_error = 0.0f;
float alt_d_filtered = 0.0f;

float target_altitude = 0.5f; // Target altitude in meters
int hover_throttle_us = 1200; // Base throttle needed to maintain hover
int max_throttle_us = 1500;   // Maximum throttle limit for safety during testing

unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_US = 20000; // 50 Hz (20 ms)

BNO080 myIMU;
Adafruit_DPS310 dps; 

// --- 2-State Kinematic Altitude Kalman Filter ---
class AltitudeKalman {
public:
  float x[2] = {0.0f, 0.0f}; // State: [Altitude, Vertical Velocity]
  float P[2][2] = {
    {1.0f, 0.0f},
    {0.0f, 1.0f}
  };

  // --- TUNED: PROCESS NOISE VARIANCES ---
  float Q_z = 0.00001f; 
  float Q_v = 0.0001f;  
  
  float R_z = 1.0f; 

  void predict(float a, float dt) {
    float dt2 = dt * dt;
    
    // State prediction (Pure Kinematics)
    float z_pred = x[0] + x[1] * dt + 0.5f * a * dt2;
    float v_pred = x[1] + a * dt;

    // Jacobian F
    float F[2][2] = {
      {1.0f, dt},
      {0.0f, 1.0f}
    };

    // Covariance prediction P = F * P * F^T + Q
    float FP[2][2];
    for(int i=0; i<2; i++) {
      for(int j=0; j<2; j++) {
        FP[i][j] = F[i][0]*P[0][j] + F[i][1]*P[1][j];
      }
    }
    for(int i=0; i<2; i++) {
      for(int j=0; j<2; j++) {
        P[i][j] = FP[i][0]*F[j][0] + FP[i][1]*F[j][1];
      }
    }
    
    P[0][0] += Q_z;
    P[1][1] += Q_v;

    x[0] = z_pred;
    x[1] = v_pred;
  }

  void update(float z_meas) {
    float y = z_meas - x[0];
    float S = P[0][0] + R_z;
    
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    x[0] += K[0] * y;
    x[1] += K[1] * y;

    float I_KH[2][2] = {
      {1.0f - K[0], 0.0f},
      {-K[1],       1.0f}
    };

    float P_new[2][2];
    for(int i=0; i<2; i++) {
      for(int j=0; j<2; j++) {
        P_new[i][j] = I_KH[i][0]*P[0][j] + I_KH[i][1]*P[1][j];
      }
    }
    for(int i=0; i<2; i++) {
      for(int j=0; j<2; j++) {
        P[i][j] = P_new[i][j];
      }
    }
  }
};

AltitudeKalman kalman;
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
    for (int i = 20; i > 0; i--) {
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
}

void loop() {
  if (myIMU.hasReset()) {
    myIMU.enableLinearAccelerometer(2);
    myIMU.enableGameRotationVector(2);
  }

  // 1. Read BNO085 IMU 
  if (myIMU.dataAvailable() == true) {
    unsigned long now_micros = micros();
    
    if (prev_imu_time == 0) prev_imu_time = now_micros;
    
    float dt = (now_micros - prev_imu_time) * 1e-6f;
    prev_imu_time = now_micros;

    // --- ACCELERATION & KALMAN (X+ is UP) ---
    float accel_x = myIMU.getLinAccelX(); 

    imu_vel += accel_x * dt;
    imu_alt += imu_vel * dt + 0.5f * accel_x * dt * dt;
    
    kalman.predict(accel_x, dt);

    // --- TVC & ROLL ORIENTATION SENSING (X+ is UP) ---
    // Fetch raw quaternions (w, x, y, z)
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();

    // Rotate the Earth's "UP" vector (0, 0, 1) into the IMU's body frame for TVC Pitch/Yaw
    float up_x = 2.0f * (qx * qz - qw * qy);
    float up_y = 2.0f * (qy * qz + qw * qx);
    float up_z = qw * qw - qx * qx - qy * qy + qz * qz;

    // Calculate Pitch/Yaw deviation from X+ vector
    float pitch_rad = atan2(-up_z, up_x); // Tilt around Y-axis
    float yaw_rad   = atan2(up_y, up_x);  // Tilt around Z-axis
    float pitch_deg = pitch_rad * (180.0f / PI);
    float yaw_deg   = yaw_rad * (180.0f / PI);

    // Calculate pure Roll around X+ axis relative to startup orientation (prevents gimbal lock)
    // q_diff = q_init^-1 * q_curr
    float diff_w = qw_init * qw + qx_init * qx + qy_init * qy + qz_init * qz;
    float diff_x = qw_init * qx - qx_init * qw - qy_init * qz + qz_init * qy;
    // Extract rotation specifically around the X axis from the relative quaternion
    float raw_roll_rad = 2.0f * atan2(diff_x, diff_w);
    float roll_deg = raw_roll_rad * (180.0f / PI);
    
    int escTop_us = 1000;
    int escBot_us = 1000;

    // --- PID CONTROLLER IMPLEMENTATION ---
    if (pid_first_run) {
      prev_pitch_deg = pitch_deg;
      prev_yaw_deg = yaw_deg;
      prev_roll_deg = roll_deg;
      prev_alt_error = target_altitude - kalman.x[0];
      pid_first_run = false;
    }

    if (dt > 0.0001f) {
      // 1. Calculate Altitude Error
      float current_alt = kalman.x[0];
      float alt_error = target_altitude - current_alt;

      // 2. Integral Terms with anti-windup clamping
      pitch_integral += pitch_deg * dt;
      yaw_integral += yaw_deg * dt;
      roll_integral += roll_deg * dt;
      alt_integral += alt_error * dt;
      
      pitch_integral = constrain(pitch_integral, -15.0f, 15.0f);
      yaw_integral = constrain(yaw_integral, -15.0f, 15.0f);
      roll_integral = constrain(roll_integral, -15.0f, 15.0f);
      alt_integral = constrain(alt_integral, -100.0f, 100.0f); // Anti-windup clamping for altitude

      // 3. Raw Numerical Derivatives
      float pitch_d_raw = (pitch_deg - prev_pitch_deg) / dt;
      float yaw_d_raw   = (yaw_deg - prev_yaw_deg) / dt;
      float roll_d_raw  = (roll_deg - prev_roll_deg) / dt;
      float alt_d_raw   = (alt_error - prev_alt_error) / dt;

      prev_pitch_deg = pitch_deg;
      prev_yaw_deg = yaw_deg;
      prev_roll_deg = roll_deg;
      prev_alt_error = alt_error;

      // 4. Low Pass Filter applied to Derivatives
      pitch_d_filtered = pitch_d_filtered + d_lpf_alpha * (pitch_d_raw - pitch_d_filtered);
      yaw_d_filtered   = yaw_d_filtered + d_lpf_alpha * (yaw_d_raw - yaw_d_filtered);
      roll_d_filtered  = roll_d_filtered + d_lpf_alpha * (roll_d_raw - roll_d_filtered);
      alt_d_filtered   = alt_d_filtered + d_lpf_alpha * (alt_d_raw - alt_d_filtered);

      // 5. PID Command Calculation
      float pitch_output_deg = (tvc_kp * pitch_deg) + (tvc_ki * pitch_integral) + (tvc_kd * pitch_d_filtered);
      float yaw_output_deg   = (tvc_kp * yaw_deg) + (tvc_ki * yaw_integral) + (tvc_kd * yaw_d_filtered);
      float roll_output      = (roll_kp * roll_deg) + (roll_ki * roll_integral) + (roll_kd * roll_d_filtered);
      float alt_output       = (alt_kp * alt_error) + (alt_ki * alt_integral) + (alt_kd * alt_d_filtered);

      // Clamp before IK
      float max_gimbal_deg = 20.0f;
      pitch_output_deg = constrain(pitch_output_deg, -max_gimbal_deg, max_gimbal_deg);
      yaw_output_deg   = constrain(yaw_output_deg, -max_gimbal_deg, max_gimbal_deg);

      // 6. INVERSE KINEMATICS PIPELINE (Pitch & Yaw via TVC Servos)
      float desired_gimbal_pitch_rad = pitch_output_deg * (PI / 180.0f);
      float desired_gimbal_yaw_rad   = yaw_output_deg * (PI / 180.0f);


      float servo_phi_deg = 0.0f;
      float servo_theta_deg = 0.0f;
      calculateInverseKinematics(desired_gimbal_pitch_rad, desired_gimbal_yaw_rad, servo_phi_deg, servo_theta_deg);

      int pitch_us = servo_center_us - (int)(servo_phi_deg * 11.111f); // servo arrangement does not have rotational symmetry
      int yaw_us   = servo_center_us + (int)(servo_theta_deg * 11.111f);

      pitchServo.writeMicroseconds(pitch_us);
      yawServo.writeMicroseconds(yaw_us);

      // 7. ALTITUDE & ROLL CONTROL PIPELINE (via Differential ESC RPM)
      int base_throttle_us = hover_throttle_us + (int)alt_output;
      
      // Prevent base throttle from dropping below arming or going above max before roll modulation
      base_throttle_us = constrain(base_throttle_us, 1000, max_throttle_us);

      // NOTE: Which PWM direction speeds up/slows down which rotor, and which rotor spins CW vs CCW 
      // needs to be tested and verified in hardware. If the vehicle rolls in the wrong direction 
      // during tests, swap the `+` and `-` signs below!
      escTop_us = base_throttle_us + (int)roll_output;
      escBot_us = base_throttle_us - (int)roll_output;

      // Constrain ESC PWM strictly between arming (1000) and max throttle
      escTop_us = constrain(escTop_us, 1000, max_throttle_us);
      escBot_us = constrain(escBot_us, 1000, max_throttle_us);

      // Altitude locking for initial testing
      if (kalman.x[0] > target_altitude ) {
        altitude_locked = true;
      }

      if (altitude_locked) {
        escTop_us = 1200;
        escBot_us = 1200;
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
      
      kalman.update(baro_alt);
    }

    // 2. Gate telemetry output to 50 Hz non-blocking
    unsigned long current_micros = micros();
    if (current_micros - last_telemetry_time >= TELEMETRY_INTERVAL_US) {
        last_telemetry_time = current_micros;
        
        // Only attempt to transmit if buffer is clear
        if (Serial && Serial.availableForWrite() >= 128) {
          // --- TELEMETRY SERIAL PRINT (CSV FORMAT) ---
          // Quaternions (w, x, y, z)
          Serial.print(qw);                Serial.print(",");
          Serial.print(qx);                Serial.print(",");
          Serial.print(qy);                Serial.print(",");
          Serial.print(qz);                Serial.print(",");

          // Orientation Angles (Pitch, Yaw, Roll in deg)
          Serial.print(pitch_deg);         Serial.print(",");
          Serial.print(yaw_deg);           Serial.print(",");
          Serial.print(roll_deg);          Serial.print(",");

          // Angular Rates (Pitch, Yaw, Roll rates in deg/s)
          Serial.print(pitch_d_filtered);  Serial.print(",");
          Serial.print(yaw_d_filtered);    Serial.print(",");
          Serial.print(roll_d_filtered);   Serial.print(",");

          // Kalman Filtered 1D Altitude (meters)
          Serial.print(kalman.x[0], 4);       Serial.print(",");

          // Top & Bottom Motor Pulse Widths (microseconds)
          Serial.print(escTop_us);            Serial.print(",");
          Serial.print(escBot_us);            Serial.print(",");

          // Servo Command Angles (deg)
          Serial.print(pitchServo.readMicroseconds());     Serial.print(",");
          Serial.println(yawServo.readMicroseconds());
        }
    }

  }
}
