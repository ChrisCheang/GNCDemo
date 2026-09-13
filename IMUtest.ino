#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h" 
#include <Adafruit_DPS310.h> 
#include <Servo.h> 

#define BNO085_CS   10
#define BNO085_INT  9
#define BNO085_RST  8
#define DPS_CS      4   

#define PWM_PIN_ROLL  5 
#define PWM_PIN_PITCH 6 
#define ESC_PIN       2   
#define POT_PIN       14  

BNO080 myIMU;
Adafruit_DPS310 dps; 
Servo esc;

// --- 3-State 1D Altitude Kalman Filter ---
class AltitudeKalman {
public:
  float x[3] = {0.0f, 0.0f, 0.0f}; // State: [Altitude, Vertical Velocity, Accel Bias]
  float P[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
  };

  // Process & Measurement Noise Variances (Tune these if needed)
  float Q_z = 0.001f;
  float Q_v = 0.01f;
  float Q_bias = 0.005f;
  float R_z = 0.1f; 

  void predict(float a, float dt) {
    float dt2 = dt * dt;
    
    // State prediction
    float z_pred = x[0] + x[1] * dt + 0.5f * (a - x[2]) * dt2;
    float v_pred = x[1] + (a - x[2]) * dt;
    float bias_pred = x[2];

    // Jacobian F
    float F[3][3] = {
      {1.0f, dt, -0.5f * dt2},
      {0.0f, 1.0f, -dt},
      {0.0f, 0.0f, 1.0f}
    };

    // Covariance prediction P = F * P * F^T + Q
    float FP[3][3];
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        FP[i][j] = F[i][0]*P[0][j] + F[i][1]*P[1][j] + F[i][2]*P[2][j];
      }
    }
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        P[i][j] = FP[i][0]*F[j][0] + FP[i][1]*F[j][1] + FP[i][2]*F[j][2];
      }
    }
    
    P[0][0] += Q_z;
    P[1][1] += Q_v;
    P[2][2] += Q_bias;

    x[0] = z_pred;
    x[1] = v_pred;
    x[2] = bias_pred;
  }

  void update(float z_meas) {
    float y = z_meas - x[0];
    float S = P[0][0] + R_z;
    
    float K[3];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;
    K[2] = P[2][0] / S;

    x[0] += K[0] * y;
    x[1] += K[1] * y;
    x[2] += K[2] * y;

    float I_KH[3][3] = {
      {1.0f - K[0], 0.0f, 0.0f},
      {-K[1],       1.0f, 0.0f},
      {-K[2],       0.0f, 1.0f}
    };

    float P_new[3][3];
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        P_new[i][j] = I_KH[i][0]*P[0][j] + I_KH[i][1]*P[1][j] + I_KH[i][2]*P[2][j];
      }
    }
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++) {
        P[i][j] = P_new[i][j];
      }
    }
  }
};

AltitudeKalman kalman;
// ---------------------------------------------

float qOffsetReal = 1.0;
float qOffsetK = 0.0;
float baselinePressure = 0.0;

float pwmFreq = 333.0;
float pwmMin = 500;
float pwmMax = 2500;
float pwmMid = 1520; 

const int ESC_MIN_PULSE = 1000; 
const int ESC_MAX_PULSE = 2000; 
const int ESC_DEADBAND  = 1040; 

uint16_t neutral_duty = 1520;
unsigned long last_valid_data_time = 0;
bool failsafe_active = false;

void initHardwareWatchdog() {
  WDOG1_WCR = WDOG_WCR_WDE | WDOG_WCR_SRS | WDOG_WCR_WT(1); 
}

inline void feedHardwareWatchdog() {
  WDOG1_WSR = 0x5555;
  WDOG1_WSR = 0xAAAA;
}

void setup() {
  Serial.begin(115200);
  delay(100); 
  
  Serial.println("Teensy boot started...");

  esc.attach(ESC_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
  esc.writeMicroseconds(ESC_MIN_PULSE);

  pinMode(BNO085_CS, OUTPUT);
  pinMode(DPS_CS, OUTPUT);
  digitalWrite(BNO085_CS, HIGH);
  digitalWrite(DPS_CS, HIGH);
  
  pinMode(BNO085_RST, OUTPUT);
  digitalWrite(BNO085_RST, HIGH);

  pinMode(BNO085_INT, INPUT_PULLUP); 

  pinMode(PWM_PIN_ROLL, OUTPUT);
  pinMode(PWM_PIN_PITCH, OUTPUT);
  
  *(portConfigRegister(PWM_PIN_ROLL)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3);
  *(portConfigRegister(PWM_PIN_PITCH)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3);
  
  analogWriteFrequency(PWM_PIN_ROLL, pwmFreq);  
  analogWriteFrequency(PWM_PIN_PITCH, pwmFreq); 
  analogWriteResolution(16); 

  neutral_duty = 1520;
  analogWrite(PWM_PIN_ROLL, neutral_duty);
  analogWrite(PWM_PIN_PITCH, neutral_duty);

  SPI.begin();

  if (!dps.begin_SPI(DPS_CS)) {
    Serial.println("WARNING: DPS310 not detected!");
  } else {
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  }

  if (myIMU.beginSPI(BNO085_CS, 255, BNO085_INT, BNO085_RST, 3000000) == false) {
    Serial.println("WARNING: BNO085 not detected!");
  } else {
    myIMU.enableRotationVector(2.5); 
    myIMU.enableLinearAccelerometer(2.5); // Required for KF Z-axis acceleration
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (myIMU.dataAvailable()) { 
      myIMU.getQuatReal(); 
      last_valid_data_time = millis(); 
    }
  }

  unsigned long pressureTimeout = millis();
  sensors_event_t temp_event, pressure_event;
  while (!dps.getEvents(&temp_event, &pressure_event)) {
    if (millis() - pressureTimeout > 3000) {
      pressure_event.pressure = 101325.0; 
      break;
    }
    delay(10);
  }
  baselinePressure = pressure_event.pressure;

  initHardwareWatchdog();
}

void loop() {
  
  if (myIMU.hasReset()) {
    Serial.println("IMU Brownout Detected. Re-enabling...");
    myIMU.enableRotationVector(2.5);
    myIMU.enableLinearAccelerometer(2.5);
  }

  if (millis() - last_valid_data_time > 100) {
    if (!failsafe_active) {
      Serial.println("FAILSAFE TRIGGERED: Loss of IMU telemetry!");
      esc.writeMicroseconds(ESC_MIN_PULSE);
      analogWrite(PWM_PIN_ROLL, neutral_duty);
      analogWrite(PWM_PIN_PITCH, neutral_duty);
      failsafe_active = true;
    }
  }

  if (myIMU.dataAvailable() == true) {
    
    last_valid_data_time = millis();
    failsafe_active = false;
    feedHardwareWatchdog(); 

    // --- Kalman Filter Timing ---
    static unsigned long last_kalman_time = 0;
    unsigned long now_micros_kf = micros();
    if (last_kalman_time == 0) last_kalman_time = now_micros_kf;
    float dt_kf = (now_micros_kf - last_kalman_time) * 1e-6f;
    last_kalman_time = now_micros_kf;

    float qI = myIMU.getQuatI();
    float qJ = myIMU.getQuatJ();
    float qK = myIMU.getQuatK();
    float qR = myIMU.getQuatReal();

    float corr_qR = (qOffsetReal * qR) - (qOffsetK * qK);
    float corr_qI = (qOffsetReal * qI) - (qOffsetK * qJ);
    float corr_qJ = (qOffsetReal * qJ) + (qOffsetK * qI);
    float corr_qK = (qOffsetReal * qK) + (qOffsetK * qR);

    // --- Retrieve & Rotate Body Acceleration to Earth Z ---
    float ax = myIMU.getLinAccelX();
    float ay = myIMU.getLinAccelY();
    float az = myIMU.getLinAccelZ();

    float a_earth_z = 2.0f * (corr_qI * corr_qK - corr_qR * corr_qJ) * ax +
                      2.0f * (corr_qJ * corr_qK + corr_qR * corr_qI) * ay +
                      (1.0f - 2.0f * (corr_qI * corr_qI + corr_qJ * corr_qJ)) * az;

    // Execute High-Frequency IMU Prediction
    kalman.predict(a_earth_z, dt_kf);

    float roll_rad = atan2(2.0 * (corr_qR * corr_qI + corr_qJ * corr_qK), 
                           1.0 - 2.0 * (corr_qI * corr_qI + corr_qJ * corr_qJ));

    float sinp = 2.0 * (corr_qR * corr_qJ - corr_qK * corr_qI);
    sinp = constrain(sinp, -1.0, 1.0);
    float pitch_rad = asin(sinp);

    float mag_factor = 4;
    float offset = 0;
    
    float pulse_us_roll = pwmMid + (mag_factor * roll_rad / PI) * 1000.0 + offset;
    float pulse_us_pitch = pwmMid + (mag_factor * pitch_rad / PI) * 1000.0 + offset;
    
    pulse_us_roll = constrain(pulse_us_roll, pwmMin, pwmMax);
    pulse_us_pitch = constrain(pulse_us_pitch, pwmMin, pwmMax);

    float us_per_period = 1000000.0 / pwmFreq;
    uint16_t duty_cycle_roll = (uint16_t)((pulse_us_roll / us_per_period) * 65535.0);
    uint16_t duty_cycle_pitch = (uint16_t)((pulse_us_pitch / us_per_period) * 65535.0);
    
    analogWrite(PWM_PIN_ROLL, duty_cycle_roll);
    analogWrite(PWM_PIN_PITCH, duty_cycle_pitch);

    int rawAnalog = analogRead(POT_PIN);

    static float filteredAnalog = -1.0;
    static unsigned long last_pot_time = 0;
    unsigned long now_micros = micros();

    if (filteredAnalog < 0.0) {
      filteredAnalog = (float)rawAnalog; 
    } else {
      float dt = (now_micros - last_pot_time) * 1e-6f;
      float tau = 0.5f; 
      float alpha = dt / (tau + dt);
      filteredAnalog += alpha * ((float)rawAnalog - filteredAnalog);
    }
    last_pot_time = now_micros;

    int pulse_us_esc = map((int)filteredAnalog, 0, 1023, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    if (pulse_us_esc < ESC_DEADBAND) {
      pulse_us_esc = ESC_MIN_PULSE;
    }
    
    esc.writeMicroseconds(pulse_us_esc);

    sensors_event_t temp_event, pressure_event;
    if (dps.getEvents(&temp_event, &pressure_event)) {
      float alt_change_m = 44330.0 * (1.0 - pow(pressure_event.pressure / baselinePressure, 0.190295));

      // Execute Low-Frequency Barometer Update
      kalman.update(alt_change_m);

      Serial.print(corr_qI, 7);
      Serial.print(",");
      Serial.print(corr_qJ, 7);
      Serial.print(",");
      Serial.print(corr_qK, 7);
      Serial.print(",");
      Serial.print(corr_qR, 7);
      Serial.print(",");
      Serial.print(alt_change_m, 2); // Raw Baro Altitude
      Serial.print(",");
      Serial.print(kalman.x[0], 2);  // Fused Altitude
      Serial.print(",");
      Serial.print(kalman.x[1], 2);  // Fused Velocity Z
      Serial.print(",");
      Serial.println(pulse_us_esc);
    }
  }
}
