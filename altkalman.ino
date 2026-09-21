#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h" 
#include <Adafruit_DPS310.h> 

// ---------------------------------------------------------
// HARDWARE DEFINITIONS 
// ---------------------------------------------------------
#define BNO085_CS   10
#define BNO085_INT  9
#define BNO085_RST  8
#define DPS_CS      4   

BNO080 myIMU;
Adafruit_DPS310 dps; 

// --- 2-State Kinematic Altitude Kalman Filter ---
// Bias state removed because BNO085 handles 3DOF bias internally.
class AltitudeKalman {
public:
  float x[2] = {0.0f, 0.0f}; // State: [Altitude, Vertical Velocity]
  float P[2][2] = {
    {1.0f, 0.0f},
    {0.0f, 1.0f}
  };

  // --- TUNED: PROCESS NOISE VARIANCES ---
  // Extremely low values. This tells the filter the accelerometer is near-perfect.
  float Q_z = 0.00001f; 
  float Q_v = 0.0001f;  
  
  // High measurement noise. The barometer is only trusted for slow drift correction.
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

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // 1. SPI Bus Preparation
  pinMode(BNO085_CS, OUTPUT);
  pinMode(DPS_CS, OUTPUT);
  digitalWrite(BNO085_CS, HIGH);
  digitalWrite(DPS_CS, HIGH);
  
  pinMode(BNO085_RST, OUTPUT);
  digitalWrite(BNO085_RST, HIGH);

  pinMode(BNO085_INT, INPUT_PULLUP); 

  SPI.begin();

  // 2. Initialize DPS310 (SPI)
  if (!dps.begin_SPI(DPS_CS)) {
    Serial.println("WARNING: DPS310 not detected!");
  } else {
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  }

  // 3. Initialize BNO085 (SPI)
  if (myIMU.beginSPI(BNO085_CS, 255, BNO085_INT, BNO085_RST, 3000000) == false) {
    Serial.println("WARNING: BNO085 not detected!");
  } else {
    myIMU.enableLinearAccelerometer(2.5); 
  }

  // 4. Let IMU Settle
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (myIMU.dataAvailable()) { 
      prev_imu_time = micros(); 
    }
  }

  // 5. Capture Baseline Pressure & Temperature
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
    myIMU.enableLinearAccelerometer(2.5);
  }

  // 1. Read BNO085 IMU X+ Acceleration & Double Integrate
  if (myIMU.dataAvailable() == true) {
    unsigned long now_micros = micros();
    
    if (prev_imu_time == 0) prev_imu_time = now_micros;
    
    float dt = (now_micros - prev_imu_time) * 1e-6f;
    prev_imu_time = now_micros;

    float accel_x = myIMU.getLinAccelX(); 

    imu_vel += accel_x * dt;
    imu_alt += imu_vel * dt + 0.5f * accel_x * dt * dt;
    
    // Kalman Prediction (High Frequency)
    kalman.predict(accel_x, dt);

    // 2. Read DPS310 Barometer
    sensors_event_t temp_event, pressure_event;
    if (dps.getEvents(&temp_event, &pressure_event)) {
      
      float delta_temp = temp_event.temperature - baselineTemp;
      float comp_pressure = pressure_event.pressure + (delta_temp * tempCompCoef);
      
      baro_alt = 44330.0f * (1.0f - pow(comp_pressure / baselinePressure, 0.190295f));
      
      // Dynamic R penalty for high acceleration removed. 
      // A static R_z of 1.0f forces the filter to heavily favor the IMU during fast movements naturally.
      kalman.update(baro_alt);
    }

    // 3. Print Data to Serial Plotter
    Serial.print(0);
    Serial.print(",");
    Serial.print(0.2);
    Serial.print(",");
    Serial.print(imu_alt, 4);
    Serial.print(",");
    Serial.print(baro_alt, 4);
    Serial.print(",");
    Serial.print(temp_event.temperature, 2);
    Serial.print(",");
    Serial.println(kalman.x[0], 4);
  }
}
