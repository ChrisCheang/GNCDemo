#include <Servo.h>
#include <Wire.h>
#include <INA228.h>

// --- HARDWARE PIN DEFINITIONS ---
#define SERVO_PITCH_PIN 6  
#define SERVO_YAW_PIN   5  

#define ESC_TOP_PIN     2
#define ESC_BOT_PIN     3

// --- HX711 SERIAL PIN DEFINITIONS ---
#define HX711_DT_PIN    17  // Data pin
#define HX711_SCK_PIN   16  // Clock pin

// --- ACTUATOR OBJECTS & MIDPOINTS ---
Servo pitchServo;
Servo yawServo;
Servo escTop;
Servo escBot;

int servo_center_us_pitch = 1500; // Pitch servo midpoint (us)
int servo_center_us_yaw   = 1485; // Yaw servo midpoint (us)

const int PWM_MIN_ESC     = 1000; // Armed / idle ESC duty cycle (us)
const int PWM_MAX_ESC     = 1800; // User-defined maximum ESC duty cycle (us)

// --- MATEK INA228 POWER MONITOR SETUP ---
INA228 ina(0x45); // Matek default decimal address 69 is 0x45 in hex

// --- LOAD CELL & FILTER VARIABLES ---
float loadCellCalibrationFactor = 3500000.0; // Adjust based on your 5kg load cell calibration
long loadCellZeroOffset = 0;

float loadCellTimeConstant = 0.0f;       // User-adjustable time constant (seconds)
float filteredLoadCellKg   = 0.0f;       // Filtered load cell output
unsigned long lastLoopMicros = 0;

// --- TEST SEQUENCE STATE VARIABLES ---
unsigned long testStartMillis = 0;
bool calibrate_esc = false; // Defaulted to false so ESCs do NOT recalibrate by default

// Sensor telemetry variables
float battery_voltage = 0.0f;
float battery_current = 0.0f;
float battery_power   = 0.0f;
float loadCellKg      = 0.0f;
int currentEscPwm     = PWM_MIN_ESC;

// --- SENSOR HELPER FUNCTIONS ---

void initLoadCell() {
  pinMode(HX711_DT_PIN, INPUT);
  pinMode(HX711_SCK_PIN, OUTPUT);
  digitalWrite(HX711_SCK_PIN, LOW);
  
  // Tare / zero offset calibration reading upon startup
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += readHX711Raw();
    delay(10);
  }
  loadCellZeroOffset = sum / 10;
  
  // Initialize filter state
  long rawVal = readHX711Raw();
  filteredLoadCellKg = (float)(rawVal - loadCellZeroOffset) / loadCellCalibrationFactor;
  lastLoopMicros = micros();
  
  Serial.println(F("[INFO] HX711 Load Cell initialized, zeroed, and filtered."));
}

long readHX711Raw() {
  // Wait for DT to go low (read ready)
  unsigned long startWait = millis();
  while (digitalRead(HX711_DT_PIN) == HIGH) {
    if (millis() - startWait > 100) return 0; // Timeout safeguard
  }

  unsigned long value = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(HX711_SCK_PIN, HIGH);
    delayMicroseconds(1);
    value = (value << 1) | digitalRead(HX711_DT_PIN);
    digitalWrite(HX711_SCK_PIN, LOW);
    delayMicroseconds(1);
  }

  // 25th pulse for Channel A, Gain 128
  digitalWrite(HX711_SCK_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(HX711_SCK_PIN, LOW);
  delayMicroseconds(1);

  // Sign extension for 24-bit two's complement value
  if (value & 0x800000) {
    value |= 0xFF000000;
  }
  return (long)value;
}

void readLoadCell(float &kg) {
  unsigned long now = micros();
  float dt = (now - lastLoopMicros) * 1e-6f;
  lastLoopMicros = now;
  
  if (dt <= 0.0f || dt > 0.1f) dt = 0.02f; // Failsafe fallback delta-t (~50Hz)

  long rawVal = readHX711Raw();
  float instantKg = (float)(rawVal - loadCellZeroOffset) / loadCellCalibrationFactor;

  // First-order low-pass filter calculation
  float alpha = dt / (loadCellTimeConstant + dt);
  filteredLoadCellKg = filteredLoadCellKg + alpha * (instantKg - filteredLoadCellKg);

  kg = filteredLoadCellKg;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for serial monitor connection

  // Initialize Wire for INA228
  Wire.begin();
  if (!ina.begin()) {
    Serial.println("WARNING: Matek I2C-INA-BM not detected!");
  } else {
    ina.setMaxCurrentShunt(204.8f, 0.0002f); // Max current 204.8A, 200 uOhm shunt
  }

  // 1. Setup TVC Servos and lock them at their respective midpoints
  pitchServo.attach(SERVO_PITCH_PIN);
  yawServo.attach(SERVO_YAW_PIN);
  pitchServo.writeMicroseconds(servo_center_us_pitch);
  yawServo.writeMicroseconds(servo_center_us_yaw);

  // 2. Setup ESCs & Calibration / Bypass Option
  escTop.attach(ESC_TOP_PIN);
  escBot.attach(ESC_BOT_PIN);

  Serial.println(F("[INFO] Motor Characterization Initialized."));
  Serial.println(F("[INFO] Send any character within 3 seconds to BYPASS ESC calibration..."));
  
  escTop.writeMicroseconds(PWM_MIN_ESC);
  escBot.writeMicroseconds(PWM_MIN_ESC);

  unsigned long calStart = millis();
  bool bypassCalibration = false;
  while (millis() - calStart < 3000) {
    if (Serial.available()) {
      Serial.read(); // Clear buffer
      bypassCalibration = true;
      Serial.println(F("[INFO] ESC Calibration Bypassed by user."));
      break;
    }
  }

  if (!bypassCalibration && calibrate_esc) {
    Serial.println(F("[INFO] Entering standard ESC calibration sequence..."));
    escTop.writeMicroseconds(2000);
    escBot.writeMicroseconds(2000);
    delay(5000);
    escTop.writeMicroseconds(1000);
    escBot.writeMicroseconds(1000);
    delay(3000);
    Serial.println(F("[INFO] ESC Calibration Complete."));
  } else {
    delay(4000); // Standard initialization delay
  }

  initLoadCell();

  Serial.println(F("[INFO] Starting test in 2 seconds..."));
  delay(2000);

  // Initialize millisecond timer at start of test loop
  testStartMillis = millis();
  Serial.println(F("Time(s),Voltage(V),Current(A),Power(W),ESC_PWM(us),LoadCell(kg)"));
}

void loop() {
  // Millisecond timer initialized at start of loop
  unsigned long currentMillis = millis();
  float t = (currentMillis - testStartMillis) / 1000.0f;

  // --- Throttle Profile Logic ---
  if (t <= 2.0f) {
    currentEscPwm = PWM_MIN_ESC;
  } else {
    float activeTime = t - 2.0f;
    int rampUpSteps = (PWM_MAX_ESC - PWM_MIN_ESC) / 100; 
    
    if (activeTime < rampUpSteps) {
      int step = (int)activeTime;
      currentEscPwm = PWM_MIN_ESC + (step * 100);
      if (currentEscPwm > PWM_MAX_ESC) currentEscPwm = PWM_MAX_ESC;
    } else {
      float rampDownTime = activeTime - rampUpSteps;
      int downStep = (int)rampDownTime;
      currentEscPwm = PWM_MAX_ESC - (downStep * 200);
      if (currentEscPwm < PWM_MIN_ESC) {
        currentEscPwm = PWM_MIN_ESC;
      }
    }
  }

  // Apply PWM signal to ESCs
  escTop.writeMicroseconds(currentEscPwm);
  escBot.writeMicroseconds(currentEscPwm);

  // Keep servos strictly locked at their respective midpoints for test duration
  pitchServo.writeMicroseconds(servo_center_us_pitch);
  yawServo.writeMicroseconds(servo_center_us_yaw);

  // --- Read Sensors ---
  battery_voltage = ina.getBusVoltage();
  battery_current = ina.getCurrent();
  battery_power   = ina.getPower();
  readLoadCell(loadCellKg);

  // --- Telemetry Serial Logging ---
  Serial.print(t, 3); Serial.print(",");
  Serial.print(battery_voltage, 2); Serial.print(",");
  Serial.print(battery_current, 2); Serial.print(",");
  Serial.print(battery_power, 2); Serial.print(",");
  Serial.print(currentEscPwm); Serial.print(",");
  Serial.println(loadCellKg, 4);

  // 50 Hz control/telemetry loop rate
  delay(20); 
}
