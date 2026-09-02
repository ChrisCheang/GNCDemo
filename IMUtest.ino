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

void setup() {
  Serial.begin(115200);
  
  // Wait up to 3 seconds for the Serial Monitor window to open
  unsigned long serialStartTime = millis();
  while (!Serial && (millis() - serialStartTime < 3000)) {
    delay(10);
  }
  
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

  uint16_t neutral_duty = (uint16_t)((1500.0 / (1000000.0 / pwmFreq)) * 65535.0);
  analogWrite(PWM_PIN_ROLL, neutral_duty);
  analogWrite(PWM_PIN_PITCH, neutral_duty);

  SPI.begin();
  Serial.println("SPI initialized.");

  // Initialize DPS310 over Hardware SPI (Non-blocking check)
  if (!dps.begin_SPI(DPS_CS)) {
    Serial.println("WARNING: DPS310 not detected! Check wiring.");
  } else {
    Serial.println("DPS310 detected.");
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  }

  // Initialize BNO085 (Non-blocking check)
  if (myIMU.beginSPI(BNO085_CS, 255, BNO085_INT, BNO085_RST, 3000000) == false) {
    Serial.println("WARNING: BNO085 not detected! Check SPI/CS/INT/RST wiring.");
  } else {
    Serial.println("BNO085 detected.");
    myIMU.enableRotationVector(2.5); 
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (myIMU.dataAvailable()) { myIMU.getQuatReal(); }
  }

  // Attempt baseline pressure capture with timeout
  unsigned long pressureTimeout = millis();
  sensors_event_t temp_event, pressure_event;
  while (!dps.getEvents(&temp_event, &pressure_event)) {
    if (millis() - pressureTimeout > 3000) {
      Serial.println("WARNING: DPS310 baseline pressure timeout.");
      pressure_event.pressure = 101325.0; // Fallback standard pressure
      break;
    }
    delay(10);
  }
  baselinePressure = pressure_event.pressure;
  Serial.println("Setup complete. Entering loop...");
}

void loop() {
  if (myIMU.dataAvailable() == true) {
    float qI = myIMU.getQuatI();
    float qJ = myIMU.getQuatJ();
    float qK = myIMU.getQuatK();
    float qR = myIMU.getQuatReal();

    float corr_qR = (qOffsetReal * qR) - (qOffsetK * qK);
    float corr_qI = (qOffsetReal * qI) - (qOffsetK * qJ);
    float corr_qJ = (qOffsetReal * qJ) + (qOffsetK * qI);
    float corr_qK = (qOffsetReal * qK) + (qOffsetK * qR);

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
    int pulse_us_esc = map(rawAnalog, 0, 1023, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    if (pulse_us_esc < ESC_DEADBAND) {
      pulse_us_esc = ESC_MIN_PULSE;
    }
    
    esc.writeMicroseconds(pulse_us_esc);

    sensors_event_t temp_event, pressure_event;
    if (dps.getEvents(&temp_event, &pressure_event)) {
      float alt_change_m = 44330.0 * (1.0 - pow(pressure_event.pressure / baselinePressure, 0.190295));

      Serial.print(corr_qI, 7);
      Serial.print(",");
      Serial.print(corr_qJ, 7);
      Serial.print(",");
      Serial.print(corr_qK, 7);
      Serial.print(",");
      Serial.print(corr_qR, 7);
      Serial.print(",");
      Serial.print(alt_change_m, 7);
      Serial.print(",");
      Serial.println(pulse_us_esc);
    }
  }
}
