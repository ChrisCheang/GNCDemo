#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h" 

#define BNO085_CS   10
#define BNO085_INT  9
#define BNO085_RST  8

// Define the PWM output pin (Pin 2 is PWM capable on Teensy 4.1)
#define PWM_PIN     2 

BNO080 myIMU;

// Global variables to store the correction quaternion (a pure Z-axis rotation)
float qOffsetReal = 1.0;
float qOffsetK = 0.0;

float pwmFreq = 50.0;
float pwmMin = 500;
float pwmMax = 2500;
float pwmMid = 1500;


void setup() {
  // Initialize Serial port for CSV output to Processing
  Serial.begin(115200); 
  // (Note: while(!Serial) is left out so the Teensy can still run headless without a PC)
  
  pinMode(BNO085_RST, OUTPUT);
  pinMode(BNO085_CS, OUTPUT);
  pinMode(BNO085_INT, INPUT_PULLUP); 

  // Configure Teensy native PWM for 333Hz (PTK 8810 SSG-D spec) and 16-bit resolution
  pinMode(PWM_PIN, OUTPUT);
  // Set Teensy 4.1 Pin 2 (EMC_04 / PAD control) to Maximum Drive Strength (24mA)
  *(portConfigRegister(PWM_PIN)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3);
  analogWriteFrequency(PWM_PIN, pwmFreq); // 50 for micro, 333 for 8810
  analogWriteResolution(16); // Duty cycle range: 0 to 65535

  // ==========================================================
  // --- PWM KICKSTART ---
  // Send a valid 1520us neutral pulse immediately to prevent 
  // the PTK 8810 from entering a signal-loss lockout state 
  // during the 2+ second IMU calibration delay.
  // ==========================================================
  uint16_t neutral_duty = (uint16_t)((1500.0 / (1000000.0 / pwmFreq)) * 65535.0);
  analogWrite(PWM_PIN, neutral_duty);

  SPI.begin();

  if (myIMU.beginSPI(BNO085_CS, 255, BNO085_INT, BNO085_RST, 3000000) == false) {
    Serial.println("ERROR: BNO085 not detected.");
    while (1); 
  }

  // ==========================================================
  // --- BEGIN STARTUP HANDOVER CALIBRATION ---
  // ==========================================================
  
  // 1. Enable 9-DOF Rotation Vector to find absolute Earth North
  myIMU.enableRotationVector(2.5); 
  
  // Allow 2 seconds for the magnetometer to settle
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (myIMU.dataAvailable()) {
      myIMU.getQuatReal(); 
    }
  }

  // 2. Capture the Absolute 9-DOF Yaw
  while (!myIMU.dataAvailable()) { delay(1); } 
  float qI_9 = myIMU.getQuatI();
  float qJ_9 = myIMU.getQuatJ();
  float qK_9 = myIMU.getQuatK();
  float qR_9 = myIMU.getQuatReal();
  
  float yaw9 = atan2(2.0 * (qR_9 * qK_9 + qI_9 * qJ_9), 1.0 - 2.0 * (qJ_9 * qJ_9 + qK_9 * qK_9));

  // 3. Disable 9-DOF and switch to 6-DOF Game Rotation Vector
  myIMU.enableRotationVector(0); 
  delay(50);
  
  while (myIMU.dataAvailable()) { myIMU.getQuatReal(); } 
  
  myIMU.enableGameRotationVector(2.5); 
  delay(50);

  // 4. Capture the initial arbitrary 6-DOF Yaw
  while (!myIMU.dataAvailable()) { delay(1); } 
  float qI_6 = myIMU.getQuatI();
  float qJ_6 = myIMU.getQuatJ();
  float qK_6 = myIMU.getQuatK();
  float qR_6 = myIMU.getQuatReal();
  
  float yaw6 = atan2(2.0 * (qR_6 * qK_6 + qI_6 * qJ_6), 1.0 - 2.0 * (qJ_6 * qJ_6 + qK_6 * qK_6));

  // 5. Calculate the offset and precompute the correction quaternion
  float yawOffset = yaw9 - yaw6;
  qOffsetReal = cos(yawOffset / 2.0);
  qOffsetK = sin(yawOffset / 2.0);
  
  // ==========================================================
  // --- END CALIBRATION ---
  // ==========================================================
}

void loop() {
  if (myIMU.dataAvailable() == true) {
    
    // Fetch the 6-DOF Game Rotation Vector
    float qI = myIMU.getQuatI();
    float qJ = myIMU.getQuatJ();
    float qK = myIMU.getQuatK();
    float qR = myIMU.getQuatReal();

    // Apply Z-axis yaw offset
    float corr_qR = (qOffsetReal * qR) - (qOffsetK * qK);
    float corr_qI = (qOffsetReal * qI) - (qOffsetK * qJ);
    float corr_qJ = (qOffsetReal * qJ) + (qOffsetK * qI);
    float corr_qK = (qOffsetReal * qK) + (qOffsetK * qR);

    // ==========================================================
    // --- 1. PWM HARDWARE OUTPUT ---
    // ==========================================================
    // Calculate Roll Euler Angle (ZYX convention, X-axis rotation) in radians
    float roll_rad = atan2(2.0 * (corr_qR * corr_qI + corr_qJ * corr_qK), 
                           1.0 - 2.0 * (corr_qI * corr_qI + corr_qJ * corr_qJ));

    // Map roll angle to pulse width with 1520us neutral position
    float mag_factor = 4;
    float offset = 0;
    float pulse_us = pwmMid + (mag_factor * roll_rad / PI) * 1000.0 + offset;
    
    // Constrain to strictly valid bounds for PTK 8810 SSG-D spec (900us - 2100us)
    pulse_us = constrain(pulse_us, pwmMin, pwmMax);

    // Convert microseconds to a 16-bit PWM duty cycle value for 333Hz
    // 333Hz frequency = ~3003.003 microseconds total period (1,000,000 / 333)
    uint16_t duty_cycle = (uint16_t)((pulse_us / (1000000.0 / pwmFreq)) * 65535.0);  // 333 for 8810, 50 for micro
    
    // Output the PWM signal
    analogWrite(PWM_PIN, duty_cycle);

    // ==========================================================
    // --- 2. SERIAL CSV OUTPUT ---
    // ==========================================================
    // Output corrected raw CSV data with high precision to Processing 3D Vis
    Serial.print(corr_qI, 7);
    Serial.print(",");
    Serial.print(corr_qJ, 7);
    Serial.print(",");
    Serial.print(corr_qK, 7);
    Serial.print(",");
    Serial.println(corr_qR, 7);
  }
}
