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

void setup() {
  // Initialize Serial port for CSV output to Processing
  Serial.begin(115200); 
  // (Note: while(!Serial) is left out so the Teensy can still run headless without a PC)
  
  pinMode(BNO085_RST, OUTPUT);
  pinMode(BNO085_CS, OUTPUT);
  pinMode(BNO085_INT, INPUT_PULLUP); 

  // Configure Teensy native PWM for 50Hz and 16-bit resolution
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, 50); 
  analogWriteResolution(16); // Duty cycle range: 0 to 65535

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

    // Map the roll angle (-PI to PI) to a 1000 - 2000 microsecond pulse width
    // 0 radians (level) = 1500 us
    float mag_factor = 4;
    float offset = 0;
    float pulse_us = 1500.0 + (mag_factor* roll_rad / PI) * 500.0 + offset;
    
    // Constrain to strictly valid bounds for safety
    pulse_us = constrain(pulse_us, 0.0, 3000.0);

    // Convert microseconds to a 16-bit PWM duty cycle value
    // 50Hz frequency = 20,000 microseconds total period
    uint16_t duty_cycle = (uint16_t)((pulse_us / 20000.0) * 65535.0);
    //uint16_t duty_cycle = map(pulse_us, 1000, 2000, )
    
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
    //Serial.print(", roll: ");
    //Serial.println(pulse_us, 5);
  }
}
