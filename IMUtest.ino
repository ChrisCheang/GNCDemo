#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h" 

#define BNO085_CS   10
#define BNO085_INT  9
#define BNO085_RST  8

BNO080 myIMU;

// Global variables to store the correction quaternion (a pure Z-axis rotation)
float qOffsetReal = 1.0;
float qOffsetK = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); 
  }
  
  pinMode(BNO085_RST, OUTPUT);
  pinMode(BNO085_CS, OUTPUT);
  pinMode(BNO085_INT, INPUT_PULLUP); 

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
  
  // Allow 2 seconds for the magnetometer to settle and pull valid data
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (myIMU.dataAvailable()) {
      myIMU.getQuatReal(); // Flush the buffer during the settling time
    }
  }

  // 2. Capture the Absolute 9-DOF Yaw
  while (!myIMU.dataAvailable()) { delay(1); } // Wait for a fresh packet
  float qI_9 = myIMU.getQuatI();
  float qJ_9 = myIMU.getQuatJ();
  float qK_9 = myIMU.getQuatK();
  float qR_9 = myIMU.getQuatReal();
  
  // Convert 9-DOF quaternion to Euler Yaw (Z-axis rotation)
  float yaw9 = atan2(2.0 * (qR_9 * qK_9 + qI_9 * qJ_9), 1.0 - 2.0 * (qJ_9 * qJ_9 + qK_9 * qK_9));

  // 3. Disable 9-DOF and switch to 6-DOF Game Rotation Vector
  myIMU.enableRotationVector(0); // Send interval 0 to disable
  delay(50);
  
  // Flush any leftover 9-DOF packets sitting in the pipeline
  while (myIMU.dataAvailable()) { myIMU.getQuatReal(); } 
  
  myIMU.enableGameRotationVector(2.5); // Enable 6-DOF (Magnetometer Off)
  delay(50);

  // 4. Capture the initial arbitrary 6-DOF Yaw
  while (!myIMU.dataAvailable()) { delay(1); } // Wait for first 6-DOF packet
  float qI_6 = myIMU.getQuatI();
  float qJ_6 = myIMU.getQuatJ();
  float qK_6 = myIMU.getQuatK();
  float qR_6 = myIMU.getQuatReal();
  
  float yaw6 = atan2(2.0 * (qR_6 * qK_6 + qI_6 * qJ_6), 1.0 - 2.0 * (qJ_6 * qJ_6 + qK_6 * qK_6));

  // 5. Calculate the offset and precompute the correction quaternion components
  float yawOffset = yaw9 - yaw6;
  qOffsetReal = cos(yawOffset / 2.0);
  qOffsetK = sin(yawOffset / 2.0);
  
  // ==========================================================
  // --- END CALIBRATION ---
  // ==========================================================
}

void loop() {
  if (myIMU.dataAvailable() == true) {
    
    // Fetch the 6-DOF Game Rotation Vector (No Magnetometer Drift)
    float qI = myIMU.getQuatI();
    float qJ = myIMU.getQuatJ();
    float qK = myIMU.getQuatK();
    float qR = myIMU.getQuatReal();

    // Apply the precomputed Z-axis yaw offset via quaternion multiplication.
    // This shifts the arbitrary Game Vector to align with Earth's absolute North.
    float corr_qR = (qOffsetReal * qR) - (qOffsetK * qK);
    float corr_qI = (qOffsetReal * qI) - (qOffsetK * qJ);
    float corr_qJ = (qOffsetReal * qJ) + (qOffsetK * qI);
    float corr_qK = (qOffsetReal * qK) + (qOffsetK * qR);

    // Output corrected raw CSV data with high precision to prevent data loss
    Serial.print(corr_qI, 7);
    Serial.print(",");
    Serial.print(corr_qJ, 7);
    Serial.print(",");
    Serial.print(corr_qK, 7);
    Serial.print(",");
    Serial.println(corr_qR, 7);
  }
}
