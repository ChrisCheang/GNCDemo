#include <SPI.h>
#include "SparkFun_BNO080_Arduino_Library.h" 

#define BNO085_CS   10
#define BNO085_INT  9
#define BNO085_RST  8

BNO080 myIMU;

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

  // Enable rotation vector at 400Hz (2.5ms)
  myIMU.enableRotationVector(2.5); 
}

void loop() {
  if (myIMU.dataAvailable() == true) {
    
    float qI = myIMU.getQuatI();
    float qJ = myIMU.getQuatJ();
    float qK = myIMU.getQuatK();
    float qReal = myIMU.getQuatReal();

    // Output raw CSV data with high precision to prevent data loss
    Serial.print(qI, 7);
    Serial.print(",");
    Serial.print(qJ, 7);
    Serial.print(",");
    Serial.print(qK, 7);
    Serial.print(",");
    Serial.println(qReal, 7);
  }
}
