#define GNSS_SERIAL Serial1  // Hardware Serial connected to LC29H (Pins 0/1)
#define LC29H_BAUD 460800    // Matched to module's active operating speed
#define PC_BAUD 115200       // USB Serial speed for Python script

void setup() {
  Serial.begin(PC_BAUD);      
  GNSS_SERIAL.begin(LC29H_BAUD); 
}

void loop() {
  // 1. Forward incoming RTCM3 bytes from Python script (USB) -> LC29H (Serial1)
  while (Serial.available()) {
    GNSS_SERIAL.write(Serial.read());
  }

  // 2. Read NMEA stream from LC29H (Serial1), filter for GNGGA, and print to USB Serial
  static char buffer[128];
  static uint8_t idx = 0;

  while (GNSS_SERIAL.available()) {
    char c = GNSS_SERIAL.read();
    
    // Optional Diagnostic: Uncomment the line below to see ALL raw bytes if troubleshooting
    // Serial.write(c); 

    if (c == '\n') {
      buffer[idx] = '\0';
      
      // Filter and print valid GNGGA / GPGGA fix strings
      if (strncmp(buffer, "$GNGGA", 6) == 0 || strncmp(buffer, "$GPGGA", 6) == 0) {
        Serial.println(buffer); 
      }
      
      idx = 0;
    } else if (c != '\r' && idx < sizeof(buffer) - 1) {
      buffer[idx++] = c;
    }
  }
}
