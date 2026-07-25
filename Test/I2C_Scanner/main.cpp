/*
  ================================================================================
  UTILITY: I2C Bus Scanner for STM32
  ENVIRONMENT: PlatformIO (VS Code Extension)
  ================================================================================
  
  [1] PLATFORMIO CONFIGURATION (platformio.ini):
  ---------------------------------------------
  Environment: [env:i2c_scanner]
  Platform: ststm32
  Board: bluepill_f103c8 (Or update platformio.ini to match your STM32 model)
  Framework: arduino

  [2] HARDWARE WIRING:
  --------------------
  +------------------+----------------------------------+
  | I2C Pin          | STM32 Board Pin                  |
  +------------------+----------------------------------+
  | SDA              | PB7 (I2C1_SDA)                   |
  | SCL              | PB6 (I2C1_SCL)                   |
  | VCC              | 3.3V                             |
  | GND              | GND                              |
  +------------------+----------------------------------+

  ================================================================================
*/

#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN PB7
#define SCL_PIN PB6

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  Serial.println(F("\n==========================================="));
  Serial.println(F("    STM32 PlatformIO I2C Bus Scanner       "));
  Serial.println(F("==========================================="));
}

void loop() {
  uint8_t error, address;
  int nDevices = 0;

  Serial.println(F("Scanning I2C bus..."));

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.println(F("  !"));
      nDevices++;
    } else if (error == 4) {
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println(F("No I2C devices found. Check SDA/SCL wiring and 4.7k pull-up resistors!"));
  } else {
    Serial.println(F("Scan complete."));
  }

  delay(5000); // Rescan every 5 seconds
}
