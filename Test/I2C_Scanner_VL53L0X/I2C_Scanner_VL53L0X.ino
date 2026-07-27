/*
  ================================================================================
  COMBINED TEST: I2C Scanner + VL53L0X Distance Sensor
  PLATFORM: STM32 (e.g. STM32F103C8T6 Blue Pill)
  ENVIRONMENT: PlatformIO (VS Code Extension) / Arduino IDE
  ================================================================================

  [1] PLATFORMIO CONFIGURATION (platformio.ini):
  ---------------------------------------------
  Environment: [env:i2c_vl53l0x_test]
  Platform: ststm32
  Board: bluepill_f103c8
  Framework: arduino

  Libraries (Automatic Download via platformio.ini):
  - adafruit/Adafruit VL53L0X @ ^1.2.4

  [2] HARDWARE WIRING:
  --------------------
  +------------------+----------------------------------+
  | Sensor/Device    | STM32 Board Pin                  |
  +------------------+----------------------------------+
  | VCC              | 3.3V (or 5V if module has 3.3V LDO)|
  | GND              | GND                              |
  | SDA              | PB7 (I2C1_SDA)                   |
  | SCL              | PB6 (I2C1_SCL)                   |
  | XSHUT (Optional) | PA4 (Shutdown / Enable Control)  |
  +------------------+----------------------------------+
  * NOTE: Connect 4.7k ohm pull-up resistors from SDA -> 3.3V and SCL -> 3.3V
    if your TOF breakout board lacks built-in pull-ups.

  ================================================================================
*/

#include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <Wire.h>

// STM32 I2C1 Default Pins & Control
#define SDA_PIN PB7
#define SCL_PIN PB6
#define XSHUT_PIN PA4 // Optional hardware reset pin

// Create VL53L0X sensor instance
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Function prototypes
bool scanI2CBus(bool lookForVL53L0X = true);
void resetI2CBus();

bool vl53l0xInitialized = false;
unsigned long lastScanTime = 0;
const unsigned long RESCAN_INTERVAL = 15000; // Rescan I2C bus every 15 seconds

void setup() {
  // Initialize Serial output (115200 Baud)
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait up to 3 sec for Serial Monitor

  Serial.println(F("\n==========================================="));
  Serial.println(F("  STM32 I2C Scanner + VL53L0X Test Suite   "));
  Serial.println(F("==========================================="));

  // Step 1: Hardware reset of VL53L0X via XSHUT pin (PA4) if connected
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(20);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(50); // Allow 50ms for VL53L0X internal MCU to boot

  // Step 2: I2C Bus Recovery (Clear any stuck SDA low states)
  resetI2CBus();

  // Step 3: Configure STM32 I2C Pins
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  delay(50);

  // Step 4: Direct Sensor Initialization
  Serial.println(F("Attempting VL53L0X Sensor Initialization (Address 0x29)..."));
  if (lox.begin(0x29)) {
    vl53l0xInitialized = true;
    Serial.println(F("[SUCCESS] VL53L0X sensor initialized successfully!"));
    Serial.println(F("Streaming distance measurements...\n"));
  } else {
    Serial.println(F("[ERROR] VL53L0X initialization failed at 0x29!"));
    Serial.println(F("Troubleshooting Checklist:"));
    Serial.println(F("  1. Verify VCC -> 3.3V (or 5V if module has onboard 3.3V regulator)."));
    Serial.println(F("  2. Verify GND -> GND (Common ground with STM32)."));
    Serial.println(F("  3. Verify SDA -> PB7 and SCL -> PB6."));
    Serial.println(F("  4. If XSHUT is wired to PA4, try disconnecting PA4 or wiring XSHUT to 3.3V."));
    Serial.println(F("  5. Add 4.7k ohm pull-up resistors on SDA and SCL to 3.3V."));
  }

  // Step 5: Run initial I2C bus scan
  scanI2CBus(true);
}

void loop() {
  // Periodically perform full I2C bus scan
  if (millis() - lastScanTime >= RESCAN_INTERVAL) {
    lastScanTime = millis();
    Serial.println(F("\n--- Periodic I2C Bus Rescan ---"));
    scanI2CBus(false);
    Serial.println(F("-------------------------------\n"));
  }

  // Read distance measurement if VL53L0X is initialized
  if (vl53l0xInitialized) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // Pass 'true' for raw debug log output

    if (measure.RangeStatus != 4) { // Status 4 = out of range / phase error
      uint16_t dist_mm = measure.RangeMilliMeter;
      float dist_cm = dist_mm / 10.0f;

      Serial.print(F("[VL53L0X] Distance: "));
      Serial.print(dist_mm);
      Serial.print(F(" mm  |  "));
      Serial.print(dist_cm, 1);
      Serial.println(F(" cm"));
    } else {
      Serial.println(F("[VL53L0X] Out of range / Signal too weak"));
    }
  } else {
    Serial.println(F("[VL53L0X] Sensor offline. Retrying initialization..."));
    delay(1000);
    if (lox.begin(0x29)) {
      vl53l0xInitialized = true;
      Serial.println(F("[SUCCESS] VL53L0X re-initialized successfully!"));
    }
  }

  delay(500); // Sampling interval (2Hz)
}

// Clears I2C bus lockups by sending 9 SCL clock pulses
void resetI2CBus() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
  }
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
}

bool scanI2CBus(bool lookForVL53L0X) {
  uint8_t error, address;
  int nDevices = 0;
  bool vl53l0xFound = false;

  Serial.println(F("Scanning I2C bus (Addresses 0x01 - 0x7F)..."));

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F(" -> Device found at address 0x"));
      if (address < 16)
        Serial.print(F("0"));
      Serial.print(address, HEX);

      if (address == 0x29) {
        Serial.print(F("  *** [VL53L0X TOF Sensor] ***"));
        vl53l0xFound = true;
      } else if (address == 0x68 || address == 0x69) {
        Serial.print(F("  [IMU Sensor - e.g. MPU6050 / BMI160]"));
      } else if (address == 0x3C || address == 0x3D) {
        Serial.print(F("  [OLED Display]"));
      }
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.print(F(" -> Unknown error at address 0x"));
      if (address < 16)
        Serial.print(F("0"));
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println(F("No I2C devices found! Check SDA (PB7), SCL (PB6), 3.3V, "
                     "GND, & 4.7k pull-ups."));
  } else {
    Serial.print(F("Scan finished. Total devices found: "));
    Serial.println(nDevices);
  }

  return vl53l0xFound;
}
