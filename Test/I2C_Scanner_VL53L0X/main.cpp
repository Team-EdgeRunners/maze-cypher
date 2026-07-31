/*
  ================================================================================
  STM32 Blue Pill + CJVL53L0XV2 (VL53L0X) Distance Sensor
  I2C Scanner + Continuous Ranging with Live Mode Selection
  Library: Pololu VL53L0X
  Platform: PlatformIO (Arduino Framework for STM32)
  ================================================================================

  [platformio.ini]
  ----------------
  [env:i2c_vl53l0x_test]
  platform = ststm32
  board = bluepill_f103c8
  framework = arduino
  lib_deps =
      pololu/VL53L0X
  monitor_speed = 115200

  [Wiring]
  --------
  VCC   -> 3.3V
  GND   -> GND
  SDA   -> PB7 (I2C1_SDA)
  SCL   -> PB6 (I2C1_SCL)
  XSHUT -> tied HIGH (3.3V)
  GPIO1 -> optional
  ================================================================================
*/

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
bool vl53l0xInitialized = false;

bool scanI2CBus(bool lookForVL53L0X = true);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println("\n===========================================");
  Serial.println("  STM32 I2C Scanner + VL53L0X Live Modes   ");
  Serial.println("===========================================");

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(100000);

  bool foundLox = scanI2CBus(true);

  if (foundLox) {
    Serial.println("\nAttempting VL53L0X Initialization...");
    if (sensor.init()) {
      vl53l0xInitialized = true;
      sensor.setTimeout(500);

      // Default: long-range mode
      sensor.setMeasurementTimingBudget(200000); // 200 ms
      sensor.startContinuous();

      Serial.println("[SUCCESS] VL53L0X initialized in continuous mode!");
      Serial.println("Type 'f' for fast mode, 'l' for long-range mode, 's' for single-shot.");
    } else {
      Serial.println("[ERROR] VL53L0X detected but init failed!");
    }
  }
}

void loop() {
  // Check for user input
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'f') {
      sensor.setMeasurementTimingBudget(20000); // fast mode ~20 ms
      Serial.println("[MODE] Fast mode selected (short range, quick updates).");
    } else if (cmd == 'l') {
      sensor.setMeasurementTimingBudget(200000); // long mode ~200 ms
      Serial.println("[MODE] Long-range mode selected (slower, more reliable).");
    } else if (cmd == 's') {
      uint16_t distance = sensor.readRangeSingleMillimeters();
      if (sensor.timeoutOccurred() || distance == 65535) {
        Serial.println("[VL53L0X] Single-shot invalid measurement.");
      } else {
        Serial.print("[VL53L0X] Single-shot Distance: ");
        Serial.print(distance);
        Serial.println(" mm");
      }
    }
  }

  if (vl53l0xInitialized) {
    uint16_t distance = sensor.readRangeContinuousMillimeters();

    if (sensor.timeoutOccurred()) {
      Serial.println("[VL53L0X] Timeout occurred!");
    } else if (distance == 65535) {
      Serial.println("[VL53L0X] Invalid measurement (no target, too close, or out of range).");
    } else {
      Serial.print("[VL53L0X] Distance: ");
      Serial.print(distance);
      Serial.print(" mm  |  ");
      Serial.print(distance / 10.0f, 1);
      Serial.println(" cm");
    }
  } else {
    Serial.println("[VL53L0X] Sensor offline. Retrying init...");
    if (sensor.init()) {
      vl53l0xInitialized = true;
      sensor.setTimeout(500);
      sensor.setMeasurementTimingBudget(200000);
      sensor.startContinuous();
      Serial.println("[SUCCESS] VL53L0X re-initialized successfully!");
    }
  }

  delay(200);
}

bool scanI2CBus(bool lookForVL53L0X) {
  uint8_t error, address;
  int nDevices = 0;
  bool vl53l0xFound = false;

  Serial.println("Scanning I2C bus (0x01 - 0x7F)...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(" -> Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);

      if (address == 0x29) {
        Serial.print("  *** [VL53L0X TOF Sensor] ***");
        vl53l0xFound = true;
      }
      Serial.println();
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Scan finished. Devices found: ");
    Serial.println(nDevices);
  }

  return vl53l0xFound;
}
