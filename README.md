# Maze Cypher - STM32 Robotics & Maze-Solving Platform

A modular STM32-based robotics codebase designed for maze solving (Micro-Mouse) and autonomous navigation. Powered by **PlatformIO** and the **Arduino Framework for STM32** (`ststm32`).

---

## 📁 Repository Structure

```text
maze-cypher/
├── platformio.ini              # Multi-environment PlatformIO build configuration
├── .gitignore                  # Git ignore rules for PlatformIO and STM32 builds
├── README.md                   # Project documentation & wiring reference
└── Test/                       # Modular hardware test sketches
    ├── VL53L0X/                # VL53L0XV2 TOF Distance Sensor Test
    │   └── VL53L0X.ino
    ├── BMI160/                 # BMI160 6-DOF IMU (Gyro + Accelerometer) Test
    │   └── BMI160.ino
    ├── OLED_SPI/               # 0.96" 128x64 Bicolor SPI OLED Display Test
    │   └── OLED_SPI.ino
    ├── L298N_Motors/           # L298N Motor Driver + Dual N20 Hall Encoder Test
    │   └── L298N_Motors.ino
    ├── I2C_Scanner/            # I2C Bus Address Scanner Utility
    │   └── I2C_Scanner.ino
    ├── I2C_Scanner_VL53L0X/    # I2C Scanner & VL53L0X TOF Sensor Test
    │   └── I2C_Scanner_VL53L0X.ino
    └── BLUEPILL_TEST/          # STM32 Blue Pill Blink Test
        └── BLUEPILL_TEST.ino
```

---

## ⚡ Master Hardware Pinout Reference (STM32F103C8T6)

| Module | Module Pin | STM32 Pin | Function / Protocol |
| :--- | :--- | :--- | :--- |
| **VL53L0XV2 TOF** | SDA | **PB7** | I2C1_SDA (Addr: `0x29`) |
| | SCL | **PB6** | I2C1_SCL |
| | XSHUT | **PA4** | Optional Reset |
| | VCC/GND | 3.3V/GND | Power |
| **BMI160 IMU** | SDA | **PB7** | I2C1_SDA (Addr: `0x68`) |
| | SCL | **PB6** | I2C1_SCL |
| | SA0/CS | GND/3.3V | Addr Config / I2C Mode |
| | VCC/GND | 3.3V/GND | Power |
| **0.96" SPI OLED** | D0/CLK | **PA5** | SPI1_SCK |
| | D1/DIN | **PA7** | SPI1_MOSI |
| | RES/RST | **PA2** | Reset |
| | DC/D/C | **PA3** | Data/Command |
| | CS | **PA4** | Chip Select |
| | VCC/GND | 3.3V/GND | Power |
| **L298N Driver** | ENA | **PA8** | Left Motor PWM |
| | IN1/IN2 | **PB12/PB13** | Left Motor Dir |
| | ENB | **PA11** | Right Motor PWM |
| | IN3/IN4 | **PB14/PB15** | Right Motor Dir |
| | VMS/GND | Batt/GND | Motor Power |
| **N20 Encoders** | A Out A/B | **PB0/PB1** | Left Encoder |
| | B Out A/B | **PB10/PB11** | Right Encoder |
| | VCC/GND | 3.3V/GND | Power |

---

## 🛠️ PlatformIO Environments

| Environment Name | Description | Source File | Dependencies |
| :--- | :--- | :--- | :--- |
| `[env:vl53l0x_test]` | VL53L0XV2 Distance Sensor Test | `Test/VL53L0X/VL53L0X.ino` | `adafruit/Adafruit VL53L0X` |
| `[env:bmi160_test]` | BMI160 Gyro + Accel Test | `Test/BMI160/BMI160.ino` | `emotitron/BMI160-Arduino` |
| `[env:oled_spi_test]` | SPI OLED Display Test | `Test/OLED_SPI/OLED_SPI.ino` | `Adafruit SSD1306`, `Adafruit GFX` |
| `[env:l298n_motors_test]` | L298N Motor Driver Test | `Test/L298N_Motors/L298N_Motors.ino` | *None* |
| `[env:i2c_scanner]` | I2C Scanner Utility | `Test/I2C_Scanner/I2C_Scanner.ino` | *None* |
| `[env:bluepill_blink]` | STM32 Blue Pill Test, LED Blink | `Test/BLUEPILL_TEST/BLUEPILL_TEST.ino` | *None* |

---

## 🚀 Getting Started with PlatformIO

### Prerequisites
1. **VS Code** with the **PlatformIO IDE** extension.
2. Programmer options:
   - **ST-Link V2** (`upload_protocol = stlink`)
   - **USB-to-TTL Adapter** (`upload_protocol = serial`, `upload_port = COM17`)

### Uploading with ST-Link
- Connect SWDIO=PA13, SWCLK=PA14, GND, 3.3V.
- Select environment → **Upload**.

### Uploading with USB-to-TTL
- Wiring: TX→PA10, RX→PA9, GND→GND, VCC→3.3V.
- Set BOOT0=1, BOOT1=0, press Reset.
- Select environment → **Upload**.
- After upload: set BOOT0=0, press Reset to run program.

### Build & Flash
```bash
pio run -t upload -e vl53l0x_test
```
Use **Serial Monitor** at `115200` baud for debug output.

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details. Feel free to use and adapt for your robotics projects!
