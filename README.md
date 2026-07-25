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
    │   └── main.cpp
    ├── BMI160/                 # BMI160 6-DOF IMU (Gyro + Accelerometer) Test
    │   └── main.cpp
    ├── OLED_SPI/               # 0.96" 128x64 Bicolor SPI OLED Display Test
    │   └── main.cpp
    ├── L298N_Motors/           # L298N Motor Driver + Dual N20 Hall Encoder Test
    │   └── main.cpp
    └── I2C_Scanner/            # I2C Bus Address Scanner Utility
        └── main.cpp
```

---

## ⚡ Master Hardware Pinout Reference (STM32F103C8T6)

All hardware modules in this repository are assigned dedicated STM32 pins without pin conflicts:

| Module | Module Pin | STM32 Pin | Function / Protocol |
| :--- | :--- | :--- | :--- |
| **VL53L0XV2 TOF** | SDA | **PB7** | I2C1_SDA (Default Addr: `0x29`) |
| | SCL | **PB6** | I2C1_SCL |
| | XSHUT | **PA4** | Optional Reset Pin |
| | VCC / GND | 3.3V / GND | Power |
| **BMI160 IMU** | SDA | **PB7** | I2C1_SDA (Default Addr: `0x68`) |
| | SCL | **PB6** | I2C1_SCL |
| | SA0 / CS | GND / 3.3V | Address Config / I2C Mode Select |
| | VCC / GND | 3.3V / GND | Power |
| **0.96" SPI OLED** | D0 / CLK | **PA5** | SPI1_SCK |
| | D1 / DIN | **PA7** | SPI1_MOSI |
| | RES / RST | **PA2** | OLED Reset |
| | DC / D/C | **PA3** | Data / Command Control |
| | CS | **PA4** | Chip Select |
| | VCC / GND | 3.3V / GND | Power |
| **L298N Driver** | ENA | **PA8** | Left Motor PWM (TIM1_CH1) |
| | IN1 / IN2 | **PB12 / PB13** | Left Motor Direction |
| | ENB | **PA11** | Right Motor PWM (TIM1_CH4) |
| | IN3 / IN4 | **PB14 / PB15** | Right Motor Direction |
| | VMS / GND | Batt / GND | Motor Power & Common GND |
| **N20 Encoders** | Motor A Out A/B | **PB0 / PB1** | Left Encoder Interrupt (EXTI0) / Phase B |
| | Motor B Out A/B | **PB10 / PB11** | Right Encoder Interrupt (EXTI10) / Phase B |
| | VCC / GND | 3.3V / GND | Encoder Logic Power |

---

## 🛠️ PlatformIO Environments

The project includes pre-configured environments inside [`platformio.ini`](platformio.ini):

| Environment Name | Description | Source File | Automatic Dependencies |
| :--- | :--- | :--- | :--- |
| `[env:vl53l0x_test]` | VL53L0XV2 Distance Sensor Test | `Test/VL53L0X/main.cpp` | `adafruit/Adafruit VL53L0X` |
| `[env:bmi160_test]` | BMI160 6-Axis Gyro + Accel Test | `Test/BMI160/main.cpp` | `emotitron/BMI160-Arduino` |
| `[env:oled_spi_test]` | 0.96" SPI Yellow-Blue OLED Test | `Test/OLED_SPI/main.cpp` | `Adafruit SSD1306`, `Adafruit GFX` |
| `[env:l298n_motors_test]`| L298N + 2x N20 Encoder Motors Test | `Test/L298N_Motors/main.cpp`| *None* |
| `[env:i2c_scanner]` | I2C Device Address Scanner | `Test/I2C_Scanner/main.cpp`| *None* |

---

## 🚀 Getting Started with PlatformIO

### Prerequisites
1. **VS Code** with the **PlatformIO IDE** extension installed.
2. **ST-Link V2 Programmer** (or USB Serial / DFU) connected to your STM32 board.

### Building & Flashing

1. Clone or open the repository in VS Code:
   ```bash
   code .
   ```
2. Open the **PlatformIO** extension tab on the left sidebar.
3. Under **PROJECT TASKS**, expand the test environment you wish to run (e.g. `env:vl53l0x_test` or `env:l298n_motors_test`).
4. Click **Build** to compile the code.
5. Click **Upload** to flash the binary to your STM32 board via ST-Link.
6. Click **Serial Monitor** at `115200` baud to view real-time debug output.

---

## 📝 License

This project is licensed under the MIT License - feel free to use and adapt for your robotics projects!
