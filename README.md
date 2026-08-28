
# Pi Pico 6-DoF Sensor Array & N20 Motor Controller

This project integrates a Raspberry Pi Pico with a suite of spatial sensors (Time-of-Flight, IMU, IR Obstacle) and an encoder-equipped N20 gear motor. It includes two main components:

1.  **C++ (Arduino IDE) Firmware:** Handles hardware initialization, I2C sensor reading, motor PWM safety limits, and hardware interrupts for precise distance tracking.
    
2.  **Python 3D Visualizer:** Reads the live serial CSV stream to render a real-time 3D model of the robot's orientation (Pitch, Roll, Yaw) and sensor readings using `vpython`.
    

## 🛠️ Hardware Bill of Materials (BOM)

-   **Microcontroller:** Raspberry Pi Pico (RP2040)
    
-   **Motors:** GA12-N20 6V Gear Motor (300 RPM, 1:100 Ratio, 7 PPR Encoder)
    
-   **Motor Driver:** L298N Module (with 1N5819 Schottky Flyback Diodes)
    
-   **IMU:** BMI160 6-DoF (Accelerometer + Gyroscope)
    
-   **Distance Sensors:** 3x VL53L0X Time-of-Flight (ToF) Lasers
    
-   **Obstacle Sensor:** Custom IR Bumper (LM358P Op-Amp, 10K Potentiometer, IR LED, Photodiode)
    
-   **Power Supply:** 11.7V 3S LiPo Battery
    
-   **Voltage Regulator:** L7805CV (Drops 11.7V to 5V for Logic)
    

> **⚠️ SAFETY WARNING:** The N20 motors are rated for **6V**, but the main power rail is **11.7V**. The Pico firmware strictly limits the motor's PWM output to `128` (50%) to prevent burning out the motor coils.

## 🔌 Wiring & Pinout Guide

### I2C Sensors (VL53L0X & BMI160)

**Sensor Pin**

**Pico Pin**

**Notes**

SDA

**GP4**

Shared I2C Bus

SCL

**GP5**

Shared I2C Bus

ToF 1 XSHUT

**GP10**

For assigning unique I2C addresses

ToF 2 XSHUT

**GP11**

For assigning unique I2C addresses

ToF 3 XSHUT

**GP12**

For assigning unique I2C addresses

### L298N Motor Driver

**L298N Pin**

**Pico Pin**

**Notes**

IN1

**GP19**

Motor Direction

IN2

**GP18**

Motor Direction

EN A

**GP21**

PWM Speed Control (Jumper Removed!)

EN B

**GP20**

PWM Speed Control (Optional for Motor 2)

Logic VSS

**L7805CV**

Powered safely by the 5V Regulator output

### N20 Motor Encoder

**Encoder Pin**

**Pico Pin**

**Notes**

VCC

**3V3(OUT)**

Must be 3.3V to protect Pico logic pins!

GND

**GND**

Shared common ground

C1

**GP26**

Hardware Interrupt (Counts ticks)

C2

**GP27**

Direction tracking

## 💻 Software Installation

### 1. Arduino IDE Setup (Microcontroller)

1.  Install the Raspberry Pi Pico board manager in the Arduino IDE.
    
2.  Install the following library via the Library Manager:
    
    -   **Adafruit_VL53L0X** (for the ToF sensors)
        
3.  Connect your Pico via USB.
    
4.  Open the `.ino` file and adjust the calibration variables at the top of the script (see Calibration section below).
    
5.  Compile and upload to the Pico.
    

### 2. Python Visualizer Setup (PC)

You will need Python 3 installed on your computer. Install the required dependencies using pip:

Bash

```
pip install pyserial vpython
```
