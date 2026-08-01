import serial
import time
import math
from vpython import *

# --- CONFIGURATION ---
COM_PORT = 'COM6' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE)
except Exception as e:
    print(f"Failed to connect to {COM_PORT}. Close Arduino IDE monitors!")
    exit()

# --- 3D SCENE SETUP ---
scene.title = "Pi Pico 3D Sensor Array (Full 6-DoF + IR)"
scene.background = color.gray(0.2)
scene.width = 800
scene.height = 600
scene.forward = vector(0.5, -0.5, -1)

# Create the breadboard body
board = box(length=10, width=6, height=0.5, color=color.blue)

# Create 3 cylinders to represent the ToF laser beams
tof1 = cylinder(axis=vector(0,1,0), radius=0.4, color=color.red)
tof2 = cylinder(axis=vector(0,1,0), radius=0.4, color=color.green)
tof3 = cylinder(axis=vector(0,1,0), radius=0.4, color=color.yellow)

# Create a visual indicator for the IR Obstacle Sensor (Front Bumper)
ir_indicator = box(length=0.5, width=6, height=1, color=color.green)

print("Reading data... Move your sensors!")

# Tracking variables for the Gyroscope
yaw_angle = 0.0
last_time = time.time()

while True:
    try:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            data = line.split(',')
            
            # Now expecting 10 pieces of data!
            if len(data) == 10:
                t1, t2, t3 = float(data[0]), float(data[1]), float(data[2])
                ax, ay, az = float(data[3]), float(data[4]), float(data[5])
                gx, gy, gz = float(data[6]), float(data[7]), float(data[8])
                ir_state = int(data[9]) # Read the 10th value (0 or 1)
                
                # --- TIME CALCULATION ---
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # --- 1. PITCH & ROLL (Absolute Gravity from Accelerometer) ---
                up_vector = vector(ax, -az, ay).norm()
                board.up = up_vector
                
                # --- 2. YAW (Integrated speed from Gyroscope) ---
                if abs(gz) > 50: 
                    gyro_z_dps = gz / 16.4 
                    yaw_angle += math.radians(gyro_z_dps) * dt

                # --- 3. COMBINE 3D MATH ---
                flat_forward = vector(math.sin(yaw_angle), 0, math.cos(yaw_angle))
                
                # Lock the forward vector to be exactly 90-degrees to our gravity vector
                board.axis = cross(board.up, cross(flat_forward, board.up)).norm() * 10
                board.length = 10 
                
                # To easily position sensors, let's calculate the "Right" direction
                board_right = cross(board.axis, board.up).norm()
                board_forward = board.axis.norm()
                
                # --- TOF VISUALIZATION ---
                beam1_length = max(t1 / 50.0, 0.1)
                beam2_length = max(t2 / 50.0, 0.1)
                beam3_length = max(t3 / 50.0, 0.1)
                
                tof1.axis = board.up * beam1_length
                tof2.axis = board.up * beam2_length
                tof3.axis = board.up * beam3_length
                
                # Position ToF sensors side-by-side using the right vector
                tof1.pos = board.pos + (board_right * -2) + (board.up * 0.25)
                tof2.pos = board.pos + (board.up * 0.25)
                tof3.pos = board.pos + (board_right * 2) + (board.up * 0.25)
                
                # --- IR SENSOR VISUALIZATION ---
                # Position the IR bumper at the very front edge of the board
                ir_indicator.pos = board.pos + (board_forward * 5) + (board.up * 0.5)
                # Rotate the bumper to match the board's orientation
                ir_indicator.axis = board.axis
                ir_indicator.up = board.up
                ir_indicator.length = 0.5 # keep it thin like a bumper
                
                # Turn RED if obstacle detected (0), GREEN if clear (1)
                if ir_state == 0:
                    ir_indicator.color = color.red
                else:
                    ir_indicator.color = color.green
                
    except Exception as e:
        # Ignore random serial glitches or parsing errors during startup
        pass