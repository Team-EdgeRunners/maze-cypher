// =============================================================================
// Maze Cypher — Micromouse Firmware for Raspberry Pi Pico
// Ported from MMS Simulator to Real Hardware
// =============================================================================
// Algorithm: Flood-Fill Exploration + 8-Connected A* Fast Run
// Hardware:  3x VL53L0X ToF, BMI160 IMU, 2x N20 Motors via L298N, Encoders
// =============================================================================

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// =============================================================================
// PIN DEFINITIONS
// =============================================================================

// --- ToF Sensor XSHUT Pins ---
// From the front: sensor1=LEFT, sensor2=FRONT, sensor3=RIGHT
#define XSHUT_LEFT   10
#define XSHUT_FRONT  11
#define XSHUT_RIGHT  12

// --- Motor Control Pins (L298N) ---
// Motor A = LEFT wheel
#define MOTOR_L_IN1  19
#define MOTOR_L_IN2  18
#define MOTOR_L_EN   21  // PWM

// Motor B = RIGHT wheel
#define MOTOR_R_IN1  16
#define MOTOR_R_IN2  17
#define MOTOR_R_EN   20  // PWM

// --- Encoder Pins ---
// Left motor encoder (M1)
#define ENC_L_C1     8   // Hardware interrupt
#define ENC_L_C2     9   // Direction

// Right motor encoder (M2)
#define ENC_R_C1     14  // Hardware interrupt
#define ENC_R_C2     15  // Direction

#define LED_PIN      LED_BUILTIN

// =============================================================================
// ROBOT PHYSICAL CONSTANTS
// =============================================================================

const float WHEEL_DIAMETER_CM   = 4.3;
const float GEAR_RATIO          = 100.0;
const float ENCODER_BASE_TICKS  = 7.0;

// Derived calibration
const float TICKS_PER_WHEEL_REV   = ENCODER_BASE_TICKS * GEAR_RATIO;          // 700
const float WHEEL_CIRCUMFERENCE   = WHEEL_DIAMETER_CM * 3.14159;              // ~13.51 cm
const float TICKS_PER_CM          = TICKS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE; // ~51.8

// =============================================================================
// MAZE & NAVIGATION CONSTANTS
// =============================================================================

const int   MAZE_SIZE            = 16;
const float CELL_SIZE_CM         = 18.0;
const long  TICKS_PER_CELL       = (long)(CELL_SIZE_CM * TICKS_PER_CM);  // ~933 ticks
const int   WALL_THRESHOLD_MM    = 150;   // Wall present if ToF reads below this
const int   COLLISION_THRESHOLD  = 30;    // Emergency stop if front sensor < 30mm
const int   PWM_CAP              = 128;   // 50% duty — protects 6V motors from 11.7V rail
const int   MOTOR_PWM_BASE       = 100;   // Base speed for exploration
const int   PWM_FAST             = 120;   // Speed for fast run
const unsigned long STALL_TIMEOUT_MS = 2000; // Emergency stop if no encoder tick for 2s

// BMI160 IMU
const int   BMI160_ADDR          = 0x69;
const float GYRO_SENSITIVITY     = 16.4;  // LSB per deg/s at ±2000 dps default range

// Wall bitmasks (absolute directions)
const uint8_t WALL_N = 0x01;
const uint8_t WALL_E = 0x02;
const uint8_t WALL_S = 0x04;
const uint8_t WALL_W = 0x08;

// =============================================================================
// HEADING & STATE ENUMS & DATA STRUCTURES
// =============================================================================

// Sequential clockwise for circular angular math
enum Heading : uint8_t {
  H_NORTH     = 0,
  H_NORTHEAST = 1,
  H_EAST      = 2,
  H_SOUTHEAST = 3,
  H_SOUTH     = 4,
  H_SOUTHWEST = 5,
  H_WEST      = 6,
  H_NORTHWEST = 7
};

enum MouseState : uint8_t {
  EXPLORING_TO_CENTER,
  RETURNING_TO_START,
  FAST_RUN
};

// Data structures defined early for Arduino preprocessor prototype generation
struct Mouse {
  int x = 0;
  int y = 0;
  Heading heading = H_NORTH;
};

struct Cell {
  int8_t x;
  int8_t y;
};

struct QueueEntry {
  uint8_t x;
  uint8_t y;
};

struct AStarNode {
  int8_t  x, y;
  int16_t g;        // Cumulative cost
  int16_t h;        // Heuristic
  Heading heading;

  int16_t f() const { return g + h; }
};

// =============================================================================
// HARDWARE OBJECTS
// =============================================================================

Adafruit_VL53L0X sensorLeft  = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorFront = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight = Adafruit_VL53L0X();

// =============================================================================
// ENCODER ISR VARIABLES (must be volatile)
// =============================================================================

volatile long encoderLeftCount  = 0;
volatile long encoderRightCount = 0;

void isrEncoderLeft() {
  if (digitalRead(ENC_L_C2) == HIGH) encoderLeftCount++;
  else encoderLeftCount--;
}

void isrEncoderRight() {
  if (digitalRead(ENC_R_C2) == HIGH) encoderRightCount++;
  else encoderRightCount--;
}

// =============================================================================
// MAZE DATA STRUCTURES (optimized for RP2040 SRAM)
// =============================================================================

uint8_t wallMap[MAZE_SIZE][MAZE_SIZE];
uint8_t distMap[MAZE_SIZE][MAZE_SIZE];
bool    visitedMap[MAZE_SIZE][MAZE_SIZE];

Mouse myMouse;
MouseState currentState = EXPLORING_TO_CENTER;

// --- A* Fast Run Path (static allocation, max 256 waypoints) ---
Cell    fastRunPath[256];
int     fastRunPathLen = 0;
int     pathIndex      = 0;

// --- BFS Queue (static ring buffer, avoids heap allocation) ---
QueueEntry bfsQueue[MAZE_SIZE * MAZE_SIZE];
int bfsHead = 0;
int bfsTail = 0;

void bfsReset()                    { bfsHead = 0; bfsTail = 0; }
bool bfsEmpty()                    { return bfsHead == bfsTail; }
void bfsPush(uint8_t x, uint8_t y) { bfsQueue[bfsTail++] = {x, y}; }
QueueEntry bfsPop()                { return bfsQueue[bfsHead++]; }

// --- A* Priority Queue (static, simple min-heap) ---
AStarNode pqNodes[512];
int pqSize = 0;

void pqReset() { pqSize = 0; }

void pqPush(AStarNode node) {
  if (pqSize >= 512) return;  // Safety: don't overflow
  pqNodes[pqSize] = node;
  // Sift up
  int i = pqSize;
  pqSize++;
  while (i > 0) {
    int parent = (i - 1) / 2;
    if (pqNodes[i].f() < pqNodes[parent].f()) {
      AStarNode tmp = pqNodes[i];
      pqNodes[i] = pqNodes[parent];
      pqNodes[parent] = tmp;
      i = parent;
    } else break;
  }
}

AStarNode pqPop() {
  AStarNode top = pqNodes[0];
  pqSize--;
  pqNodes[0] = pqNodes[pqSize];
  // Sift down
  int i = 0;
  while (true) {
    int left = 2 * i + 1;
    int right = 2 * i + 2;
    int smallest = i;
    if (left < pqSize && pqNodes[left].f() < pqNodes[smallest].f()) smallest = left;
    if (right < pqSize && pqNodes[right].f() < pqNodes[smallest].f()) smallest = right;
    if (smallest != i) {
      AStarNode tmp = pqNodes[i];
      pqNodes[i] = pqNodes[smallest];
      pqNodes[smallest] = tmp;
      i = smallest;
    } else break;
  }
  return top;
}

// =============================================================================
// ERROR HANDLING
// =============================================================================

void errorHalt(const char* message) {
  Serial.println("\n--- CRITICAL ERROR ---");
  Serial.println(message);
  stopMotors();
  while (1) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

// =============================================================================
// MOTOR CONTROL
// =============================================================================

void stopMotors() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_L_EN, 0);

  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
  analogWrite(MOTOR_R_EN, 0);
}

// Set individual motor speed and direction
// speed: -PWM_CAP to +PWM_CAP (positive = forward)
void setMotorLeft(int speed) {
  speed = constrain(speed, -PWM_CAP, PWM_CAP);
  if (speed >= 0) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    speed = -speed;
  }
  analogWrite(MOTOR_L_EN, speed);
}

void setMotorRight(int speed) {
  speed = constrain(speed, -PWM_CAP, PWM_CAP);
  if (speed >= 0) {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
  } else {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    speed = -speed;
  }
  analogWrite(MOTOR_R_EN, speed);
}

// =============================================================================
// SENSOR READING
// =============================================================================

// Read a single ToF sensor distance in mm, returns 9999 on failure
int readToF(Adafruit_VL53L0X &sensor) {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  }
  return 9999;  // Out of range / error → treat as no wall
}

bool wallLeft()  { return readToF(sensorLeft)  < WALL_THRESHOLD_MM; }
bool wallFront() { return readToF(sensorFront) < WALL_THRESHOLD_MM; }
bool wallRight() { return readToF(sensorRight) < WALL_THRESHOLD_MM; }

// Read BMI160 gyroscope Z-axis (raw value)
int16_t readGyroZ() {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x16);  // GYR_Z LSB register
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 2);

  int16_t gz = 0;
  if (Wire.available() == 2) {
    gz = Wire.read() | (Wire.read() << 8);
  }
  return gz;
}

// =============================================================================
// MOVEMENT PRIMITIVES
// =============================================================================

// Drive forward exactly one maze cell (18cm) using encoder feedback
// Simple P-controller keeps both wheels synchronized
void driveOneCellForward() {
  noInterrupts();
  encoderLeftCount  = 0;
  encoderRightCount = 0;
  interrupts();

  unsigned long lastTickTime = millis();
  long prevLeft = 0;

  Serial.print("FWD 1 cell (");
  Serial.print(TICKS_PER_CELL);
  Serial.println(" ticks)");

  while (true) {
    noInterrupts();
    long leftTicks  = abs(encoderLeftCount);
    long rightTicks = abs(encoderRightCount);
    interrupts();

    // Target reached?
    long avgTicks = (leftTicks + rightTicks) / 2;
    if (avgTicks >= TICKS_PER_CELL) break;

    // Stall detection
    if (leftTicks != prevLeft) {
      lastTickTime = millis();
      prevLeft = leftTicks;
    }
    if (millis() - lastTickTime > STALL_TIMEOUT_MS) {
      Serial.println("STALL DETECTED!");
      stopMotors();
      errorHalt("Motor stalled during forward drive");
    }

    // Collision check — emergency stop if too close to front wall
    int frontDist = readToF(sensorFront);
    if (frontDist < COLLISION_THRESHOLD) {
      Serial.println("COLLISION IMMINENT!");
      stopMotors();
      delay(200);
      return;  // Abort this move, don't halt — let the algorithm reroute
    }

    // Proportional correction: speed up the slower wheel
    int error = (int)(leftTicks - rightTicks);
    int correction = error / 4;  // P-gain = 0.25

    int leftPWM  = constrain(MOTOR_PWM_BASE - correction, 40, PWM_CAP);
    int rightPWM = constrain(MOTOR_PWM_BASE + correction, 40, PWM_CAP);

    setMotorLeft(leftPWM);
    setMotorRight(rightPWM);

    delay(2);  // Control loop at ~500Hz
  }

  stopMotors();
  delay(50);  // Brief settling time
}

// Turn in place by a given angle in degrees (positive = right, negative = left)
// Uses BMI160 gyroscope Z-axis integration for accurate heading control
void turnByAngle(float targetDegrees) {
  Serial.print("TURN ");
  Serial.print(targetDegrees);
  Serial.println(" deg");

  float accumulated = 0.0;
  float abstarget = abs(targetDegrees);
  unsigned long prevTime = micros();

  // Determine turn direction
  int leftSpeed, rightSpeed;
  if (targetDegrees > 0) {
    // Turn RIGHT: left wheel forward, right wheel backward
    leftSpeed  =  MOTOR_PWM_BASE;
    rightSpeed = -MOTOR_PWM_BASE;
  } else {
    // Turn LEFT: left wheel backward, right wheel forward
    leftSpeed  = -MOTOR_PWM_BASE;
    rightSpeed =  MOTOR_PWM_BASE;
  }

  setMotorLeft(leftSpeed);
  setMotorRight(rightSpeed);

  while (accumulated < abstarget) {
    int16_t gz_raw = readGyroZ();
    float gz_dps = (float)gz_raw / GYRO_SENSITIVITY;  // Convert to degrees per second

    unsigned long now = micros();
    float dt = (float)(now - prevTime) / 1000000.0;
    prevTime = now;

    accumulated += abs(gz_dps) * dt;

    // Slow down as we approach the target for precision
    if (accumulated > abstarget * 0.75) {
      int slowPWM = 60;  // Reduced speed for final approach
      if (targetDegrees > 0) {
        setMotorLeft(slowPWM);
        setMotorRight(-slowPWM);
      } else {
        setMotorLeft(-slowPWM);
        setMotorRight(slowPWM);
      }
    }

    delayMicroseconds(500);  // ~2000Hz gyro sampling
  }

  stopMotors();
  delay(100);  // Settling time after turn
}

// High-level turn functions matching the simulator API
void turnLeft90()  { turnByAngle(-90.0); }
void turnRight90() { turnByAngle(90.0); }
void turnAround()  { turnByAngle(180.0); }

// =============================================================================
// MOUSE MOVEMENT (replaces API::moveForward / turnLeft / turnRight)
// =============================================================================

void moveMouseForward() {
  driveOneCellForward();

  // Update logical position
  if (myMouse.heading == H_NORTH)      myMouse.y++;
  else if (myMouse.heading == H_EAST)  myMouse.x++;
  else if (myMouse.heading == H_SOUTH) myMouse.y--;
  else if (myMouse.heading == H_WEST)  myMouse.x--;

  Serial.print("POS: (");
  Serial.print(myMouse.x);
  Serial.print(",");
  Serial.print(myMouse.y);
  Serial.println(")");
}

void turnMouseLeft() {
  turnLeft90();
  myMouse.heading = static_cast<Heading>((myMouse.heading + 6) % 8);
}

void turnMouseRight() {
  turnRight90();
  myMouse.heading = static_cast<Heading>((myMouse.heading + 2) % 8);
}

// =============================================================================
// WALL MAPPING (from simulation, API calls replaced with hardware reads)
// =============================================================================

void setWallAbsolute(int x, int y, Heading dir) {
  if (dir == H_NORTH) {
    wallMap[x][y] |= WALL_N;
    if (y + 1 < MAZE_SIZE) wallMap[x][y + 1] |= WALL_S;
  }
  else if (dir == H_EAST) {
    wallMap[x][y] |= WALL_E;
    if (x + 1 < MAZE_SIZE) wallMap[x + 1][y] |= WALL_W;
  }
  else if (dir == H_SOUTH) {
    wallMap[x][y] |= WALL_S;
    if (y - 1 >= 0) wallMap[x][y - 1] |= WALL_N;
  }
  else if (dir == H_WEST) {
    wallMap[x][y] |= WALL_W;
    if (x - 1 >= 0) wallMap[x - 1][y] |= WALL_E;
  }
}

void updateWalls() {
  visitedMap[myMouse.x][myMouse.y] = true;

  Heading frontDir = myMouse.heading;
  Heading leftDir  = static_cast<Heading>((myMouse.heading + 6) % 8);
  Heading rightDir = static_cast<Heading>((myMouse.heading + 2) % 8);

  if (wallFront()) setWallAbsolute(myMouse.x, myMouse.y, frontDir);
  if (wallLeft())  setWallAbsolute(myMouse.x, myMouse.y, leftDir);
  if (wallRight()) setWallAbsolute(myMouse.x, myMouse.y, rightDir);

  // Debug: print current cell walls
  Serial.print("WALLS @(");
  Serial.print(myMouse.x);
  Serial.print(",");
  Serial.print(myMouse.y);
  Serial.print("): ");
  if (wallMap[myMouse.x][myMouse.y] & WALL_N) Serial.print("N");
  if (wallMap[myMouse.x][myMouse.y] & WALL_E) Serial.print("E");
  if (wallMap[myMouse.x][myMouse.y] & WALL_S) Serial.print("S");
  if (wallMap[myMouse.x][myMouse.y] & WALL_W) Serial.print("W");
  Serial.println();
}

// =============================================================================
// FLOOD FILL (BFS, identical logic to simulation)
// =============================================================================

void floodFill(bool routingToCenter) {
  for (int i = 0; i < MAZE_SIZE; i++)
    for (int j = 0; j < MAZE_SIZE; j++)
      distMap[i][j] = 255;

  bfsReset();

  if (routingToCenter) {
    distMap[7][7] = 0; bfsPush(7, 7);
    distMap[7][8] = 0; bfsPush(7, 8);
    distMap[8][7] = 0; bfsPush(8, 7);
    distMap[8][8] = 0; bfsPush(8, 8);
  } else {
    distMap[0][0] = 0;
    bfsPush(0, 0);
  }

  while (!bfsEmpty()) {
    QueueEntry cur = bfsPop();
    uint8_t cx = cur.x;
    uint8_t cy = cur.y;
    uint8_t cd = distMap[cx][cy];

    if (!(wallMap[cx][cy] & WALL_N) && cy + 1 < MAZE_SIZE && distMap[cx][cy + 1] == 255) {
      distMap[cx][cy + 1] = cd + 1;
      bfsPush(cx, cy + 1);
    }
    if (!(wallMap[cx][cy] & WALL_E) && cx + 1 < MAZE_SIZE && distMap[cx + 1][cy] == 255) {
      distMap[cx + 1][cy] = cd + 1;
      bfsPush(cx + 1, cy);
    }
    if (!(wallMap[cx][cy] & WALL_S) && cy - 1 >= 0 && distMap[cx][cy - 1] == 255) {
      distMap[cx][cy - 1] = cd + 1;
      bfsPush(cx, cy - 1);
    }
    if (!(wallMap[cx][cy] & WALL_W) && cx - 1 >= 0 && distMap[cx - 1][cy] == 255) {
      distMap[cx - 1][cy] = cd + 1;
      bfsPush(cx - 1, cy);
    }
  }
}

// =============================================================================
// A* PATHFINDER (ported from simulation, static memory)
// =============================================================================

int getHeuristic(int x, int y) {
  int targetX = (x < 8) ? 7 : 8;
  int targetY = (y < 8) ? 7 : 8;
  return (abs(targetX - x) + abs(targetY - y)) * 10;
}

bool isPathBlocked(int cx, int cy, int dirIndex) {
  if (dirIndex == 0) return (wallMap[cx][cy] & WALL_N) != 0;
  if (dirIndex == 2) return (wallMap[cx][cy] & WALL_E) != 0;
  if (dirIndex == 4) return (wallMap[cx][cy] & WALL_S) != 0;
  if (dirIndex == 6) return (wallMap[cx][cy] & WALL_W) != 0;

  // Diagonal checks — both adjacent orthogonal walls must be clear
  if (dirIndex == 1) {  // NORTHEAST
    if ((wallMap[cx][cy] & WALL_N) || (wallMap[cx][cy] & WALL_E)) return true;
    if (cx + 1 < MAZE_SIZE && (wallMap[cx + 1][cy] & WALL_N)) return true;
    if (cy + 1 < MAZE_SIZE && (wallMap[cx][cy + 1] & WALL_E)) return true;
    return false;
  }
  if (dirIndex == 3) {  // SOUTHEAST
    if ((wallMap[cx][cy] & WALL_S) || (wallMap[cx][cy] & WALL_E)) return true;
    if (cx + 1 < MAZE_SIZE && (wallMap[cx + 1][cy] & WALL_S)) return true;
    if (cy - 1 >= 0 && (wallMap[cx][cy - 1] & WALL_E)) return true;
    return false;
  }
  if (dirIndex == 5) {  // SOUTHWEST
    if ((wallMap[cx][cy] & WALL_S) || (wallMap[cx][cy] & WALL_W)) return true;
    if (cx - 1 >= 0 && (wallMap[cx - 1][cy] & WALL_S)) return true;
    if (cy - 1 >= 0 && (wallMap[cx][cy - 1] & WALL_W)) return true;
    return false;
  }
  if (dirIndex == 7) {  // NORTHWEST
    if ((wallMap[cx][cy] & WALL_N) || (wallMap[cx][cy] & WALL_W)) return true;
    if (cx - 1 >= 0 && (wallMap[cx - 1][cy] & WALL_N)) return true;
    if (cy + 1 < MAZE_SIZE && (wallMap[cx][cy + 1] & WALL_W)) return true;
    return false;
  }
  return true;
}

void calculateAStarPath() {
  int16_t closedG[MAZE_SIZE][MAZE_SIZE];
  Cell    parentMap[MAZE_SIZE][MAZE_SIZE];

  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      closedG[i][j] = 30000;
      parentMap[i][j] = { -1, -1 };
    }
  }

  pqReset();
  closedG[0][0] = 0;
  pqPush({0, 0, 0, (int16_t)getHeuristic(0, 0), H_NORTH});

  Cell destination = { -1, -1 };
  fastRunPathLen = 0;

  while (pqSize > 0) {
    AStarNode curr = pqPop();

    if (curr.g > closedG[curr.x][curr.y]) continue;

    if ((curr.x == 7 || curr.x == 8) && (curr.y == 7 || curr.y == 8)) {
      destination = { curr.x, curr.y };
      break;
    }

    const Heading headings[8] = { H_NORTH, H_NORTHEAST, H_EAST, H_SOUTHEAST,
                                   H_SOUTH, H_SOUTHWEST, H_WEST, H_NORTHWEST };
    const int8_t dx[8] = { 0,  1, 1,  1, 0, -1, -1, -1 };
    const int8_t dy[8] = { 1,  1, 0, -1, -1, -1,  0,  1 };
    const int16_t baseCosts[8] = { 10, 14, 10, 14, 10, 14, 10, 14 };

    for (int i = 0; i < 8; i++) {
      if (!isPathBlocked(curr.x, curr.y, i)) {
        int nx = curr.x + dx[i];
        int ny = curr.y + dy[i];

        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
          if (!visitedMap[nx][ny]) continue;

          int16_t stepCost = baseCosts[i];

          // Circular turn penalty
          int diff = abs((int)headings[i] - (int)curr.heading);
          if (diff > 4) diff = 8 - diff;

          if (diff == 1)      stepCost += 15;
          else if (diff == 2) stepCost += 75;
          else if (diff >= 3) stepCost += 150;

          int16_t nextG = curr.g + stepCost;

          if (nextG < closedG[nx][ny]) {
            closedG[nx][ny] = nextG;
            parentMap[nx][ny] = { (int8_t)curr.x, (int8_t)curr.y };
            pqPush({(int8_t)nx, (int8_t)ny, nextG, (int16_t)getHeuristic(nx, ny), headings[i]});
          }
        }
      }
    }
  }

  if (destination.x == -1) {
    Serial.println("A* FAILED: No path found!");
    return;
  }

  // Backtrack to build path (stored in reverse, then reversed)
  Cell backtrack = destination;
  while (backtrack.x != 0 || backtrack.y != 0) {
    if (fastRunPathLen < 256) {
      fastRunPath[fastRunPathLen++] = backtrack;
    }
    Cell parent = parentMap[backtrack.x][backtrack.y];
    backtrack = parent;
  }

  // Reverse in place
  for (int i = 0; i < fastRunPathLen / 2; i++) {
    Cell tmp = fastRunPath[i];
    fastRunPath[i] = fastRunPath[fastRunPathLen - 1 - i];
    fastRunPath[fastRunPathLen - 1 - i] = tmp;
  }

  Serial.print("A* path found: ");
  Serial.print(fastRunPathLen);
  Serial.println(" waypoints");
}

// =============================================================================
// BMI160 INITIALIZATION (raw I2C, from reference code)
// =============================================================================

void initBMI160() {
  // Power on accelerometer (normal mode)
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x7E);  // CMD register
  Wire.write(0x11);  // acc_set_pmu_mode(normal)
  Wire.endTransmission();
  delay(50);

  // Power on gyroscope (normal mode)
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(0x7E);
  Wire.write(0x15);  // gyr_set_pmu_mode(normal)
  Wire.endTransmission();
  delay(50);
}

// =============================================================================
// HARDWARE INITIALIZATION
// =============================================================================

void initToFSensors() {
  Serial.println("Booting ToF Sensors...");

  // Shut down all sensors
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  // Bring up sensor 1 (LEFT) at address 0x30
  pinMode(XSHUT_LEFT, INPUT);
  delay(100);
  if (!sensorLeft.begin(0x30, false, &Wire))
    errorHalt("ToF LEFT (sensor1) init failed!");

  // Bring up sensor 2 (FRONT) at address 0x31
  pinMode(XSHUT_FRONT, INPUT);
  delay(100);
  if (!sensorFront.begin(0x31, false, &Wire))
    errorHalt("ToF FRONT (sensor2) init failed!");

  // Bring up sensor 3 (RIGHT) at default address 0x29
  pinMode(XSHUT_RIGHT, INPUT);
  delay(100);
  if (!sensorRight.begin(0x29, false, &Wire))
    errorHalt("ToF RIGHT (sensor3) init failed!");

  Serial.println("ToF sensors OK.");
}

void initMotors() {
  Serial.println("Booting Motors & Encoders...");

  // Motor pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_L_EN,  OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  pinMode(MOTOR_R_EN,  OUTPUT);

  stopMotors();

  // Encoder pins
  pinMode(ENC_L_C1, INPUT_PULLUP);
  pinMode(ENC_L_C2, INPUT_PULLUP);
  pinMode(ENC_R_C1, INPUT_PULLUP);
  pinMode(ENC_R_C2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), isrEncoderLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), isrEncoderRight, RISING);

  Serial.println("Motors & encoders OK.");
}

void initMaze() {
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      wallMap[i][j] = 0;
      distMap[i][j] = 255;
      visitedMap[i][j] = false;
    }
  }

  // Set outer boundary walls
  for (int i = 0; i < MAZE_SIZE; i++) {
    wallMap[i][0]             |= WALL_S;
    wallMap[i][MAZE_SIZE - 1] |= WALL_N;
    wallMap[0][i]             |= WALL_W;
    wallMap[MAZE_SIZE - 1][i] |= WALL_E;
  }
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // USB serial safety delay (Pico needs time for USB enumeration)
  for (int i = 0; i < 50; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  digitalWrite(LED_PIN, HIGH);

  Serial.println("\n========================================");
  Serial.println("  Maze Cypher — Micromouse Firmware");
  Serial.println("  Hardware Init Starting...");
  Serial.println("========================================");

  // I2C bus on GP4 (SDA) and GP5 (SCL) - default pins on Pico
  #if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
  Wire.setSDA(4);
  Wire.setSCL(5);
  #endif
  Wire.begin();

  initToFSensors();
  Serial.println("Booting BMI160 IMU...");
  initBMI160();
  Serial.println("BMI160 OK.");
  initMotors();
  initMaze();

  Serial.println("\n--- All hardware initialized! ---");
  Serial.println("Starting maze exploration...\n");

  // Brief pause before starting
  delay(1000);
}

// =============================================================================
// MAIN LOOP — State Machine
// =============================================================================

void loop() {
  // --- Exploration & Return phases: Flood-fill + greedy step ---
  if (currentState != FAST_RUN) {
    updateWalls();

    // Check if we reached the center
    if (currentState == EXPLORING_TO_CENTER &&
        (myMouse.x == 7 || myMouse.x == 8) &&
        (myMouse.y == 7 || myMouse.y == 8)) {
      Serial.println("\n*** CENTER REACHED! Returning to start... ***\n");
      currentState = RETURNING_TO_START;
      return;
    }

    // Check if we returned to start
    if (currentState == RETURNING_TO_START &&
        myMouse.x == 0 && myMouse.y == 0) {
      Serial.println("\n*** START REACHED! Computing A* fast run path... ***\n");
      currentState = FAST_RUN;
      return;
    }

    // Run flood fill for current goal
    floodFill(currentState == EXPLORING_TO_CENTER);

    // Pick the neighbor with the lowest distance
    int minDistance = 255;
    Heading bestDirection = myMouse.heading;

    int mx = myMouse.x;
    int my = myMouse.y;

    if (!(wallMap[mx][my] & WALL_N) && my + 1 < MAZE_SIZE && distMap[mx][my + 1] < minDistance) {
      minDistance = distMap[mx][my + 1];
      bestDirection = H_NORTH;
    }
    if (!(wallMap[mx][my] & WALL_E) && mx + 1 < MAZE_SIZE && distMap[mx + 1][my] < minDistance) {
      minDistance = distMap[mx + 1][my];
      bestDirection = H_EAST;
    }
    if (!(wallMap[mx][my] & WALL_S) && my - 1 >= 0 && distMap[mx][my - 1] < minDistance) {
      minDistance = distMap[mx][my - 1];
      bestDirection = H_SOUTH;
    }
    if (!(wallMap[mx][my] & WALL_W) && mx - 1 >= 0 && distMap[mx - 1][my] < minDistance) {
      minDistance = distMap[mx - 1][my];
      bestDirection = H_WEST;
    }

    // Turn to face the best direction, then move forward
    int turnStep = (bestDirection - myMouse.heading + 8) % 8;
    if (turnStep == 2)      turnMouseRight();
    else if (turnStep == 4) { turnMouseRight(); turnMouseRight(); }
    else if (turnStep == 6) turnMouseLeft();

    moveMouseForward();
  }
  // --- Fast Run phase: Follow A* path ---
  else {
    if (fastRunPathLen == 0) {
      calculateAStarPath();
      pathIndex = 0;

      if (fastRunPathLen == 0) {
        Serial.println("No path computed — halting.");
        stopMotors();
        while (1) { delay(1000); }
      }

      Serial.println("A* path locked. Starting fast run!");
    }

    if (pathIndex < fastRunPathLen) {
      Cell nextCell = fastRunPath[pathIndex];

      // Decompose diagonal targets into orthogonal steps
      // (resolve X-axis first if target is diagonal)
      if (nextCell.x != myMouse.x && nextCell.y != myMouse.y) {
        nextCell.y = myMouse.y;  // Move along X first
      }

      // Determine required heading
      Heading targetHeading = myMouse.heading;
      if (nextCell.y > myMouse.y)      targetHeading = H_NORTH;
      else if (nextCell.x > myMouse.x) targetHeading = H_EAST;
      else if (nextCell.y < myMouse.y) targetHeading = H_SOUTH;
      else if (nextCell.x < myMouse.x) targetHeading = H_WEST;

      // Execute turn
      int turnStep = (targetHeading - myMouse.heading + 8) % 8;
      if (turnStep == 2)      turnMouseRight();
      else if (turnStep == 4) { turnMouseRight(); turnMouseRight(); }
      else if (turnStep == 6) turnMouseLeft();

      moveMouseForward();

      // Advance path index only when we've reached the actual waypoint
      if (myMouse.x == fastRunPath[pathIndex].x &&
          myMouse.y == fastRunPath[pathIndex].y) {
        pathIndex++;
      }
    } else {
      Serial.println("\n========================================");
      Serial.println("  FAST RUN COMPLETE!");
      Serial.println("  Center reached via optimized path.");
      Serial.println("========================================");

      // Victory LED pattern
      stopMotors();
      while (1) {
        digitalWrite(LED_PIN, HIGH); delay(200);
        digitalWrite(LED_PIN, LOW);  delay(200);
      }
    }
  }
}