
#include <Servo.h>
#include <Wire.h>
#include <math.h>


// Servo signal: D12
// Ultrasonic TRIG: D9
// Ultrasonic ECHO: D2 (external interrupt)
// L293D EN1/EN2: D5/D6 (PWM)
// L293D IN1/IN2: D3/D4
// L293D IN3/IN4: D7/D8
// MPU6050 SDA: A4, SCL: A5

const byte SERVO_PIN = 12;
const byte TRIG_PIN  = 9;
const byte ECHO_PIN  = 2;

const byte ENA = 5;
const byte ENB = 6;

const byte IN1 = 3;
const byte IN2 = 4;
const byte IN3 = 7;
const byte IN4 = 8;


const int SAFE_DISTANCE = 55;
const int MAX_DISTANCE  = 250;

const int SCAN_STEP = 30;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;

const int BASE_SPEED = 150;
const int TURN_SPEED = 120;

const unsigned long SERVO_SETTLE_MS = 500;
const unsigned long ECHO_TIMEOUT_US = 25000;
const unsigned long FORWARD_STEP_MS = 3000;
const unsigned long BACKUP_MS = 3000;

const float CELL_CM = 10.0;
const int MAP_SIZE = 11;
const int MAP_CENTER = MAP_SIZE / 2;


Servo scanner;


volatile unsigned long echoRiseTime = 0;
volatile unsigned long echoPulseWidth = 0;
volatile bool echoFinished = false;
volatile bool echoRiseDetected = false;

unsigned long triggerStart = 0;
bool echoPending = false;


const byte MPU_ADDR = 0x68;

float gyroZBias = 0;
float headingDeg = 0;

unsigned long lastGyroUpdate = 0;
bool imuAvailable = false;

// ---------------- ROBOT POSE -------------------------

float robotX = 0;
float robotY = 0;

byte occupancy[MAP_SIZE][MAP_SIZE];


int scanAngle = MIN_ANGLE;
int scanDirection = 1;

unsigned long servoMovedAt = 0;
bool servoSettled = false;

long frontDistance = MAX_DISTANCE;
long bestDistance = 0;
int bestAngle = 90;

bool scanMeasurementReady = false;


float targetHeading = 0;

float Kp = 2.5;
float Ki = 0.0;
float Kd = 0.15;

float pidIntegral = 0;
float previousError = 0;

const float MAX_TURN_OUTPUT = 100;
const float HEADING_TOLERANCE = 4.0;


enum RoverState {
  SCANNING,
  MOVING_FORWARD,
  BACKING_UP,
  TURNING,
  BLOCKED
};

RoverState state = SCANNING;

unsigned long stateStartedAt = 0;
unsigned long lastMotionUpdate = 0;

int turnDirection = 1;

float turnStartHeading = 0;
float turnTargetChange = 60.0;




float normalizeAngle(float angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}

float angleError(float target, float current) {
  float error = target - current;

  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;

  return error;
}




void setMotorA(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  if (speedValue > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speedValue < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, abs(speedValue));
}

void setMotorB(int speedValue) {
  speedValue = constrain(speedValue, -255, 255);

  if (speedValue > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speedValue < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENB, abs(speedValue));
}

void drive(int leftSpeed, int rightSpeed) {
  setMotorA(leftSpeed);
  setMotorB(rightSpeed);
}

void stopMotors() {
  drive(0, 0);
}

void forward() {
  drive(BASE_SPEED, BASE_SPEED);
}

void reverse() {
  drive(-BASE_SPEED, -BASE_SPEED);
}




void echoISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echoRiseTime = micros();
    echoRiseDetected = true;
  } else if (echoRiseDetected) {
    echoPulseWidth = micros() - echoRiseTime;
    echoFinished = true;
    echoRiseDetected = false;
  }
}

void startUltrasonicMeasurement() {
  noInterrupts();
  echoFinished = false;
  echoRiseDetected = false;
  echoPulseWidth = 0;
  interrupts();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  triggerStart = micros();
  echoPending = true;
}

bool getUltrasonicResult(long &distance) {
  unsigned long pulseWidth = 0;
  bool finished = false;

  noInterrupts();

  if (echoFinished) {
    pulseWidth = echoPulseWidth;
    echoFinished = false;
    finished = true;
  }

  interrupts();

  if (finished) {
    echoPending = false;
    distance = pulseWidth * 0.0343 / 2.0;

    if (distance < 2 || distance > MAX_DISTANCE) {
      distance = MAX_DISTANCE;
    }

    return true;
  }

  if (micros() - triggerStart > ECHO_TIMEOUT_US) {
    echoPending = false;
    distance = MAX_DISTANCE;
    return true;
  }

  return false;
}



void writeMPU(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, (byte)14) != 14) {
    return false;
  }

 
  for (int i = 0; i < 6; i++) Wire.read();

  int16_t gx = Wire.read();
  int16_t gy = Wire.read();
  int16_t gz = Wire.read();


  (void)gx;
  (void)gy;

  unsigned long now = micros();

  if (lastGyroUpdate != 0) {
    float dt = (now - lastGyroUpdate) / 1000000.0;

   
    float rateZ = (gz / 131.0) - gyroZBias;

    headingDeg = normalizeAngle(headingDeg + rateZ * dt);
  }

  lastGyroUpdate = now;
  return true;
}

void initializeIMU() {
  Wire.begin();


  writeMPU(0x6B, 0x00);

 
  writeMPU(0x1B, 0x00);

  delay(100);


  long sum = 0;
  int validSamples = 0;

  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);

    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(MPU_ADDR, (byte)2) == 2) {
      int16_t gz = ((int16_t)Wire.read() << 8) | Wire.read();
      sum += gz;
      validSamples++;
    }

    delay(3);
  }

  if (validSamples > 0) {
    gyroZBias = (sum / (float)validSamples) / 131.0;
    imuAvailable = true;
  } else {
    imuAvailable = false;
  }

  lastGyroUpdate = micros();
}

void updateHeading() {
  if (imuAvailable) {
    readMPU();
  }
}



int calculatePID() {
  float error = angleError(targetHeading, headingDeg);

  unsigned long now = millis();
  float dt = (now - lastMotionUpdate) / 1000.0;

  if (dt <= 0 || dt > 0.5) dt = 0.02;

  pidIntegral += error * dt;
  pidIntegral = constrain(pidIntegral, -50.0, 50.0);

  float derivative = (error - previousError) / dt;

  float output =
      Kp * error +
      Ki * pidIntegral +
      Kd * derivative;

  previousError = error;
  lastMotionUpdate = now;

  return constrain((int)output,
                   -(int)MAX_TURN_OUTPUT,
                   (int)MAX_TURN_OUTPUT);
}

void beginTurn(int direction) {
  turnDirection = direction;

  targetHeading = normalizeAngle(
    headingDeg + direction * turnTargetChange
  );

  pidIntegral = 0;
  previousError = 0;
  lastMotionUpdate = millis();

  state = TURNING;
  stateStartedAt = millis();
}

void runPIDTurn() {
  if (!imuAvailable) {
   
    if (turnDirection > 0) {
      drive(TURN_SPEED, -TURN_SPEED);
    } else {
      drive(-TURN_SPEED, TURN_SPEED);
    }

    if (millis() - stateStartedAt >= 450) {
      stopMotors();
      beginScan();
    }

    return;
  }

  updateHeading();

  float error = angleError(targetHeading, headingDeg);

  if (abs(error) < HEADING_TOLERANCE) {
    stopMotors();
    beginScan();
    return;
  }

  int correction = calculatePID();


  if (correction > 0 && correction < 65) correction = 65;
  if (correction < 0 && correction > -65) correction = -65;

  correction = constrain(correction, -TURN_SPEED, TURN_SPEED);


  drive(correction, -correction);

 
  if (millis() - stateStartedAt > 5000) {
    stopMotors();
    state = BLOCKED;
  }
}


bool insideMap(int gx, int gy) {
  return gx >= 0 && gx < MAP_SIZE &&
         gy >= 0 && gy < MAP_SIZE;
}

void markGrid(float x, float y, byte value) {
  int gx = (int)round(x / CELL_CM) + MAP_CENTER;
  int gy = (int)round(y / CELL_CM) + MAP_CENTER;

  if (insideMap(gx, gy)) {
    occupancy[gy][gx] = value;
  }
}

void recordObstacle(int angle, long distanceCm) {
  float relativeAngle = (angle - 90) * PI / 180.0;

  float localX = distanceCm * sin(relativeAngle);
  float localY = distanceCm * cos(relativeAngle);

  float headingRad = headingDeg * PI / 180.0;

  float worldX = robotX +
                 localX * cos(headingRad) +
                 localY * sin(headingRad);

  float worldY = robotY -
                 localX * sin(headingRad) +
                 localY * cos(headingRad);

  if (distanceCm < MAX_DISTANCE) {
    markGrid(worldX, worldY, 2);
  }

  Serial.print("COORD,");
  Serial.print(angle);
  Serial.print(',');
  Serial.print(distanceCm);
  Serial.print(',');
  Serial.print(worldX, 2);
  Serial.print(',');
  Serial.println(worldY, 2);
}

void printOccupancyMap() {
  Serial.println("MAP_BEGIN");

  for (int row = 0; row < MAP_SIZE; row++) {
    for (int col = 0; col < MAP_SIZE; col++){
    if (row == MAP_CENTER && col == MAP_CENTER) {
        Serial.print(5);
      } else {
        Serial.print(occupancy[row][col]);
      
      //Serial.print(occupancy[row][col]);
    }
      if (col < MAP_SIZE - 1) Serial.print(',');
    }

    Serial.println();
  }
  
  Serial.println("MAP_END");
}


void beginScan() {
  stopMotors();

  scanAngle = MIN_ANGLE;
  scanDirection = 1;
  servoSettled = false;
  echoPending = false;

  frontDistance = MAX_DISTANCE;
  bestDistance = 0;
  bestAngle = 90;

  scanner.write(scanAngle);
  servoMovedAt = millis();

  state = SCANNING;
  stateStartedAt = millis();
}

void performScan() {
  if (!servoSettled) {
    if (millis() - servoMovedAt >= SERVO_SETTLE_MS) {
      startUltrasonicMeasurement();
      servoSettled = true;
    }
    return;
  }

  long distanceCm;

  if (echoPending) {
    if (!getUltrasonicResult(distanceCm)) {
      return;
    }
  } else {
    return;
  }

  updateHeading();

  if (scanAngle == 90) {
    frontDistance = distanceCm;
  }

  if (distanceCm > bestDistance) {
    bestDistance = distanceCm;
    bestAngle = scanAngle;
  }

  recordObstacle(scanAngle, distanceCm);

  Serial.print("SCAN,");
  Serial.print(scanAngle);
  Serial.print(',');
  Serial.println(distanceCm);

  scanAngle += SCAN_STEP * scanDirection;

  if (scanAngle > MAX_ANGLE) {
    scanAngle = MAX_ANGLE;
    scanDirection = -1;
  } else if (scanAngle < MIN_ANGLE) {
    printOccupancyMap();

    if (frontDistance > SAFE_DISTANCE) {
      state = MOVING_FORWARD;
      stateStartedAt = millis();
    } else if (bestDistance > SAFE_DISTANCE) {
      int direction = (bestAngle > 90) ? 1 : -1;
      beginTurn(direction);
    } else {
      state = BACKING_UP;
      stateStartedAt = millis();
    }

    servoSettled = false;
    return;
  }

  scanner.write(scanAngle);
  servoMovedAt = millis();
  servoSettled = false;
}


void runForward() {
 
  forward();

  if (millis() - stateStartedAt >= FORWARD_STEP_MS) {
    stopMotors();

  
    const float SPEED_CM_PER_SEC = 10.0;
    float dt = (millis() - stateStartedAt) / 1000.0;
    float distanceTravelled = SPEED_CM_PER_SEC * dt;

    float headingRad = headingDeg * PI / 180.0;

    robotX += distanceTravelled * sin(headingRad);
    robotY += distanceTravelled * cos(headingRad);

    markGrid(robotX, robotY, 1);

    beginScan();
  }
}



void runBackup() {
  reverse();

  if (millis() - stateStartedAt >= BACKUP_MS) {
    stopMotors();

  
    const float REVERSE_SPEED_CM_PER_SEC = 10.0;
    float distanceTravelled =
      REVERSE_SPEED_CM_PER_SEC *
      (millis() - stateStartedAt) / 1000.0;

    float headingRad = headingDeg * PI / 180.0;

    robotX -= distanceTravelled * sin(headingRad);
    robotY -= distanceTravelled * cos(headingRad);

    if (bestDistance > SAFE_DISTANCE) {
      beginTurn(bestAngle > 90 ? 1 : -1);
    } else {
      state = BLOCKED;
      stopMotors();
    }
  }
}



void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(115200);

  scanner.attach(SERVO_PIN);
  scanner.write(90);

  for (int y = 0; y < MAP_SIZE; y++) {
    for (int x = 0; x < MAP_SIZE; x++) {
      occupancy[y][x] = 0;
    }
  }

  attachInterrupt(
    digitalPinToInterrupt(ECHO_PIN),
    echoISR,
    CHANGE
  );

  stopMotors();

  initializeIMU();

  Serial.println("time_ms,event,details");
  Serial.println(imuAvailable ?
                 "IMU,READY" : "IMU,NOT_FOUND");
  Serial.println("ROVER_START");

  delay(500);
  beginScan();
}



void loop() {
  
  // Use 'static' so it remembers its value on every loop iteration
  static int lastStateInt = -1;

  // Print status only when the state actually changes
  if ((int)state != lastStateInt) {
    lastStateInt = (int)state;
    if (state == SCANNING) Serial.println("STATE: SCANNING");
    else if (state == MOVING_FORWARD) Serial.println("STATE: MOVING_FORWARD");
    else if (state == BACKING_UP) Serial.println("STATE: BACKING_UP");
    else if (state == TURNING) Serial.println("STATE: TURNING");
    else if (state == BLOCKED) Serial.println("STATE: BLOCKED");
  }
  switch (state) {
    case SCANNING:
      performScan();
      break;

    case MOVING_FORWARD:
          runForward();
      break;

    case BACKING_UP:
         runBackup();
      break;

    case TURNING:
         runPIDTurn();
      break;

    case BLOCKED:
      stopMotors();
    if (millis() - stateStartedAt >= 3000) {
        beginScan();
      }
      break;
  }
}

