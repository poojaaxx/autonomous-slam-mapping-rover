# Autonomous SLAM-Inspired Occupancy Mapping Rover

## Arduino Firmware – Motor Control and Autonomous Navigation

### 1. Overview

The Arduino firmware is responsible for controlling the movement of the autonomous rover, performing ultrasonic-based environmental scanning, detecting obstacles, and generating spatial coordinate information.

The system uses an Arduino Uno as its main controller and an L293D motor driver to control two DC gear motors.

The firmware combines motor control, servo-based ultrasonic scanning, distance measurement, obstacle avoidance, heading estimation, and occupancy grid generation.

### 2. Motor Control Architecture

The L293D motor driver is used as an interface between the Arduino Uno and the two DC motors.

The Arduino generates direction signals and PWM signals to control motor movement and speed.

#### Pin Configuration

| Component | Arduino Pin | Function |
|---|---|---|
| L293D EN1 (ENA) | D5 | Motor A speed control |
| L293D IN1 | D3 | Motor A direction |
| L293D IN2 | D4 | Motor A direction |
| L293D EN2 (ENB) | D6 | Motor B speed control |
| L293D IN3 | D7 | Motor B direction |
| L293D IN4 | D8 | Motor B direction |

Both enable pins use PWM signals for motor speed control.

### 3. Motor Control Implementation

The firmware implements separate functions for controlling the two DC motors.

**Motor A Control**

The `setMotorA()` function controls the direction and speed of the first motor.

**Motor B Control**

The `setMotorB()` function controls the direction and speed of the second motor.

**Combined Motor Control**

The `drive()` function coordinates both motors.

```cpp
void drive(int leftSpeed, int rightSpeed) {
    setMotorA(leftSpeed);
    setMotorB(rightSpeed);
}
```

#### Movement Functions

| Function | Operation |
|---|---|
| `forward()` | Moves both motors in their forward-command direction |
| `reverse()` | Reverses both motors |
| `stopMotors()` | Stops both motors |
| `drive()` | Controls both motors independently |
| `setMotorA()` | Controls Motor A |
| `setMotorB()` | Controls Motor B |

### 4. PWM Speed Control

The firmware uses PWM values to control motor speed.

```cpp
const int BASE_SPEED = 150;
const int TURN_SPEED = 120;
```

The PWM range is 0–255.

- Base speed: 150
- Turning speed: 120

Positive speed values select one motor direction, negative values select the opposite direction, and zero stops the motor command.

### 5. Autonomous Navigation

The rover uses a state-based control architecture.

The firmware defines five operating states:

```cpp
enum RoverState {
    SCANNING,
    MOVING_FORWARD,
    BACKING_UP,
    TURNING,
    BLOCKED
};
```

**SCANNING**

The ultrasonic sensor scans the surrounding environment using servo rotation.

**MOVING_FORWARD**

The rover moves forward when the measured front distance is greater than the configured safe distance.

**BACKING_UP**

The rover performs reverse movement when the front path is blocked and no suitable alternative direction is identified.

**TURNING**

The rover executes a turning operation using either gyro-based PID control or a timed turning fallback.

**BLOCKED**

The rover stops when a suitable movement direction is unavailable.

### 6. Obstacle Avoidance Logic

The firmware uses the following parameters:

```cpp
const int SAFE_DISTANCE = 55;
const int MAX_DISTANCE = 250;
```

The safe distance is configured as 25 cm, and the maximum measurement range is configured as 250 cm.

After scanning, the firmware evaluates the front distance and the best available scanning direction.

The rover then selects an appropriate movement state.

### 7. Optional MPU6050 Integration

The firmware includes optional MPU6050 gyroscope support through I2C communication.

| MPU6050 Pin | Arduino Pin |
|---|---|
| SDA | A4 |
| SCL | A5 |

The gyroscope provides angular velocity measurements used to estimate the rover's heading.

When the IMU is available, the firmware uses heading feedback and PID calculations for turning.

When the IMU is unavailable, the firmware uses a timed turning method.

**Note:** The displayed Tinkercad circuit does not include an MPU6050, so the timed turning fallback is relevant to that circuit configuration.

### 8. Servo-Based Ultrasonic Scanning

The ultrasonic sensor is mounted on a servo motor.

The firmware performs scanning across a 180-degree range.

```cpp
const int SCAN_STEP = 5;
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
```

The sensor measures distances at different angular positions.

The measured distance and scanning angle are used to calculate spatial coordinates.

### 9. Coordinate Generation

The firmware converts polar measurements into spatial coordinates.

```text
relativeAngle = (angle - 90) * PI / 180

localX = distance * sin(relativeAngle)
localY = distance * cos(relativeAngle)
```

The calculated coordinates are transformed using the estimated rover heading and position.

The resulting coordinates are transmitted through the Arduino serial interface.

### 10. Serial Communication

The Arduino generates structured serial output containing coordinate and scanning information.

Example:

```text
COORD,0,173,-173.00,35.00
SCAN,0,173

COORD,15,173,-167.11,79.78
SCAN,15,173

COORD,30,172,-148.96,121.00
SCAN,30,172
```

The `COORD` records contain:

- Scanning angle
- Measured distance
- X coordinate
- Y coordinate

The `SCAN` records indicate the scanning angle and measured distance.

These records are subsequently processed by Python to generate a 2D environmental visualization.

### 11. Implementation Workflow

1. Initialize Arduino peripherals and motor driver pins.
2. Initialize the servo and ultrasonic sensor.
3. Check optional IMU availability.
4. Perform environmental scanning.
5. Measure distances at different angles.
6. Generate spatial coordinates.
7. Update the internal occupancy grid.
8. Evaluate obstacle conditions.
9. Select the appropriate rover movement state.
10. Repeat scanning and navigation.

### 12. Simulation Environment

The complete rover circuit is designed using Tinkercad Circuits.

The simulation includes:

- Arduino Uno
- L293D motor driver
- Two DC motors
- HC-SR04 ultrasonic sensor
- Servo motor
- Breadboard and virtual power connections

**Project Type:** Virtual Circuit Simulation

No physical hardware implementation is claimed.

### 13. Conclusion

The Arduino firmware integrates motor control, ultrasonic scanning, obstacle avoidance, coordinate generation, and occupancy grid updates into a single autonomous rover application.

The use of a state-based control architecture helps organize the rover's scanning and movement operations.

The generated coordinate information is transferred through serial communication and processed using Python to visualize the scanned environment in two dimensions.

**project lead**
**Poojaa D V**
**project members**
**Beeram Yeswanth Reddy
  Aritra Bhunia
  Deshpande Akash**
