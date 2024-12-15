// SensorArray Class
class SensorArray {
  int sensorPins[5];
  int weights[5] = {-2, -1, 0, 1, 2};
  int sensorValues[5];

  public:
    // Constructor to initialize sensor pins
    SensorArray(int pins[5]) {
      for (int i = 0; i < 5; i++) {
        sensorPins[i] = pins[i];
        pinMode(sensorPins[i], INPUT);
      }
    }

    // Function to read all sensor values
    void readSensors() {
      for (int i = 0; i < 5; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
      }
    }

    // Function to calculate the error based on sensor readings
    float calculateError() {
      int sumWeights = 0;
      int sumSensors = 0;

      // Calculate weighted average of sensor readings
      for (int i = 0; i < 5; i++) {
        if (sensorValues[i] == 1) {  // Line detected on sensor
          sumWeights += weights[i];
          sumSensors++;
        }
      }

      // If no sensor detects the line, return previous error (default behavior)
      if (sumSensors == 0) {
        return 0;
      }

      return (float)sumWeights / sumSensors;
    }
};

// PIDController Class
class PIDController {
  float Kp, Ki, Kd;
  float previousError, integralSum;

  public:
    // Constructor to initialize PID constants
    PIDController(float p, float i, float d) : Kp(p), Ki(i), Kd(d), previousError(0), integralSum(0) {}

    // Function to calculate PID correction
    float calculateCorrection(float error) {
      float derivative = error - previousError;
      integralSum += error;

      float correction = Kp * error + Ki * integralSum + Kd * derivative;
      previousError = error;
      return correction;
    }

    // Function to reset PID variables (useful during sharp turns)
    void resetPID() {
      integralSum = 0;
      previousError = 0;
    }
};

// MotorController Class (with 4 motor pins for direction control)
class MotorController {
  int leftMotorPin1, leftMotorPin2;  // Pins for left motor direction control
  int rightMotorPin1, rightMotorPin2;  // Pins for right motor direction control
  int baseSpeed;

  public:
    // Constructor to initialize motor pins and base speed
    MotorController(int lPin1, int lPin2, int rPin1, int rPin2, int speed) 
      : leftMotorPin1(lPin1), leftMotorPin2(lPin2), rightMotorPin1(rPin1), rightMotorPin2(rPin2), baseSpeed(speed) {
      pinMode(leftMotorPin1, OUTPUT);
      pinMode(leftMotorPin2, OUTPUT);
      pinMode(rightMotorPin1, OUTPUT);
      pinMode(rightMotorPin2, OUTPUT);
    }

    // Function to set motor speeds and direction based on PID correction
    void adjustMotors(float correction) {
      int leftSpeed = baseSpeed + correction;
      int rightSpeed = baseSpeed - correction;

      // Constrain motor speeds within allowed range
      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);

      // Set motor direction and speed based on correction
      setMotorDirection(leftMotorPin1, leftMotorPin2, leftSpeed);
      setMotorDirection(rightMotorPin1, rightMotorPin2, rightSpeed);
    }

    // Function to stop motors (useful when the robot detects a stop condition)
    void stopMotors() {
      digitalWrite(leftMotorPin1, LOW);
      digitalWrite(leftMotorPin2, LOW);
      digitalWrite(rightMotorPin1, LOW);
      digitalWrite(rightMotorPin2, LOW);
    }

    // Helper function to set motor direction and speed
    void setMotorDirection(int pin1, int pin2, int speed) {
      if (speed > 0) {
        analogWrite(pin1, speed);
        digitalWrite(pin2, LOW);
      } else {
        analogWrite(pin2, -speed);
        digitalWrite(pin1, LOW);
      }
    }
};

// LineFollowerRobot Class
class LineFollowerRobot {
  SensorArray sensorArray;
  PIDController pidController;
  MotorController motorController;

  public:
    // Constructor to initialize sensors, PID controller, and motor controller
    LineFollowerRobot(SensorArray sensors, PIDController pid, MotorController motors)
      : sensorArray(sensors), pidController(pid), motorController(motors) {}

    // Main function to follow the line
    void followLine() {
      sensorArray.readSensors();
      float error = sensorArray.calculateError();
      float correction = pidController.calculateCorrection(error);
      motorController.adjustMotors(correction);
    }
};

// Define sensor pins (example pins)
int sensorPins[5] = {2, 3, 4, 5, 6};  // L1, L, M, R, R2

// Define motor control pins for left and right motors (directional control)
int leftMotorPin1 = 9;  // Pin for left motor +ve
int leftMotorPin2 = 10;  // Pin for left motor -ve
int rightMotorPin1 = 11;  // Pin for right motor +ve
int rightMotorPin2 = 12;  // Pin for right motor -ve

// Define PID constants
float Kp = 15.0;
float Ki = 0.0;
float Kd = 7.0;

// Create instances of the classes
SensorArray sensorArray(sensorPins);
PIDController pidController(Kp, Ki, Kd);
MotorController motorController(leftMotorPin1, leftMotorPin2, rightMotorPin1, rightMotorPin2, 150);  // Base speed = 150

// Create instance of the LineFollowerRobot
LineFollowerRobot lineFollower(sensorArray, pidController, motorController);

void setup() {
  // Initialization of components is done in class constructors
}

void loop() {
  // Continuously follow the line
  lineFollower.followLine();
}
