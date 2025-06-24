#include "arduino_secrets.h"


#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// Motor pins
const int ENA = 3, IN1 = 4, IN2 = 5;
const int ENB = 6, IN3 = 7, IN4 = 8;

// MPU6050 interrupt pin
const int INT_PIN = 2;

// PID variables
double input, output, setpoint = 180; // target angle = 180Â°
double Kp = 25.0, Ki = 80.0, Kd = 1.2;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Fall detection threshold (degrees)
const double fallLimit = 15;

// MPU6050 object and DMP state
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
volatile bool mpuInterrupt = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  // Initialize serial, I2C, and motors
  Serial.begin(115200);
  Wire.begin();

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  stopMotors();

  // Initialize MPU6050
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Load calibration offsets from your calibration run
  mpu.setXGyroOffset(195);
  mpu.setYGyroOffset(12);
  mpu.setZGyroOffset(-13);
  mpu.setZAccelOffset(1073);

  if (devStatus == 0) {
    // Success: enable DMP, attach interrupt, start PID
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);  // 10ms sample time for ~100Hz control loop
    pid.SetOutputLimits(-255, 255); // Output power range
    Serial.println("DMP ready, PID enabled");
  } else {
    Serial.print("DMP init failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    while (1); // Halt if init failed
  }
}

void loop() {
  if (!dmpReady) return;

  // Wait for DMP data interrupt
  if (!mpuInterrupt) return;
  mpuInterrupt = false;

  // Read full FIFO packet
  if (mpu.getFIFOCount() >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // Get angle data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Compute the tilt angle (pitch)
    input = ypr[1] * 180/M_PI; // degrees

    // If robot falls beyond threshold, stop motors
    if (abs(input - setpoint) > fallLimit) {
      stopMotors();
      Serial.println("Robot fell! Stopping.");
      while (1); // Halt further operation
    }

    // Compute PID output
    pid.Compute();

    // Control motors based on PID output
    if (output > 0) {
      drive(output, true);
    } else {
      drive(-output, false);
    }

    // Debugging: show angle and control output
    Serial.print("Angle: "); Serial.print(input);
    Serial.print("  PID out: "); Serial.println(output);
  }
}

// Drive both motors: pwm = speed (0â255), dir = true for forward
void drive(double pwm, bool dir) {
  int speed = constrain(pwm, 0, 255);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);

  digitalWrite(IN1, dir);
  digitalWrite(IN2, !dir);
  digitalWrite(IN3, dir);
  digitalWrite(IN4, !dir);
}

// Stop both motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
