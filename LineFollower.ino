#include <QTRSensors.h>
const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;
int m1Speed = 0;
int m2Speed = 0;

float kp = 8;
float ki = 0;
float kd = 20;

int p = 1;
int i = 0;
int d = 0;
int error = 0;
int lastError = 0;
const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 200;
QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];

int calibrationTime = 4800;
unsigned long lastCalibration = 0;
int oneCalibrationTime = 500;
bool isCalibrating = 1;
int calibrationIndex = 0;
int noOfCalibrations = 6;
int singleCalibrationTime = 1000;
int calibrationSpeed = 150;

unsigned long errorIndex = 0;

void setup() {
  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);

  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  Serial.begin(9600);

  digitalWrite(LED_BUILTIN, LOW);
}

void calibrate() {

  if (lastCalibration == 0) {
    lastCalibration = millis();
  }


  if (millis() - lastCalibration >= oneCalibrationTime) {
    lastCalibration = millis();
    calibrationIndex++;

    //when calibration starts and when it ends it does only a half of a rotation, therefore we split in half the time
    if (calibrationIndex == noOfCalibrations - 1) {
      oneCalibrationTime = singleCalibrationTime / 2;
    } else {
      oneCalibrationTime = singleCalibrationTime;
    }

  }

  else {
    qtr.calibrate();
    if (calibrationIndex % 2 == 1) {
      //one time it goes forward and the other backward
      setMotorSpeed(-calibrationSpeed, 0);
    } else {
      setMotorSpeed(calibrationSpeed, 0);
    }
  }
}


void loop() {

  if (isCalibrating) {
    calibrate();
    if (millis() - lastCalibration > calibrationTime) {
      isCalibrating = false;
    }
  }

  else {
    computePID();
    computeMotorSpeed();
  }
}


void computePID() {

  errorIndex++;

  int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);
  p = error;
  i = i + error;
  d = error - lastError;

  //we reset the last error every 10 interations in order to accumulate a more significant rate of change
  if (errorIndex % 10 == 0) {
    lastError = error;
  }
}

void computeMotorSpeed() {

  int motorSpeed = kp * p + ki * i + kd * d;
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  if (error < 0) {
    m1Speed += motorSpeed;
  } else if (error > 0) {
    m2Speed -= motorSpeed;
  }

  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);
  setMotorSpeed(m1Speed, m2Speed);
}

// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {

  motor2Speed = -motor2Speed;
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  } else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  } else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    }
  }
}
