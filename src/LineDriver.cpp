#include "LineDriver.h"

LineDriver::LineDriver (
  AnalogMultiplexer *analogMplex,
  PIDController *pidController,
  Motor *leftMotor,
  Motor *rightMotor
) {
  this->analogMplex = analogMplex;
  this->pidController = pidController;
  this->leftMotor = leftMotor;
  this->rightMotor = rightMotor;
  this->speed = DEFAULT_DRIVE_SPEED;
}

void LineDriver::calibrate () {
  // Rotate CCW half a period
  leftMotor->setSpeed(-CALIBRATE_SPEED);
  rightMotor->setSpeed(CALIBRATE_SPEED);
  calibrateDelay(CALIBRATE_DELAY_MSEC / 2);
  // Rotate CW a full period
  leftMotor->setSpeed(CALIBRATE_SPEED);
  rightMotor->setSpeed(-CALIBRATE_SPEED);
  calibrateDelay(CALIBRATE_DELAY_MSEC);
  // Rotate CCW half a period, returning to center
  leftMotor->setSpeed(-CALIBRATE_SPEED);
  rightMotor->setSpeed(CALIBRATE_SPEED);
  calibrateDelay(CALIBRATE_DELAY_MSEC / 2);
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
}

void LineDriver::drive () {
  int position = calcLinePosition();
  // Normalize calculated position to a standard range if needed
  if (POS_RANGE_MAX > NORM_RANGE_MAX) {
    position = map(position, 0, POS_RANGE_MAX, 0, NORM_RANGE_MAX);
  }
  int correction = pidController->getCorrection(position);
  // Trim correction at max speed boundaries
  if (correction > speed) {
    correction = speed;
  } else if (correction < -speed) {
    correction = -speed;
  }
  // Negative -> Turn left
  // Positive -> Turn right
  int leftSpeed;
  int rightSpeed;
  if (correction < 0) {
    leftSpeed = speed + correction;
    rightSpeed = speed;
  } else {
    leftSpeed = speed;
    rightSpeed = speed - correction;
  }
  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);
  // Serial.print("Left: ");
  // Serial.print(leftSpeed);
  // Serial.print(", Right: ");
  // Serial.println(rightSpeed);
}

void LineDriver::setSpeed (int speed) {
  this->speed = speed;
}

int LineDriver::getSpeed () {
  return speed;
}

void LineDriver::stop () {
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
}

int LineDriver::calcLinePosition () {
  int samples [IR_SENSOR_COUNT];
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    samples[i] = analogMplex->sample(IR_SENSOR_CHAN_START + i);
  }
  return lineSensorArray.calcPosition(samples);
}

void LineDriver::calibrateDelay (int timeMsec) {
  for (int i = 0; i < timeMsec; i++) {
    // Sample from sensor array to update windows
    (void)calcLinePosition();
    delay(1);
  }
}