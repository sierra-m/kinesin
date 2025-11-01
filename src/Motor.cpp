#include "Motor.h"

Motor::Motor (uint8_t pin, uint8_t inverted) {
  this->pin = pin;
  this->inverted = inverted;
  currentSpeed = 0;
  if (inverted) {
    minSpeed = MOTOR_SPEED_MAX;
    maxSpeed = MOTOR_SPEED_MIN;
  } else {
    minSpeed = MOTOR_SPEED_MIN;
    maxSpeed = MOTOR_SPEED_MAX;
  }
  // Set up 50Hz PWM wave and attach to GPIO
  ledcAttach(pin, 50, MOTOR_PWM_RESOLUTION);
}

void Motor::setPulseWidth(int widthMs) {
  int dutyCycle = map(widthMs, MOTOR_PWM_MSEC_MIN, MOTOR_PWM_MSEC_MAX, MOTOR_PWM_RANGE_MIN, MOTOR_PWM_RANGE_MAX);
  ledcWrite(pin, dutyCycle);
}

void Motor::setSpeed(int speed) {
  this->currentSpeed = speed;
  int dutyCycle = map(currentSpeed, minSpeed, maxSpeed, MOTOR_PWM_RANGE_MIN, MOTOR_PWM_RANGE_MAX);
  // Set PWM
  ledcWrite(pin, dutyCycle);
}

int Motor::getSpeed() {
  return currentSpeed;
}