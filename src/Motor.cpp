#include "Motor.h"

Motor::Motor (uint8_t pin, uint8_t inverted, uint8_t disengageAtZero) {
  this->pin = pin;
  this->inverted = inverted;
  this->disengageAtZero = disengageAtZero;
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

void Motor::setPulseWidth (int widthMs) {
  currentDutyCycle = map(widthMs, MOTOR_PWM_MSEC_MIN, MOTOR_PWM_MSEC_MAX, MOTOR_PWM_RANGE_MIN, MOTOR_PWM_RANGE_MAX);
  ledcWrite(pin, currentDutyCycle);
}

void Motor::setSpeed (int speed) {
  currentSpeed = speed;
  currentDutyCycle = map(currentSpeed, minSpeed, maxSpeed, MOTOR_PWM_RANGE_MIN, MOTOR_PWM_RANGE_MAX);
  // Set PWM
  ledcWrite(pin, currentDutyCycle);
  // If set to disengage, bring to middle first to halt motors, wait a short time,
  // then disengage waveform
  if (speed == 0 && disengageAtZero) {
    // 10 cycles of pwm
    disengageStartTime = millis();
  }
}

int Motor::getSpeed () {
  return currentSpeed;
}

void Motor::handleDisengage () {
  if (currentDutyCycle != 0 && currentSpeed == 0 && disengageAtZero) {
    if ((millis() - disengageStartTime) > MOTOR_DISENG_TIME_MS) {
      ledcWrite(pin, 0);
      currentDutyCycle = 0;
    }
  }
}