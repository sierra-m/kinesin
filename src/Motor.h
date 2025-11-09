#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "Arduino.h"
#include "esp32-hal.h"


#define MOTOR_SPEED_MIN -100
#define MOTOR_SPEED_MAX 100
#define MOTOR_PWM_MSEC_MIN 1000
#define MOTOR_PWM_MSEC_MAX 2000
#define MOTOR_PWM_RESOLUTION 14
#define MOTOR_PWM_RES_MAX (1 << MOTOR_PWM_RESOLUTION)
#define MOTOR_PWM_RANGE_MIN ((MOTOR_PWM_MSEC_MIN * MOTOR_PWM_RES_MAX) / 20'000)
#define MOTOR_PWM_RANGE_MAX ((MOTOR_PWM_MSEC_MAX * MOTOR_PWM_RES_MAX) / 20'000)

// Time after speed returns to 0 to disengage waveform
#define MOTOR_DISENG_TIME_MS 100


// Represents a PWM-driven ESC as a uniform motor with a speed
// range of [MOTOR_SPEED_MIN, MOTOR_SPEED_MAX]
class Motor {
  public:
    Motor (uint8_t pin, uint8_t inverted, uint8_t disengageAtZero = 1);
    void setPulseWidth(int widthMs);
    void setSpeed(int speed);
    int getSpeed();
    // Nonblocking disengage handling
    void handleDisengage();
  protected:
    uint8_t pin;
    uint8_t inverted;
    uint8_t disengageAtZero;
    int minSpeed;
    int maxSpeed;
    int currentSpeed;
    int currentDutyCycle;
    unsigned long disengageStartTime;
};

#endif