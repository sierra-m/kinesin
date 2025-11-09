#ifndef HS311_SERVO_H
#define HS311_SERVO_H

#include "Servo.h"

#define HS311_PULSE_WIDTH_MIN 1000
#define HS311_PULSE_WIDTH_MAX 2000
#define HS311_INVERTED 1
#define HS311_FULL_MOVE_DELAY_MS 1000

class HS311Servo: public Servo {
  public:
    HS311Servo (uint8_t pin)
    : Servo(HS311_PULSE_WIDTH_MIN, HS311_PULSE_WIDTH_MAX, HS311_FULL_MOVE_DELAY_MS, pin, HS311_INVERTED) {}
};

#endif