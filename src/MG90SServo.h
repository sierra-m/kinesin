#ifndef MG90S_SERVO_H
#define MG90S_SERVO_H

#include "Servo.h"

#define MG90S_PULSE_WIDTH_MIN 500
#define MG90S_PULSE_WIDTH_MAX 2500
#define MG90S_INVERTED 0
#define MG90S_FULL_MOVE_DELAY_MS 200

class MG90SServo: public Servo {
  public:
    MG90SServo (uint8_t pin)
    : Servo(MG90S_PULSE_WIDTH_MIN, MG90S_PULSE_WIDTH_MAX, MG90S_FULL_MOVE_DELAY_MS, pin, MG90S_INVERTED) {}
};

#endif
