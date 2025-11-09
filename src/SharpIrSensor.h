#ifndef SHARP_IR_SENSOR
#define SHARP_IR_SENSOR

#include "Arduino.h"

class SharpIrSensor {
  public:
    uint8_t pin;
    SharpIrSensor (uint8_t pin);
    float getDistanceCm ();
};

#endif