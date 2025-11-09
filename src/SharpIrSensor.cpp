#include "SharpIrSensor.h"

SharpIrSensor::SharpIrSensor (uint8_t pin) {
  this->pin = pin;
  //pinMode(pin, INPUT);
}

float SharpIrSensor::getDistanceCm () {
  float volts = analogRead(pin) * 3.3 / 4096;
  return 29.988 * pow(volts, -1.173);
}