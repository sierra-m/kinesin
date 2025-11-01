#ifndef ANALOG_MULTIPLEXER_H
#define ANALOG_MULTIPLEXER_H

#include <stdint.h>
#include "Arduino.h"


#define GET_BIT(VAL, POS) ((VAL >> POS) & 1)

// Propagation delay in microseconds
#define ANALOG_MULTIPLEXER_PROP_US 1

class AnalogMultiplexer {
  public:
    AnalogMultiplexer (
      uint8_t addrPin0,
      uint8_t addrPin1,
      uint8_t addrPin2,
      uint8_t addrPin3,
      uint8_t samplePin
    );
    int sample (uint8_t channel);
    void setChannel (uint8_t channel);
  protected:
    uint8_t addrPin0, addrPin1, addrPin2, addrPin3;
    uint8_t samplePin;
};

#endif