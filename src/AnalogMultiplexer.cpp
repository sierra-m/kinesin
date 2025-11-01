#include "esp32-hal.h"
#include "AnalogMultiplexer.h"

AnalogMultiplexer::AnalogMultiplexer (
  uint8_t addrPin0,
  uint8_t addrPin1,
  uint8_t addrPin2,
  uint8_t addrPin3,
  uint8_t samplePin
) {
  this->addrPin0 = addrPin0;
  this->addrPin1 = addrPin1;
  this->addrPin2 = addrPin2;
  this->addrPin3 = addrPin3;
  this->samplePin = samplePin;
  pinMode(addrPin0, OUTPUT);
  pinMode(addrPin1, OUTPUT);
  pinMode(addrPin2, OUTPUT);
  pinMode(addrPin3, OUTPUT);
  pinMode(samplePin, INPUT);
}

void AnalogMultiplexer::setChannel (uint8_t channel) {
  digitalWrite(addrPin0, GET_BIT(channel, 0));
  digitalWrite(addrPin1, GET_BIT(channel, 1));
  digitalWrite(addrPin2, GET_BIT(channel, 2));
  digitalWrite(addrPin3, GET_BIT(channel, 3));
}

int AnalogMultiplexer::sample (uint8_t channel) {
  setChannel(channel);
  delayMicroseconds(ANALOG_MULTIPLEXER_PROP_US); // Should account for all propagation delays
  return analogRead(samplePin);
}