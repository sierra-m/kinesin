#include "esp32-hal.h"
#include "esp32-hal-gpio.h"
#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <stdint.h>
#include "Arduino.h"
//#include "FunctionalInterrupt.h"

// Timeout in microseconds
#define ULTRASONIC_TIMEOUT_MICROS 25000

enum UltrasonicPingState {
  PingInactive,
  PingTriggered,
  PingEchoed
};

// Defines a class to control an HC SR04 ultrasonic sensor using
// interrupts. The order of operations is such:
// 1. From the main loop, the user calls trigger(), which will send a
//    10us trigger pulse
// 2. While waiting to receive the pulse, the main loop may perform other
//    tasks, periodically calling hasUpdate() followed by checkTimedOut()
//    for pulse status and timeout management, respectively
// 3. Echo pulse start and end will trigger interrupt, which will store
//    relavent statuses
// 4. When hasUpdate() is asserted, the user fetches the distance (in cm)
//    via the getDistanceCm() method. At this point, the cycle may repeat,
//    but be sure to wait the recommend polling period
// 5. If checkTimedOut() asserts, this indicates no measurement will
//    arrive. The user should then wait for the remaining timeout of the 
//    device before polling it again.
class UltrasonicSensor {
  public:
    UltrasonicSensor (uint8_t triggerPin, uint8_t echoPin);
    void trigger ();
    uint8_t hasUpdate ();
    uint8_t checkTimedOut ();
    float getDistanceCm ();
    void ARDUINO_ISR_ATTR sensorISR ();
    static void registerISR (UltrasonicSensor *sensor, void (*callback)()) {
      attachInterrupt(digitalPinToInterrupt(sensor->echoPin), callback, CHANGE);
    }
  protected:
    volatile uint8_t triggerPin;
    volatile uint8_t echoPin;
    volatile unsigned long startTimeMicros;
    volatile unsigned long endTimeMicros;
    volatile UltrasonicPingState pingState;
};

UltrasonicSensor::UltrasonicSensor (uint8_t triggerPin, uint8_t echoPin) {
  this->triggerPin = triggerPin;
  this->echoPin = echoPin;
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pingState = PingInactive;
}

void UltrasonicSensor::trigger () {
  // Send 10us trigger pulse
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pingState = PingTriggered;
}

uint8_t UltrasonicSensor::hasUpdate () {
  return pingState == PingEchoed;
}

uint8_t UltrasonicSensor::checkTimedOut () {
  if (pingState == PingTriggered) {
    if ((micros() - startTimeMicros) > ULTRASONIC_TIMEOUT_MICROS) {
      pingState = PingInactive;
      return 1;
    }
  }
  return 0;
}

float UltrasonicSensor::getDistanceCm () {
  if (pingState == PingEchoed) {
    pingState = PingInactive;
    unsigned long duration = endTimeMicros - startTimeMicros;
    return duration * 0.0343 / 2;
  }
}

void ARDUINO_ISR_ATTR UltrasonicSensor::sensorISR () {
  if (pingState == PingTriggered) {
    if (digitalRead(echoPin) == HIGH) {
      startTimeMicros = micros();
    } else {
      endTimeMicros = micros();
      pingState = PingEchoed;
    }
  }
}

#endif