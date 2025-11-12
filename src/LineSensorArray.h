#ifndef LINE_SENSOR_ARRAY_H
#define LINE_SENSOR_ARRAY_H

#include <stdint.h>
#include "Arduino.h"

#define ADC_MIN_VALUE 0
#define ADC_MAX_VALUE 4096

#define CHANNEL_MIN_INIT ADC_MAX_VALUE
#define CHANNEL_MAX_INIT ADC_MIN_VALUE

struct IrChannelLimits {
  int minValue;
  int maxValue;
};

template <int Size>
class LineSensorArray {
  public:
    uint8_t trackBlack;
    LineSensorArray(uint8_t trackBlack = 1);
    int calcPosition(int *samples);

    IrChannelLimits channelLimits[Size];
    int normalize(int channel, int sample);
    int calcWeightedAvg(int *normalized);
};

// Implementations

template <int Size>
LineSensorArray<Size>::LineSensorArray(uint8_t trackBlack) {
  this->trackBlack = trackBlack;
  for (int i = 0; i < Size; i++) {
    channelLimits[i].minValue = CHANNEL_MIN_INIT;
    channelLimits[i].maxValue = CHANNEL_MAX_INIT;
  }
}

template <int Size>
int LineSensorArray<Size>::calcPosition(int *samples) {
  int normalized[Size];
  for (int i = 0; i < Size; i++) {
    if (samples[i] < channelLimits[i].minValue) {
      channelLimits[i].minValue = samples[i];
    }
    if (samples[i] > channelLimits[i].maxValue) {
      channelLimits[i].maxValue = samples[i];
    }
    normalized[i] = normalize(i, samples[i]);
    // If tracking white, invert normalized value
    if (!trackBlack) {
      normalized[i] = 1000 - normalized[i];
    }
  }
  return calcWeightedAvg(normalized);
}

template <int Size>
int LineSensorArray<Size>::normalize(int channel, int sample) {
  IrChannelLimits limits = channelLimits[channel];
  int diff = limits.maxValue - limits.minValue;
  if (diff == 0) return 0;
  return (sample - limits.minValue) * 1000 / (diff);
}

template <int Size>
int LineSensorArray<Size>::calcWeightedAvg(int *normalized) {
  int dividend = 0;
  int divisor = 0;
  for (int i = 0; i < Size; i++) {
    int weight = (i * 1000 * normalized[i]);
    dividend += weight;
    divisor += normalized[i];
  }
  if (divisor == 0) {
    return 0;
  }
  return dividend / divisor;
}

#endif