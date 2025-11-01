#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Arduino.h"

#define PID_INPUT_POS_MIN 0
#define PID_INPUT_POS_MAX 5000

class PIDController {
  public:
    PIDController(
      int target,
      float pVal,
      float iVal,
      float dVal
    );
    int getCorrection(int position);
  protected:
    float pVal;
    float iVal;
    float dVal;
    int target;
    int lastProportional;
    int integral;
};

#endif