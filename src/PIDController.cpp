#include "PIDController.h"

PIDController::PIDController(
  int target,
  float pVal,
  float iVal,
  float dVal
) {
  this->target = target;
  this->pVal = pVal;
  this->iVal = iVal;
  this->dVal = dVal;
}

int PIDController::getCorrection(int position) {
  int proportional = position - target;
  int derivative = proportional - lastProportional;
  integral += proportional;

  lastProportional = proportional;

  float error = (proportional * pVal) + (integral * iVal) + (derivative * dVal);
  // Invert error for correctino
  return -round(error);
}