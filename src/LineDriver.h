#ifndef LINE_DRIVER_H
#define LINE_DRIVIR_H

#include "Arduino.h"
#include "LineSensorArray.h"
#include "AnalogMultiplexer.h"
#include "PIDController.h"
#include "Motor.h"

#define IR_SENSOR_COUNT 6
#define IR_SENSOR_CHAN_START 5  // 5, 6, 7, 8, 9, 10

#define CALIBRATE_SPEED 20
#define CALIBRATE_DELAY_MSEC 1000

// Number of sensors used can change, so normalize the calculated
// position to a standard range. Ranges always start at 0.
#define POS_RANGE_MAX ((IR_SENSOR_COUNT - 1) * 1000)
#define NORM_RANGE_MAX 5000

#define DEFAULT_DRIVE_SPEED 20 

class LineDriver {
  public:
    LineSensorArray<IR_SENSOR_COUNT> lineSensorArray;
    AnalogMultiplexer *analogMplex;
    PIDController *pidController;
    Motor *leftMotor;
    Motor *rightMotor;

    LineDriver (
      AnalogMultiplexer *analogMplex,
      PIDController *pidController,
      Motor *leftMotor,
      Motor *rightMotor
    );
    // Blocking calibration routine
    void calibrate();
    // Controlling drive method to be called continuously
    void drive ();
    // Set the max speed
    void setSpeed(int speed);
    // Get the max speed
    int getSpeed();
    // Stop driving
    void stop ();
  protected:
    int speed;
    int calcLinePosition ();
    void calibrateDelay (int timeMsec);
};

#endif
