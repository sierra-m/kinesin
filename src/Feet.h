#ifndef FEET_H
#define FEET_H

#include "MG90SServo.h"

#define FEET_WALK_SPEED 90

#define FOOT_UP 1
#define FOOT_DOWN 0

#define FEET_WALK_SPEED 50

class Feet {
  public:
    MG90SServo *leftFoot;
    MG90SServo *rightFoot;
    
    Feet(MG90SServo *leftFoot, MG90SServo *rightFoot);
    void init ();
    void setLeft(uint8_t up);
    void setRight(uint8_t up);
    void startWalking();
    void stopWalking();
    void update ();
  protected:
    uint8_t walking;
    uint8_t leftFootUp;
};

#endif
