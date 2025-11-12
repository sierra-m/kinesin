#ifndef FEET_H
#define FEET_H

#include "HS311Servo.h"

#define FEET_WALK_SPEED 50

class Feet {
  public:
    HS311Servo *leftFoot;
    HS311Servo *rightFoot;
    
    Feet(HS311Servo *leftFoot, HS311Servo *rightFoot);
    void init ();
    void startWalking();
    void stopWalking();
    void update ();
  protected:
    uint8_t walking;
    uint8_t leftFootUp;
};

#endif
