#include "Feet.h"


Feet::Feet (HS311Servo *leftFoot, HS311Servo *rightFoot) {
  this->leftFoot = leftFoot;
  this->rightFoot = rightFoot;
}

void Feet::init () {
  leftFoot->setPos(0);
  rightFoot->setPos(0);
}

void Feet::startWalking() {
  walking = true;
  leftFootUp = true;
  leftFoot->setPos(1000);
  rightFoot->setPos(0);
}

void Feet::stopWalking() {
  walking = false;
  init();
}

void Feet::update () {
  if (walking) {
    if (! (leftFoot->requiresUpdate() || rightFoot->requiresUpdate()) ) {
      leftFootUp = !leftFootUp;
      if (leftFootUp) {
        leftFoot->setPos(1000);
        rightFoot->setPos(0);
      } else {
        leftFoot->setPos(0);
        rightFoot->setPos(1000);
      }
    }
  }
}
