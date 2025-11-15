#include "Feet.h"


Feet::Feet (MG90SServo *leftFoot, MG90SServo *rightFoot) {
  this->leftFoot = leftFoot;
  this->rightFoot = rightFoot;
}

void Feet::init () {
  leftFoot->setSpeed(FEET_WALK_SPEED);
  rightFoot->setSpeed(FEET_WALK_SPEED);
  stopWalking();
}

void Feet::setLeft(uint8_t up) {
  if (up) {
    leftFoot->setPos(1000);
  } else {
    leftFoot->setPos(500);
  }
}

void Feet::setRight(uint8_t up) {
  if (up) {
    rightFoot->setPos(0);
  } else {
    rightFoot->setPos(500);
  }
}

void Feet::startWalking() {
  walking = true;
  leftFootUp = true;
  setLeft(FOOT_UP);
  setRight(FOOT_DOWN);
}

void Feet::stopWalking() {
  walking = false;
  setLeft(FOOT_DOWN);
  setRight(FOOT_DOWN);
}

void Feet::update () {
  if (walking) {
    // Invert when they both stop moving
    if (! (leftFoot->requiresUpdate() || rightFoot->requiresUpdate()) ) {
      leftFootUp = !leftFootUp;
      if (leftFootUp) {
        setLeft(FOOT_UP);
        setRight(FOOT_DOWN);
      } else {
        setLeft(FOOT_DOWN);
        setRight(FOOT_UP);
      }
    }
  }
}
