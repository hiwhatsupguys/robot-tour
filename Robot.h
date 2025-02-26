#pragma once
#include "Motor.h"

class Robot {
public:
  Robot(Motor leftMotor, Motor rightMotor);
  float getX();
  float getY();
  float getHeading();
  float turnDegrees(float degrees);
  float getSpeed();
  void setSpeed(float speed);
  void goTo(float x, float y, float heading);
  void goTo(float x, float y);


private:
  Motor rightMotor;
  Motor leftMotor;
  float x;
  float y;
  float heading;
  float speed;
};