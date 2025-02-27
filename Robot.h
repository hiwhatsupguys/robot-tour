#pragma once
#include "Motor.h"

class Robot {
public:
  Robot(Motor leftMotor, Motor rightMotor, float initX, float initY, float initHeading);
  float getX();
  float getY();
  float getHeading();
  void turn(float degrees);
  void forward(float speed);
  void calculateAndUpdatePose();
  Motor rightMotor;
  Motor leftMotor;


private:
  // counts for revolution for the encoder
  static const int COUNTS_PER_REVOLUTION = 12*50; // CPR * gear ratio
  // wheel radius in cm
  static const float WHEEL_RADIUS = 2;
  float x;
  float y;
  float heading;

  float initX;
  float initY;
  float initHeading;

};