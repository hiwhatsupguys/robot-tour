#pragma once
#include "Motor.h"


class Robot {
public:
  Robot(Motor leftMotor, Motor rightMotor, float initX, float initY, float initHeading);
  float getX();
  float getY();
  float getHeading();
  void turn(float speed);
  void forward(float speed);
  void turnRadians(float angle);
  // distance in cm
  void forwardCm(float distance);
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
  // cm
  static const float AXIAL_TOLERANCE = 10;
  // radians
  static const float HEADING_TOLERANCE = 10*(3.14159265/180);

  static const float KP = 0;
  static const float KI = 0;
  static const float KD = 0;


  float initX;
  float initY;
  float initHeading;

};