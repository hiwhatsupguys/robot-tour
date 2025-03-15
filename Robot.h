#pragma once
#include "Motor.h"


class Robot {
public:
  Robot(Motor leftMotor, Motor rightMotor);
  void turn(double speed);
  void forward(double speed);
  void turnRadians(double angle);
  // distance in cm
  void forwardCm(double distance);
  void forwardSeconds(double time, double speed);
  void calculateAndUpdatePose(double* axialOffset, double* lateralOffset, double* headingOffset);
  Motor rightMotor;
  Motor leftMotor;


private:
  // counts for revolution for the encoder
  static const double COUNTS_PER_REVOLUTION = 12.0*50.0; // CPR * gear ratio
  // wheel radius in cm
  static const double WHEEL_RADIUS = 2.0;
  static const double TRACK_WIDTH = 11.0;
  static const double RIGHT_MOTOR_SCALAR = 0.8;
  double axialOffset;
  double lateralOffset;
  // radians
  double headingOffset;
  // cm
  static const double AXIAL_TOLERANCE = 10.0;
  // radians
  static const double HEADING_TOLERANCE = 10.0*(3.14159265/180.0);

  static const double KP_AXIAL = 0.4;
  static const double KI_AXIAL = 0.0;
  static const double KD_AXIAL = 0.0;

  static const double KP_LATERAL = 0.0;
  static const double KI_LATERAL = 0.0;
  static const double KD_LATERAL = 0.0;

};