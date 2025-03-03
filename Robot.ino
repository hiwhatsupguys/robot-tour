#include "Robot.h"

Robot::Robot(Motor leftMotor, Motor rightMotor, float initX, float initY, float initHeading)
  : leftMotor(leftMotor), rightMotor(rightMotor) {

    x = initX;
    y = initY;
    heading = initHeading;
    

}

// velocity might be negative?
void  Robot::turn(float speed) {
  rightMotor.setVelocity(speed);
  leftMotor.setVelocity(-speed);
}

void Robot::forward(float speed) {
  rightMotor.setVelocity(speed);
  leftMotor.setVelocity(speed);
}

// TODO: UNFINISHED
void Robot::turnRadians(float angle) {

}

// TODO:UNFINISHED
void Robot::forwardCm(float distance) {
  float endPosition = x + distance;
  float errorPrior = 0;
  float integralPrior = 0;
  float bias = 0;

  // x within bounds
  while (x < endPosition + AXIAL_TOLERANCE && x > endPosition - AXIAL_TOLERANCE) {
    float error = endPosition - x;
    // float integral = integralPrior + error * 
  }
}

// https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/
void Robot::calculateAndUpdatePose() {
    long leftMotorPosition = leftMotor.getPosition();
    long rightMotorPosition = rightMotor.getPosition();

    long leftNumberOfRevolutions = leftMotorPosition/this->COUNTS_PER_REVOLUTION;
    long rightNumberOfRevolutions = rightMotorPosition/this->COUNTS_PER_REVOLUTION;

    float leftDistanceTraveled = leftNumberOfRevolutions * 2 * 3.14159265 * this->WHEEL_RADIUS;
    float rightDistanceTraveled = rightNumberOfRevolutions * 2 * 3.14159265 * this->WHEEL_RADIUS;

    float averageDisplacement = (leftDistanceTraveled + rightDistanceTraveled) / 2; // cm
    float deltaHeading = (leftDistanceTraveled - rightDistanceTraveled) / 2; // radians

    float deltaX = averageDisplacement * cos(deltaHeading);
    float deltaY = averageDisplacement * sin(deltaHeading);

    this->x = this->initX + deltaX;
    this->y = this->initY + deltaY;
    this->heading = this->initHeading + deltaHeading;    
}
