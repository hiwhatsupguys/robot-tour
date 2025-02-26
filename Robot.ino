#include "Robot.h"

Robot::Robot(Motor left, Motor right)
  : leftMotor(left), rightMotor(right) {
    
}

float Robot::getX() {
  return x;
}

float Robot::getY() {
  return y;
}

float Robot::getHeading() {
  return heading;
}

float Robot::turnDegrees(float degrees) {
}

float Robot::getSpeed() {
  return speed;
}

void Robot::setSpeed(float speed) {
}

void Robot::goTo(float x, float y, float heading) {
}

void Robot::goTo(float x, float y) {
}