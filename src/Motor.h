#pragma once
#include <Encoder.h>

class Motor {
public:
  Motor(int encoderPinA, int encoderPinB, int pwmPin, int directionPin, int brakePin);
  double getVelocity();
  void setVelocity(double velocity);
  int getPosition();
  void updatePosition();



private:
  Encoder encoder;
  double velocity = 0;
  int PWM_PIN;
  int DIRECTION_PIN;
  int BRAKE_PIN;


};