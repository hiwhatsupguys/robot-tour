#pragma once
#include <Encoder.h>

class Motor {
public:
  Motor(int encoderPinA, int encoderPinB, int pwmPin, int directionPin, int brakePin);
  float getVelocity();
  void setVelocity(float velocity);
  int getPosition();
  void updatePosition();


private:
  Encoder encoder;
  float velocity = 0;
  int PWM_PIN;
  int DIRECTION_PIN;
  int BRAKE_PIN;

};