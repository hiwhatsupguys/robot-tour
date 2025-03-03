#pragma once
#include <Encoder.h>

class Motor {
public:
  Motor(int encoderPinA, int encoderPinB, int pwmPin, int directionPin, int brakePin, bool isReversed);
  float getVelocity();
  void setVelocity(float velocity);
  int getPosition();
  void updatePosition();
  void setReversed(bool isReversed);



private:
  Encoder encoder;
  float velocity = 0;
  int PWM_PIN;
  int DIRECTION_PIN;
  int BRAKE_PIN;
  bool isReversed;


};