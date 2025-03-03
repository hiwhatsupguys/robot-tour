#include "Motor.h"
#include <Encoder.h>

Motor::Motor(int pinA, int pinB, int pwmPin, int directionPin, int brakePin, bool isReversed)
  : encoder(pinA, pinB) {

  PWM_PIN = pwmPin;
  DIRECTION_PIN = directionPin;
  BRAKE_PIN = brakePin;
  this->isReversed = isReversed;


  pinMode(DIRECTION_PIN, OUTPUT);  // motor pin
  pinMode(BRAKE_PIN, OUTPUT);      // brake pin
}


float Motor::getVelocity() {
  return velocity;
}

void Motor::setVelocity(float velocity) {
  // take the absolute value of velocity and clamp to 0 - 255
  int speed = constrain((int)((abs(velocity) * 255)), 0, 255);

  // return the sign of velocity (true if positive, false if negative)
  bool isClockwise = (velocity >= 0);
  if (isReversed) isClockwise = !isClockwise;
  bool enableBrake = (velocity == 0);

  digitalWrite(DIRECTION_PIN, isClockwise);  //backwards (counter clockwise)
  digitalWrite(BRAKE_PIN, enableBrake);      //Engage/Disengage the Brake for Channel A
  analogWrite(PWM_PIN, speed);               //Spins the motor on Channel A

  this->velocity = velocity;
}

void Motor::setReversed(bool isReversed) {
  this->isReversed = isReversed;
}

int Motor::getPosition() {
  return encoder.read();
}
