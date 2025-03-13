#include "Motor.h"

Motor::Motor(int encoderPinA, int encoderPinB, int pwmPin, int dirPin, int brakePin) {
  encoder = new Encoder(encoderPinA, encoderPinB);
  this->pwmPin = pwmPin;
  this->dirPin = dirPin;
  this->brakePin = brakePin;
  this->currentSpeed = 0;
  this->lastPosition = 0;
  this->lastTime = 0;
  this->velocity = 0;
}

Motor::~Motor() {
  delete encoder;
}

void Motor::init() {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  
  // Initialize motor to stopped state
  digitalWrite(brakePin, HIGH);  // Brake on
  digitalWrite(dirPin, LOW);
  analogWrite(pwmPin, 0);
  
  resetEncoder();
}

void Motor::setSpeed(int speed) {
  // Constrain speed to valid range
  speed = constrain(speed, -255, 255);
  currentSpeed = speed;
  
  if (speed == 0) {
    // Stop the motor
    digitalWrite(brakePin, HIGH);  // Brake on
    analogWrite(pwmPin, 0);
  } else {
    // Set direction
    digitalWrite(dirPin, speed > 0 ? HIGH : LOW);
    // Release brake
    digitalWrite(brakePin, LOW);
    // Set speed
    analogWrite(pwmPin, abs(speed));
  }
}

void Motor::stop() {
  digitalWrite(brakePin, HIGH);  // Brake on
  analogWrite(pwmPin, 0);
  currentSpeed = 0;
}

long Motor::getEncoderPosition() {
  return encoder->read();
}

void Motor::resetEncoder() {
  encoder->write(0);
  lastPosition = 0;
  lastTime = millis();
  velocity = 0;
}

double Motor::getVelocity() {
  return velocity;
}

void Motor::update() {
  unsigned long currentTime = millis();
  long currentPosition = encoder->read();
  
  // Calculate velocity if enough time has passed
  if (currentTime - lastTime >= 50) {  // Update every 50ms
    // Calculate velocity in encoder counts per second
    velocity = (double)(currentPosition - lastPosition) / ((currentTime - lastTime) / 1000.0);
    
    lastPosition = currentPosition;
    lastTime = currentTime;
  }
}