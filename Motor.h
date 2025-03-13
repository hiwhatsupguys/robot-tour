#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Encoder.h>

class Motor {
  private:
    Encoder* encoder;
    int pwmPin;
    int dirPin;
    int brakePin;
    int currentSpeed;
    long lastPosition;
    unsigned long lastTime;
    double velocity;
    
  public:
    Motor(int encoderPinA, int encoderPinB, int pwmPin, int dirPin, int brakePin);
    ~Motor();
    
    void init();
    void setSpeed(int speed);  // -255 to 255
    void stop();
    long getEncoderPosition();
    void resetEncoder();
    double getVelocity();  // in encoder counts per second
    void update();  // Update velocity calculation
};

#endif
