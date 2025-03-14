#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <Arduino.h>
#include <PID_v1.h>
#include "Motor.h"

class DifferentialDrive {
  private:
    Motor* leftMotor;
    Motor* rightMotor;
    
    double countsPerRevolution;
    double wheelRadius;
    double trackWidth;
    
    // Robot position
    double x;
    double y;
    double heading;  // in radians
    
    // PID controllers
    PID* pidX;
    PID* pidY;
    PID* pidHeading;
    
    // PID variables
    double pidInputX, pidOutputX, pidSetpointX;
    double pidInputY, pidOutputY, pidSetpointY;
    double pidInputHeading, pidOutputHeading, pidSetpointHeading;
    
    // PID parameters
    double Kp, Ki, Kd;
    
    // Convert encoder counts to distance
    double countsToDistance(long counts);
    
    // Update position based on encoder readings
    void updatePosition();
    
  public:
    DifferentialDrive(double countsPerRevolution, double wheelRadius, double trackWidth);
    ~DifferentialDrive();
    
    void init(int leftEncoderPinA, int leftEncoderPinB, int leftPwmPin, int leftDirPin, int leftBrakePin,
              int rightEncoderPinA, int rightEncoderPinB, int rightPwmPin, int rightDirPin, int rightBrakePin);
    
    void setPIDParams(double Kp, double Ki, double Kd);
    
    // Main control functions
    void forwardCm(double distance);
    void turnInPlace(double degrees);
    
    // Get current position and heading
    double getX();
    double getY();
    double getHeadingDegrees();
    
    // Reset position
    void resetPosition();
};

#endif