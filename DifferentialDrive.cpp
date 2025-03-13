#include "DifferentialDrive.h"
#include <math.h>

DifferentialDrive::DifferentialDrive(double countsPerRevolution, double wheelRadius, double trackWidth) {
  this->countsPerRevolution = countsPerRevolution;
  this->wheelRadius = wheelRadius;
  this->trackWidth = trackWidth;
  
  // Initialize position
  x = 0;
  y = 0;
  heading = 0;
  
  // Default PID parameters
  Kp = 2.0;
  Ki = 0.1;
  Kd = 0.5;
}

DifferentialDrive::~DifferentialDrive() {
  delete leftMotor;
  delete rightMotor;
  delete pidX;
  delete pidY;
  delete pidHeading;
}

void DifferentialDrive::init(int leftEncoderPinA, int leftEncoderPinB, int leftPwmPin, int leftDirPin, int leftBrakePin,
                            int rightEncoderPinA, int rightEncoderPinB, int rightPwmPin, int rightDirPin, int rightBrakePin) {
  // Initialize motors
  leftMotor = new Motor(leftEncoderPinA, leftEncoderPinB, leftPwmPin, leftDirPin, leftBrakePin);
  rightMotor = new Motor(rightEncoderPinA, rightEncoderPinB, rightPwmPin, rightDirPin, rightBrakePin);
  
  leftMotor->init();
  rightMotor->init();
  
  // Initialize PID controllers
  pidInputX = 0;
  pidOutputX = 0;
  pidSetpointX = 0;
  
  pidInputY = 0;
  pidOutputY = 0;
  pidSetpointY = 0;
  
  pidInputHeading = 0;
  pidOutputHeading = 0;
  pidSetpointHeading = 0;
  
  pidX = new PID(&pidInputX, &pidOutputX, &pidSetpointX, Kp, Ki, Kd, DIRECT);
  pidY = new PID(&pidInputY, &pidOutputY, &pidSetpointY, Kp, Ki, Kd, DIRECT);
  pidHeading = new PID(&pidInputHeading, &pidOutputHeading, &pidSetpointHeading, Kp, Ki, Kd, DIRECT);
  
  // Configure PIDs
  pidX->SetMode(AUTOMATIC);
  pidY->SetMode(AUTOMATIC);
  pidHeading->SetMode(AUTOMATIC);
  
  pidX->SetOutputLimits(-255, 255);
  pidY->SetOutputLimits(-255, 255);
  pidHeading->SetOutputLimits(-255, 255);
  
  // Reset position tracking
  resetPosition();
}

void DifferentialDrive::setPIDParams(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  pidX->SetTunings(Kp, Ki, Kd);
  pidY->SetTunings(Kp, Ki, Kd);
  pidHeading->SetTunings(Kp, Ki, Kd);
}

double DifferentialDrive::countsToDistance(long counts) {
  // Convert encoder counts to distance in cm
  return (2.0 * PI * wheelRadius * counts) / countsPerRevolution;
}

void DifferentialDrive::updatePosition() {
  // Get encoder positions
  long leftCounts = leftMotor->getEncoderPosition();
  long rightCounts = rightMotor->getEncoderPosition();
  
  // Calculate distances
  double leftDistance = countsToDistance(leftCounts);
  double rightDistance = countsToDistance(rightCounts);
  
  // Calculate heading change
  double deltaHeading = (rightDistance - leftDistance) / trackWidth;
  heading += deltaHeading;
  
  // Normalize heading to [-PI, PI]
  while (heading > PI) heading -= 2 * PI;
  while (heading < -PI) heading += 2 * PI;
  
  // Calculate average distance traveled
  double distance = (leftDistance + rightDistance) / 2.0;
  
  // Update position
  x += distance * cos(heading);
  y += distance * sin(heading);
  
  // Update motor velocities
  leftMotor->update();
  rightMotor->update();
}

void DifferentialDrive::forwardCm(double distance) {
  // Reset encoders
  leftMotor->resetEncoder();
  rightMotor->resetEncoder();
  
  // Set target position
  double targetX = x + distance * cos(heading);
  double targetY = y + distance * sin(heading);
  
  pidSetpointX = targetX;
  pidSetpointY = targetY;
  
  // Reset PIDs
  pidX->SetMode(AUTOMATIC);
  pidY->SetMode(AUTOMATIC);
  
  // Main control loop
  unsigned long lastTime = millis();
  bool reachedTarget = false;
  
  while (!reachedTarget) {
    // Update current position
    updatePosition();
    
    // Update PID inputs
    pidInputX = x;
    pidInputY = y;
    
    // Compute PID outputs
    pidX->Compute();
    pidY->Compute();
    
    // Convert X and Y errors to motor speeds
    double forwardSpeed = pidOutputX * cos(heading) + pidOutputY * sin(heading);
    
    // Determine when to stop
    double distanceToTarget = sqrt(pow(targetX - x, 2) + pow(targetY - y, 2));
    if (distanceToTarget < 0.5) {  // Within 0.5 cm of target
      reachedTarget = true;
    }
    
    // Apply motor speeds
    leftMotor->setSpeed(forwardSpeed);
    rightMotor->setSpeed(forwardSpeed);
    
    // Small delay to prevent hogging CPU
    delay(20);
    
    // Safety timeout (10 seconds)
    if (millis() - lastTime > 10000) {
      break;
    }
  }
  
  // Stop motors
  leftMotor->stop();
  rightMotor->stop();
}

void DifferentialDrive::turnInPlace(double degrees) {
  // Convert degrees to radians
  double targetHeadingChange = degrees * PI / 180.0;
  double targetHeading = heading + targetHeadingChange;
  
  // Normalize target heading to [-PI, PI]
  while (targetHeading > PI) targetHeading -= 2 * PI;
  while (targetHeading < -PI) targetHeading += 2 * PI;
  
  // Reset encoders
  leftMotor->resetEncoder();
  rightMotor->resetEncoder();
  
  // Set PID setpoint
  pidSetpointHeading = targetHeading;
  
  // Reset PIDs
  pidHeading->SetMode(AUTOMATIC);
  
  // Main control loop
  unsigned long lastTime = millis();
  bool reachedTarget = false;
  
  while (!reachedTarget) {
    // Update current position
    updatePosition();
    
    // Update PID input
    pidInputHeading = heading;
    
    // Compute PID output
    pidHeading->Compute();
    
    // Apply differential speeds for rotation
    int baseSpeed = 100;  // Base rotation speed
    double rotationSpeed = pidOutputHeading;
    
    // Set motor speeds for rotation
    if (targetHeadingChange > 0) {
      // Turn clockwise
      leftMotor->setSpeed(baseSpeed);
      rightMotor->setSpeed(-baseSpeed);
    } else {
      // Turn counter-clockwise
      leftMotor->setSpeed(-baseSpeed);
      rightMotor->setSpeed(baseSpeed);
    }
    
    // Determine when to stop
    double headingError = abs(targetHeading - heading);
    if (headingError > PI) {
      headingError = 2 * PI - headingError;  // Take shorter path
    }
    
    if (headingError < 0.05) {  // Within ~3 degrees of target
      reachedTarget = true;
    }
    
    // Small delay to prevent hogging CPU
    delay(20);
    
    // Safety timeout (10 seconds)
    if (millis() - lastTime > 10000) {
      break;
    }
  }
  
  // Stop motors
  leftMotor->stop();
  rightMotor->stop();
}

double DifferentialDrive::getX() {
  return x;
}

double DifferentialDrive::getY() {
  return y;
}

double DifferentialDrive::getHeadingDegrees() {
  return heading * 180.0 / PI;
}

void DifferentialDrive::resetPosition() {
  x = 0;
  y = 0;
  heading = 0;
  
  leftMotor->resetEncoder();
  rightMotor->resetEncoder();
}