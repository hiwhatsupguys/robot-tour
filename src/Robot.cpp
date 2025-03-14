#include "Robot.h"
#include <PID_v1.h>


Robot::Robot(Motor leftMotor, Motor rightMotor)
  : leftMotor(leftMotor), rightMotor(rightMotor) {
}

// turn counter clockwise
void Robot::turn(double speed) {
  rightMotor.setVelocity(-speed);
  leftMotor.setVelocity(-speed);
}

// TODO: UNFINISHED
void Robot::turnRadians(double angle) {

}

void Robot::forward(double speed) {
  rightMotor.setVelocity(-RIGHT_MOTOR_SCALAR*speed);
  leftMotor.setVelocity(speed);
}

void Robot::forwardSeconds(double time, double speed) {
  double startTime = millis();
  while ((millis() - startTime) < time*1000) {
    forward(speed);
  }
  forward(0);
}

void Robot::turnLeft(double speed) {
  rightMotor.setVelocity(RIGHT_MOTOR_SCALAR * speed);
  leftMotor.setVelocity(-speed);
}

void Robot::turnRight(double speed) {
  rightMotor.setVelocity(-RIGHT_MOTOR_SCALAR * speed);
  leftMotor.setVelocity(speed);
}

void Robot::turnLeftSeconds(double time, double speed) {
  double startTime = millis();
  while ((millis() - startTime) < time*1000) {
      turnLeft(speed);
  }
  turnLeft(0); // Stop after turning
}

void Robot::turnRightSeconds(double time, double speed) {
  double startTime = millis();
  while ((millis() - startTime) < time*1000) {
      turnRight(speed);
  }
  turnRight(0); // Stop after turn
}

// void Robot::calculateAndUpdatePose(double* axialOffset, double* lateralOffset, double* headingOffset) {
//     long leftMotorPosition = -leftMotor.getPosition();
//     long rightMotorPosition = rightMotor.getPosition();

//     double leftNumberOfRevolutions = leftMotorPosition / this->COUNTS_PER_REVOLUTION;
//     double rightNumberOfRevolutions = rightMotorPosition / this->COUNTS_PER_REVOLUTION;

//     double leftDistanceTraveled = leftNumberOfRevolutions * 2.0 * 3.14159265 * this->WHEEL_RADIUS;
//     double rightDistanceTraveled = rightNumberOfRevolutions * 2.0 * 3.14159265 * this->WHEEL_RADIUS;

//     double averageDisplacement = (leftDistanceTraveled + rightDistanceTraveled) / 2.0; // cm
//     double deltaHeading = (leftDistanceTraveled - rightDistanceTraveled) / this->TRACK_WIDTH; // radians

//     double deltaX = averageDisplacement * cos(deltaHeading);
//     double deltaY = averageDisplacement * sin(deltaHeading);

//     *axialOffset = deltaX;
//     *lateralOffset = deltaY;
//     *headingOffset = deltaHeading;
// }


// void Robot::forwardCm(double distance) {
//   double axialOffset = 0;
//   double lateralOffset = 0;
//   double headingOffset = 0;
//   // double turnSpeed = 0;
//   double forwardSpeed = 0;
//   double setPoint = 0 + distance;

//   // calculates the new speed to set the motors
//   // uses axialOffset and setPoint, puts the speed INTO  forwardSpeed. KP, KI, and KD are just numbers to tune, nothing to care about
//   PID axialPID(&axialOffset, &forwardSpeed, &setPoint, KP, KI, KD, DIRECT);
//   axialPID.SetMode(AUTOMATIC); // turn pid on

//   while (true) {
//     // calculates the x, y, and heading of the robot INTO axialOffset, lateralOffset, and headingOffset 
//     calculateAndUpdatePose(&axialOffset, &lateralOffset, &headingOffset);
//     // uses the axialOffset and the setPoint to calculate the new speed and puts it into forwardSpeed
//     axialPID.Compute();
//     // moves the robot forward
//     forward(forwardSpeed);
//     Serial.print("axialOffset:");
//     Serial.print(axialOffset);
//     Serial.print("forwardSpeed:");
//     Serial.println(forwardSpeed);
//   }
// }