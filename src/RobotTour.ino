#include "Motor.h"
#include "Robot.h"
#include "math.h"

String inputString = "";
bool stringComplete = false;


Motor leftMotor(2, 6, 3, 12, 9);
// white cable on left motor broke :(
Motor rightMotor(5, 10, 11, 13, 8); 

Robot robot(leftMotor, rightMotor);


// from my testing, the minimum motor speed is about 80-90
void setup() {

  Serial.begin(250000);
  inputString.reserve(200);
  // robot.forward(1);

}

long rightMotorOldPosition = -999;
long leftMotorOldPosition = -999;
long leftMotorNewPosition = robot.leftMotor.getPosition();
long rightMotorNewPosition = robot.rightMotor.getPosition();



void loop() {
    printMotorPositions();
    robot.forwardSeconds(2, 0.5); 

    // robot.forwardCm(0.1);
     delay(300);
}

void printMotorPositions() {

  leftMotorNewPosition = robot.leftMotor.getPosition();
  rightMotorNewPosition = robot.rightMotor.getPosition();

  // printing encoder position
  if (rightMotorNewPosition != rightMotorOldPosition
   || leftMotorNewPosition != leftMotorOldPosition) {

    rightMotorOldPosition = rightMotorNewPosition;
    leftMotorOldPosition = leftMotorNewPosition;
    Serial.print("right:");
    Serial.print(rightMotorNewPosition);
    Serial.print(",");
    Serial.print("left:");
    Serial.println(leftMotorNewPosition);

  }
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    // If we get a newline, set the complete flag
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}