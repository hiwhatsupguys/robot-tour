#include "Motor.h"
#include "Robot.h"
#include "math.h"

String inputString = "";
bool stringComplete = false;

// needs to be pins with interrupts, so pins 2 (digital) and 3 (analog) ?
// setting this to 2 and 3 only allows 1, 0, -1 input

// encoder pin a, encoder pin b, pwm pin, direction pin, brake pin
Motor leftMotor(2, 6, 5, 12, 9);

Motor rightMotor(3, 10, 11, 13, 8); 

Robot robot(leftMotor, rightMotor);


// from my testing, the minimum motor speed is about 80-90
void setup() {
    Serial.begin(9600);
    inputString.reserve(200);
    
    // Initialize both motors to stopped state
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);
}

long rightMotorOldPosition = -999;
long leftMotorOldPosition = -999;
long leftMotorNewPosition = robot.leftMotor.getPosition();
long rightMotorNewPosition = robot.rightMotor.getPosition();



// example for basic turning 
/* robot.turnLeftSeconds(0.45, 0.3);
    delay(1000);
    robot.turnRightSeconds(0.45, 0.35);*/

    // robot.turnLeftSeconds(0.45, 0.3);
    // delay(20000000000);

void loop() {
    printMotorPositions();
    robot.forwardSeconds(1, 0.5);
    delay(1000000);
    
    // Rest of your code...
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