// https://www.instructables.com/Arduino-Motor-Shield-Tutorial/
// https://medium.com/@chrishio/basics-reading-dc-motor-encoder-values-with-arduino-2c185f3601ef
// https://www.electroniclinic.com/arduino-dc-motor-speed-control-with-encoder-arduino-dc-motor-encoder/

// controlling motor with encoder (helpful video):
// https://www.youtube.com/watch?v=dTGITLnYAY0

// https://docs.arduino.cc/tutorials/motor-shield-rev3/msr3-controlling-dc-motor/

// MOTOR + ENCODER INFO:
// https://www.pololu.com/file/0J1487/pololu-micro-metal-gearmotors-rev-6-1.pdf

// ENCODER LIBRARY (doesn't work):
// https://www.instructables.com/Motor-With-Encoder-How-to-Read-Input-Value-From-En/

// current encoder code:
// https://howtomechatronics.com/tutorials/arduino/rotary-encoder-works-use-arduino/

// general differential drive robot code with odometry and stuff:
// https://www.youtube.com/watch?v=337Sp3PtVDc

// amazing source for calculating posistion and heading (odometry):
// https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/

#include "Motor.h"
#include "Robot.h"
#include "math.h"

String inputString = "";
bool stringComplete = false;

// needs to be pins with interrupts, so pins 2 (digital) and 3 (analog) ?
// setting this to 2 and 3 only allows 1, 0, -1 input

// encoder pin a, encoder pin b, pwm pin, direction pin, brake pin
Motor leftMotor(2, 6, 3, 12, 9);
// white cable on left motor broke :(
Motor rightMotor(5, 10, 11, 13, 8); 

Robot robot(leftMotor, rightMotor);


// from my testing, the minimum motor speed is about 80-90
void setup() {

  Serial.begin(9600);
  inputString.reserve(200);
  // robot.forward(1);

}

long rightMotorOldPosition = -999;
long leftMotorOldPosition = -999;
long leftMotorNewPosition = robot.leftMotor.getPosition();
long rightMotorNewPosition = robot.rightMotor.getPosition();



void loop() {
    printMotorPositions();

    // robot.turn(1)
    
    // delay(3000);



  // if (stringComplete) {
  //   // Serial.println("string: " + inputString);
  //   float speed = inputString.toFloat();

  //   robot.forward(speed);

  //   inputString = "";
  //   stringComplete = false;
  //   // Serial.print("motor speed set to: ");
  //   // Serial.println(motorVelocity);
  // }

  



  // long leftMotorNewPosition = leftMotor.getPosition();
  // if (leftMotorNewPosition != leftMotorOldPosition) {
  //   leftMotorOldPosition = leftMotorNewPosition;
  //   Serial.println("left motor position: ");
  //   Serial.println(leftMotorNewPosition);
  // }
  // wait 3000 ms
  // delay(3000);
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