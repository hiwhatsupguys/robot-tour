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
#include "math.h"

String inputString = "";
bool stringComplete = false;
float motorVelocity = 0;

// counts for revolution for the encoder
static const int COUNTS_PER_REVOLUTION = 12*50; // CPR * gear ratio
// wheel radius in cm
static const float WHEEL_RADIUS = 2;

// needs to be pins with interrupts, so pins 2 (digital) and 3 (analog) ?
// setting this to 2 and 3 only allows 1, 0, -1 input

// encoder pin a, encoder pin b, pwm pin, direction pin, brake pin
Motor rightMotor(5, 6, 3, 12, 9);
Motor leftMotor(2, 10, 11, 13, 8);

const float INIT_X = 0;
float INIT_Y = 0;
float INIT_HEADING = 0;

struct pose {
  float x;
  float y;
  float heading;
};

struct pose robotPose = {INIT_X, INIT_Y, INIT_HEADING};


// from my testing, the minimum motor speed is about 80-90
void setup() {

  Serial.begin(115200);
  inputString.reserve(200);
}

long rightMotorOldPosition = -999;
long leftMotorOldPosition = -999;



void loop() {

  // if (stringComplete) {
  //   // Serial.println("string: " + inputString);
  //   motorVelocity = inputString.toFloat();

  //   rightMotor.setVelocity(motorVelocity);

  //   inputString = "";
  //   stringComplete = false;
  //   // Serial.print("motor speed set to: ");
  //   // Serial.println(motorVelocity);
  // }


  long rightMotorNewPosition = rightMotor.getPosition();


  // printing encoder position
  if (rightMotorNewPosition != rightMotorOldPosition) {
    rightMotorOldPosition = rightMotorNewPosition;
    Serial.println("right motor position: ");
    Serial.println(rightMotorNewPosition);
  }
  // long leftMotorNewPosition = leftMotor.getPosition();
  // if (leftMotorNewPosition != leftMotorOldPosition) {
  //   leftMotorOldPosition = leftMotorNewPosition;
  //   Serial.println("left motor position: ");
  //   Serial.println(leftMotorNewPosition);
  // }
}

// https://automaticaddison.com/calculating-wheel-odometry-for-a-differential-drive-robot/
void calculateAndUpdatePose(long leftMotorPosition, long rightMotorPosition) {

    long leftNumberOfRevolutions = leftMotorPosition/COUNTS_PER_REVOLUTION;
    long rightNumberOfRevolutions = rightMotorPosition/COUNTS_PER_REVOLUTION;

    float leftDistanceTraveled = leftNumberOfRevolutions * 2 * 3.14159265 * WHEEL_RADIUS;
    float rightDistanceTraveled = rightNumberOfRevolutions * 2 * 3.14159265 * WHEEL_RADIUS;

    float averageDisplacement = (leftDistanceTraveled + rightDistanceTraveled) / 2; // cm
    float deltaHeading = (leftDistanceTraveled - rightDistanceTraveled) / 2; // radians

    float deltaX = averageDisplacement * cos(deltaHeading);
    float deltaY = averageDisplacement * sin(deltaHeading);

    robotPose.x = INIT_X + deltaX;
    robotPose.y = INIT_Y + deltaY;
    robotPose.heading = INIT_HEADING + deltaHeading;    
}


void goStraight(float velocity) {
  rightMotor.setVelocity(velocity);
  leftMotor.setVelocity(-velocity);
}

void turn(float strength) {
  rightMotor.setVelocity(strength);
  leftMotor.setVelocity(strength);
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