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


// Modify these global variables
unsigned long lastPrintTime = 0;
long lastLeftPosition = 0;
long lastRightPosition = 0;
const unsigned long PRINT_INTERVAL = 100; // 0.1 second interval

// Add these global variables at the top
const int WINDOW_SIZE = 5;  // Number of readings to average
int32_t leftReadings[WINDOW_SIZE] = {0};
int32_t rightReadings[WINDOW_SIZE] = {0};
int readIndex = 0;

void loop() {
     printMotorPositions();

}

void printMotorPositions() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    leftMotorNewPosition = robot.leftMotor.getPosition();
    rightMotorNewPosition = robot.rightMotor.getPosition();

    // Calculate ticks per 0.1 second
    int32_t leftTicks = 0;
    int32_t rightTicks = 0;
    
    if (lastPrintTime > 0) {
      leftTicks = (int32_t)(leftMotorNewPosition - lastLeftPosition);
      rightTicks = (int32_t)(rightMotorNewPosition - lastRightPosition);
      
      // Store in circular buffer
      leftReadings[readIndex] = leftTicks;
      rightReadings[readIndex] = rightTicks;
      readIndex = (readIndex + 1) % WINDOW_SIZE;
      
      // Calculate averages
      int32_t leftAvg = 0;
      int32_t rightAvg = 0;
      for(int i = 0; i < WINDOW_SIZE; i++) {
        leftAvg += leftReadings[i];
        rightAvg += rightReadings[i];
      }
      leftAvg /= WINDOW_SIZE;
      rightAvg /= WINDOW_SIZE;
      
      // Only print if there's significant movement
      if(abs(leftAvg) > 0 || abs(rightAvg) > 0) {
        Serial.print("L:");
        Serial.print(leftAvg);
        Serial.print(" R:");
        Serial.println(rightAvg);
      }
    }

    // Update last positions and time
    lastLeftPosition = leftMotorNewPosition;
    lastRightPosition = rightMotorNewPosition;
    lastPrintTime = currentTime;
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