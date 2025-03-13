#include "DifferentialDrive.h"

// Define robot parameters
#define COUNTS_PER_REVOLUTION 12.0*50.0  // CPR * gear ratio
#define WHEEL_RADIUS 2.0
#define TRACK_WIDTH 11.0

// Create differential drive instance
DifferentialDrive drive(COUNTS_PER_REVOLUTION, WHEEL_RADIUS, TRACK_WIDTH);

void setup() {
  Serial.begin(9600);
  
  // Initialize the differential drive system
  drive.init(2, 6, 3, 12, 9,    // Left motor pins (encoder A, encoder B, PWM, dir, brake)
             5, 10, 11, 13, 8);  // Right motor pins (encoder A, encoder B, PWM, dir, brake)
  
  // Set PID parameters (Kp, Ki, Kd)
  drive.setPIDParams(2.0, 0.1, 0.5);
  
  Serial.println("Robot initialized");
  delay(1000);
}

void loop() {
  // Example usage
  Serial.println("Moving forward 30 cm");
  drive.forwardCm(30);
  delay(1000);
  
  Serial.println("Turning 90 degrees");
  drive.turnInPlace(90);
  delay(1000);
  
  Serial.println("Moving forward 20 cm");
  drive.forwardCm(20);
  delay(1000);
  
  Serial.println("Turning -90 degrees");
  drive.turnInPlace(-90);
  delay(1000);
  
  // Wait before repeating
  Serial.println("Waiting...");
  delay(5000);
}