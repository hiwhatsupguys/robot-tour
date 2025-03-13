#include "Robot.h"

Robot::Robot(Motor leftMotor, Motor rightMotor)
    : leftMotor(leftMotor), rightMotor(rightMotor), 
      axialPosition(0), lateralPosition(0), heading(0) {
}


// turn counter clockwise
void Robot::turn(double speed) {
    rightMotor.setVelocity(-speed);
    leftMotor.setVelocity(-speed);
}

// // Implemented turnRadians function
// void Robot::turnRadians(double angle) {
//     // Store initial encoder positions
//     long initialLeftPosition = -leftMotor.getPosition();
//     long initialRightPosition = rightMotor.getPosition();
    
//     // Calculate arc length for each wheel
//     double arcLength = angle * TRACK_WIDTH / 2.0;
//     double targetEncoderCounts = (arcLength / (2.0 * 3.14159265 * WHEEL_RADIUS)) * COUNTS_PER_REVOLUTION;
    
//     double turnSpeed = 0.3; // Moderate turn speed
//     double targetHeading = heading + angle;
    
//     if (angle > 0) {
//         // Turn counterclockwise
//         while (heading < targetHeading - HEADING_TOLERANCE) {
//             turn(turnSpeed);
//             double axialDelta, lateralDelta, headingDelta;
//             calculateAndUpdatePose(&axialDelta, &lateralDelta, &headingDelta);
//             heading += headingDelta;
//             delay(10);
//         }
//     } else {
//         // Turn clockwise
//         while (heading > targetHeading + HEADING_TOLERANCE) {
//             turn(-turnSpeed);
//             double axialDelta, lateralDelta, headingDelta;
//             calculateAndUpdatePose(&axialDelta, &lateralDelta, &headingDelta);
//             heading += headingDelta;
//             delay(10);
//         }
//     }
    
//     // Stop the motors
//     turn(0);
// }

void Robot::forward(double speed) {
    // Reduce RIGHT_MOTOR_SCALAR to 0.95 since the robot is now drifting right
    rightMotor.setVelocity(-0.95*speed);
    leftMotor.setVelocity(speed);
}


void Robot::forwardCm(double distance) {
    // Store initial encoder positions
    long initialLeftPosition = -leftMotor.getPosition();
    long initialRightPosition = rightMotor.getPosition();
    
    // Initialize position tracking for this movement
    double currentDistance = 0;
    double heading = 0;
    
    double targetDistance = distance;
    
    // PID variables
    double forwardSpeed = 0.5; // Start with a moderate speed
    double turnCorrection = 0;
    
    // Time tracking for the PID loop
    unsigned long currentTime = millis();
    unsigned long previousTime = currentTime;
    unsigned long startTime = currentTime;
    unsigned long timeout = startTime + 10000; // 10-second timeout
    
    // Error accumulation for I term
    double totalHeadingError = 0;
    double previousHeadingError = 0;
    
    // Debug output header
    Serial.println("Distance,Target,Heading,ForwardSpeed,TurnCorrection");
    
    while (abs(targetDistance - currentDistance) > AXIAL_TOLERANCE) {
        // Update time
        currentTime = millis();
        double deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
        previousTime = currentTime;
        
        // Get position deltas
        double axialDelta, lateralDelta, headingDelta;
        calculateAndUpdatePose(&axialDelta, &lateralDelta, &headingDelta);
        
        // Update our position and heading estimates
        currentDistance += axialDelta;
        heading += headingDelta;
        
        // Calculate PID terms for heading correction
        double headingError = 0 - heading; // We want heading to be 0
        totalHeadingError += headingError * deltaTime;
        double derivativeError = (headingError - previousHeadingError) / deltaTime;
        previousHeadingError = headingError;
        
        // Simple P controller for distance
        if (abs(targetDistance - currentDistance) < 3.0) {
            // Slow down as we approach the target
            forwardSpeed = 0.3;
        }
        
        // PID controller for heading
        turnCorrection = KP * headingError + KI * totalHeadingError + KD * derivativeError;
        
        // Limit turn correction
        if (turnCorrection > 0.3) turnCorrection = 0.3;
        if (turnCorrection < -0.3) turnCorrection = -0.3;
        
        // Apply motor speeds with heading correction
        rightMotor.setVelocity(-0.95*(forwardSpeed - turnCorrection));
        leftMotor.setVelocity(forwardSpeed + turnCorrection);
        
        // Debug output
        Serial.print(currentDistance);
        Serial.print(",");
        Serial.print(targetDistance);
        Serial.print(",");
        Serial.print(heading * 180 / 3.14159265); // Convert to degrees for readability
        Serial.print(",");
        Serial.print(forwardSpeed);
        Serial.print(",");
        Serial.println(turnCorrection);
        
        // Check for timeout or target reached
        if (millis() > timeout) {
            Serial.println("Timeout reached!");
            break;
        }
        
        // Crucial: Check if we've passed the target
        if (currentDistance > targetDistance && targetDistance > 0) {
            Serial.println("Target distance reached!");
            break;
        }
        
        delay(10); // Short delay to prevent overwhelming the serial monitor
    }
    
    // Stop the motors
    rightMotor.setVelocity(0);
    leftMotor.setVelocity(0);
    
    Serial.println("Movement complete!");
    Serial.print("Final distance: ");
    Serial.print(currentDistance);
    Serial.print(" cm, Target: ");
    Serial.print(targetDistance);
    Serial.print(" cm, Final heading: ");
    Serial.print(heading * 180 / 3.14159265);
    Serial.println(" degrees");
}



void Robot::forwardSeconds(double time, double speed) {
    // Reset heading for this movement
    heading = 0;
    
    double startTime = millis();
    double turnCorrection = 0;
    
    // Simple P controller for heading during timed movement
    while ((millis() - startTime) < time*1000) {
        double axialDelta, lateralDelta, headingDelta;
        calculateAndUpdatePose(&axialDelta, &lateralDelta, &headingDelta);
        
        // Update heading
        heading += headingDelta;
        
        // Simple proportional control for heading
        turnCorrection = heading * 0.5; // P factor for heading correction
        
        // Apply motor speeds with heading correction - note the 0.95 scalar for right motor
        rightMotor.setVelocity(-0.95*(speed - turnCorrection));
        leftMotor.setVelocity(speed + turnCorrection);
        
        delay(10);
    }
    
    // Stop the motors
    rightMotor.setVelocity(0);
    leftMotor.setVelocity(0);
}



void Robot::calculateAndUpdatePose(double* axialOffset, double* lateralOffset, double* headingOffset) {
    // Get current encoder positions
    long leftMotorPosition = -leftMotor.getPosition();
    long rightMotorPosition = rightMotor.getPosition();
    
    // Store previous positions for delta calculation
    static long prevLeftPosition = leftMotorPosition;
    static long prevRightPosition = rightMotorPosition;
    
    // Calculate change in encoder counts
    long deltaLeftCounts = leftMotorPosition - prevLeftPosition;
    long deltaRightCounts = rightMotorPosition - prevRightPosition;
    
    // Update previous positions
    prevLeftPosition = leftMotorPosition;
    prevRightPosition = rightMotorPosition;
    
    // Convert to wheel revolutions
    double leftNumberOfRevolutions = deltaLeftCounts / COUNTS_PER_REVOLUTION;
    double rightNumberOfRevolutions = deltaRightCounts / COUNTS_PER_REVOLUTION;
    
    // Calculate distance traveled by each wheel
    double leftDistanceTraveled = leftNumberOfRevolutions * 2.0 * 3.14159265 * WHEEL_RADIUS;
    double rightDistanceTraveled = rightNumberOfRevolutions * 2.0 * 3.14159265 * WHEEL_RADIUS;
    
    // Calculate robot movement
    double avgDistance = (leftDistanceTraveled + rightDistanceTraveled) / 2.0; // cm
    
    // FIXED: Corrected the sign in the heading calculation
    // If the robot is now turning right, we need to switch the order
    *headingOffset = (leftDistanceTraveled - rightDistanceTraveled) / TRACK_WIDTH; // radians
    
    // Calculate X and Y components based on current heading
    // Use small angle approximation for more stability
    *axialOffset = avgDistance;
    *lateralOffset = 0; // Simplify for straight line movements
}