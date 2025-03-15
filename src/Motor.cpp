#include "Motor.h"
#include <Encoder.h>

Motor::Motor(int encoderPinA, int encoderPinB, String m1Pin, String m2Pin, String vccPin, String gndPin)
    : encoder(encoderPinA, encoderPinB) {
    
    // Store pin configurations
    this->m1Pin = m1Pin;
    this->m2Pin = m2Pin;
    this->vccPin = vccPin;
    this->gndPin = gndPin;

    // Set up motor control pins
    pinMode(m1Pin.toInt(), OUTPUT);
    pinMode(m2Pin.toInt(), OUTPUT);
    pinMode(vccPin.toInt(), OUTPUT);
    pinMode(gndPin.toInt(), OUTPUT);
    
    // Initialize pins to stopped state
    digitalWrite(vccPin.toInt(), LOW);
    digitalWrite(gndPin.toInt(), LOW);
    digitalWrite(m1Pin.toInt(), LOW);
    digitalWrite(m2Pin.toInt(), LOW);
}

double Motor::getVelocity() {
    return velocity;
}

void Motor::setVelocity(double velocity) {
    // Clamp velocity between -1 and 1
    velocity = constrain(velocity, -1.0, 1.0);
    
    if (velocity > 0) {
        // Forward
        digitalWrite(m1Pin.toInt(), HIGH);
        digitalWrite(m2Pin.toInt(), LOW);
        digitalWrite(vccPin.toInt(), HIGH);
        digitalWrite(gndPin.toInt(), LOW);
    } 
    else if (velocity < 0) {
        // Reverse
        digitalWrite(m1Pin.toInt(), LOW);
        digitalWrite(m2Pin.toInt(), HIGH);
        digitalWrite(vccPin.toInt(), HIGH);
        digitalWrite(gndPin.toInt(), LOW);
    } 
    else {
        // Stop
        digitalWrite(m1Pin.toInt(), LOW);
        digitalWrite(m2Pin.toInt(), LOW);
        digitalWrite(vccPin.toInt(), LOW);
        digitalWrite(gndPin.toInt(), LOW);
    }
    
    this->velocity = velocity;
}

int Motor::getPosition() {
    return encoder.read();
}

void Motor::updatePosition() {
    // Add any position update logic if needed
}