#include "Encoder.h"

Encoder::Encoder(int pinA, int pinB) {
  currentPosition = 0;
  this->pinA = pinA;
  this->pinB = pinB;
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  aLastState = digitalRead(pinA);

}

int Encoder::getPosition() {
  return currentPosition;
}

void Encoder::updatePosition() {
  aState = digitalRead(pinA);  // Reads the "current" state of the ENCODER_A
  // If the previous and the current state of the ENCODER_A are different, that means a Pulse has occured
  if (aState != aLastState) {
    // If the ENCODER_B state is different to the ENCODER_A state, that means the encoder is rotating clockwise
    if (digitalRead(pinB) != aState) {
      currentPosition++;
    } else {
      currentPosition--;
    }
  }
  aLastState = aState;  // Updates the previous state of the ENCODER_A with the current state
}