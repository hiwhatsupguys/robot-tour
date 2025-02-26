#pragma once

class Encoder {
public:
  Encoder(int pinA, int pinB);
  int getPosition();
  void updatePosition();


private:
  int pinA;
  int pinB;
  int currentPosition;
  int aState;
  int aLastState;
};