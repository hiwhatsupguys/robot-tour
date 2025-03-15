#pragma once
#include <Encoder.h>

class Motor {
    public:
        Motor(int encoderPinA, int encoderPinB, String m1Pin, String m2Pin, String vccPin, String gndPin);
        double getVelocity();
        void setVelocity(double velocity);
        int getPosition();
        void updatePosition();

    private:
        Encoder encoder;
        double velocity = 0;
        String m1Pin;
        String m2Pin;
        String vccPin;
        String gndPin;
};