#pragma once
#include "Motor.h"

class Robot {
public:
    Robot(Motor leftMotor, Motor rightMotor);
    void turn(double speed);
    void forward(double speed);
    void turnRadians(double angle);
    // distance in cm
    void forwardCm(double distance);
    void forwardSeconds(double time, double speed);
    void calculateAndUpdatePose(double* axialOffset, double* lateralOffset, double* headingOffset);
    
    Motor rightMotor;
    Motor leftMotor;
    
private:
    // number of encoder counts (or pulses) per full revolution of the motor shaft after considering the gear ratio.
    static const double COUNTS_PER_REVOLUTION = 12.0*50.0; // CPR * gear ratio
    // wheel radius in cm
    static const double WHEEL_RADIUS = 2.0;
    static const double TRACK_WIDTH = 11.0;
    
    // Global position tracking
    double axialPosition;
    double lateralPosition;
    double heading; // in radians
    
    double axialOffset;
    double lateralOffset;
    // radians
    double headingOffset;
    
    // cm
    static const double AXIAL_TOLERANCE = 3.0; 
    // radians
    static const double HEADING_TOLERANCE = 5.0*(3.14159265/180.0); // 5 degrees in radians
    
    // PID constants
    static const double KP = 0.2;
    static const double KI = 0.001;
    static const double KD = 0.1;
    /*
    KP 太小，导致误差修正不够，机器人超出目标才慢慢减速。
    尝试增大 KP，比如从 0.2 调整到 0.3 或 0.4。
    KD 太小，导致系统无法有效抑制惯性，使机器人超调。
    尝试增大 KD，比如从 0.1 调整到 0.2。
    KI 可能不太重要，因为短时间内积分项的影响较小，通常不需要大幅调整。
    */
};