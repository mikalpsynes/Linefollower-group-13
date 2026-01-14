#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "Motor.h"
#include "LineSensor.h"
#include "PIDController.h"

class LineFollower {
private:
    Motor rightMotor;
    Motor leftMotor;
    LineSensor sensor;
    PIDController pidController;

    uint8_t baseSpeedA;
    uint8_t baseSpeedB;
    uint8_t maxSpeedA;
    uint8_t maxSpeedB;

    int lineThreshold;

public:
    LineFollower(uint8_t baseSpd = 200, uint8_t maxSpd = 250);
    void init();
    void calibrate();
    void update();
    void stop();
    void setBaseSpeed(uint8_t speedA, uint8_t speedB);
    void setMaxSpeed(uint8_t speedA, uint8_t speedB);
    void setPIDGains(float kp, float ki, float kd);
};

#endif
