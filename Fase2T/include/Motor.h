#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
    int directionPin1;
    int directionPin2;
    int pwmPin;
    uint8_t pwmChannel;
    uint8_t maxSpeed;

public:
    Motor(int dir1, int dir2, int pwm, uint8_t channel, uint8_t maxSpd);
    void init();
    void setSpeed(int speed);
    void stop();
};

#endif

