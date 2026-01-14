#include "Motor.h"

Motor::Motor(int dir1, int dir2, int pwm, uint8_t channel, uint8_t maxSpd)
    : directionPin1(dir1), directionPin2(dir2), pwmPin(pwm),
      pwmChannel(channel), maxSpeed(maxSpd) {
}

void Motor::init() {
    pinMode(directionPin1, OUTPUT);
    pinMode(directionPin2, OUTPUT);

    ledcSetup(pwmChannel, 5000, 8); // 5kHz, 8-bit resolution
    ledcAttachPin(pwmPin, pwmChannel);

    stop();
}

void Motor::setSpeed(int speed) {
    // Clamp speed to valid range
    speed = constrain(speed, -maxSpeed, maxSpeed);

    if (speed > 0) {
        digitalWrite(directionPin1, HIGH);
        digitalWrite(directionPin2, LOW);
    } else if (speed < 0) {
        digitalWrite(directionPin1, LOW);
        digitalWrite(directionPin2, HIGH);
        speed = -speed;
    } else {
        digitalWrite(directionPin1, LOW);
        digitalWrite(directionPin2, LOW);
    }

    uint8_t pwmValue = (uint8_t)min(255, speed);
    ledcWrite(pwmChannel, pwmValue);
}

void Motor::stop() {
    digitalWrite(directionPin1, LOW);
    digitalWrite(directionPin2, LOW);
    ledcWrite(pwmChannel, 0);
}

