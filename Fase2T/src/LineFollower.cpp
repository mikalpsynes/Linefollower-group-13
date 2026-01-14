#include "LineFollower.h"

// GPIO definitions
#define AIN1 GPIO_NUM_16
#define AIN2 GPIO_NUM_17
#define PWMA GPIO_NUM_4
#define PWMA_CHANNEL 0

#define BIN1 GPIO_NUM_5
#define BIN2 GPIO_NUM_15
#define PWMB GPIO_NUM_32
#define PWMB_CHANNEL 1

LineFollower::LineFollower(uint8_t baseSpd, uint8_t maxSpd)
    : rightMotor(AIN1, AIN2, PWMA, PWMA_CHANNEL, maxSpd),
      leftMotor(BIN1, BIN2, PWMB, PWMB_CHANNEL, maxSpd),
      pidController(1.0, 0.04, 7.0),
      baseSpeedA(baseSpd),
      baseSpeedB(baseSpd),
      maxSpeedA(maxSpd),
      maxSpeedB(maxSpd),
      lineThreshold(750) {
}

void LineFollower::init() {
    Serial.begin(115200);
    Serial.println("LineFollower initializing...");

    rightMotor.init();
    leftMotor.init();
    sensor.init();

    Serial.println("Initialization complete!");
}

void LineFollower::calibrate() {
    sensor.calibrate();
}

void LineFollower::update() {
    // Read line position
    uint16_t position = sensor.readLinePosition();
    int error = 5000 - position;

    // Calculate PID output
    int motorSpeed = pidController.calculate(error);

    // Calculate motor speeds
    int speedA = baseSpeedA + motorSpeed;
    int speedB = baseSpeedB - motorSpeed;

    // Clamp speeds
    speedA = constrain(speedA, -50, maxSpeedA);
    speedB = constrain(speedB, -50, maxSpeedB);

    // Debug output
    Serial.print("Position: ");
    Serial.print(position);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | SpeedA: ");
    Serial.print(speedA);
    Serial.print(" | SpeedB: ");
    Serial.println(speedB);

    // If error is small, go straight at max speed
    if (error >= -lineThreshold && error <= lineThreshold) {
        rightMotor.setSpeed(maxSpeedA);
        leftMotor.setSpeed(maxSpeedB);
    } else {
        rightMotor.setSpeed(speedA);
        leftMotor.setSpeed(speedB);
    }
}

void LineFollower::stop() {
    rightMotor.stop();
    leftMotor.stop();
}

void LineFollower::setBaseSpeed(uint8_t speedA, uint8_t speedB) {
    baseSpeedA = speedA;
    baseSpeedB = speedB;
}

void LineFollower::setMaxSpeed(uint8_t speedA, uint8_t speedB) {
    maxSpeedA = speedA;
    maxSpeedB = speedB;
}

void LineFollower::setPIDGains(float kp, float ki, float kd) {
    pidController.setGains(kp, ki, kd);
}
