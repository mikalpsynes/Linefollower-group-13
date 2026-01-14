#include "PIDController.h"

PIDController::PIDController(float kp_val, float ki_val, float kd_val)
    : kp(kp_val), ki(ki_val), kd(kd_val),
      p(0), i(0), d(0), lastError(0),
      iMax(5000), iMin(-5000) {
}

void PIDController::reset() {
    p = 0;
    i = 0;
    d = 0;
    lastError = 0;
}

int PIDController::calculate(int error) {
    p = error;
    i = i + error;

    // Clamp integral term
    if (i > iMax) i = iMax;
    if (i < iMin) i = iMin;

    d = error - lastError;
    lastError = error;

    float output = p * kp + i * ki + d * kd;
    return (int)output;
}

void PIDController::setGains(float kp_val, float ki_val, float kd_val) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
}

void PIDController::setILimits(int min_val, int max_val) {
    iMin = min_val;
    iMax = max_val;
}

