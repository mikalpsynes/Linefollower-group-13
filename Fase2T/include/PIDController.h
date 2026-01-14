#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    float kp;
    float ki;
    float kd;
    int p;
    int i;
    int d;
    int lastError;
    int iMax;
    int iMin;

public:
    PIDController(float kp_val, float ki_val, float kd_val);
    void reset();
    int calculate(int error);
    void setGains(float kp_val, float ki_val, float kd_val);
    void setILimits(int min_val, int max_val);
};

#endif

