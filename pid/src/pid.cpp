#include "pid.h"
#include <algorithm>

PID::PID(const PIDParams& p) : params(p) {}

double PID::update(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    // 1. Proportional Term
    double p_term = params.kp * error;

    // 2. Integral Term
    integral_sum += error * dt;
    // Anti-Windup: Clamp the integral sum separately
    integral_sum = std::clamp(integral_sum, -params.integral_max, params.integral_max);
    double i_term = params.ki * integral_sum;

    // 3. Derivative Term
    // simple difference (noisy). 
    // In production, we'd use a low-pass filter here.
    double derivative = (error - prev_error) / dt;
    double d_term = params.kd * derivative;

    prev_error = error;

    // 4. Summation
    double output = p_term + i_term + d_term;

    // 5. Output Saturation
    return std::clamp(output, params.output_min, params.output_max);
}

void PID::reset() {
    integral_sum = 0.0;
    prev_error = 0.0;
}

void PID::setGains(double kp, double ki, double kd) {
    params.kp = kp;
    params.ki = ki;
    params.kd = kd;
}
