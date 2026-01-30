#include "pid.h"
#include <algorithm>

PID::PID(const PIDParams& p) : params(p) {}

double PID::update(double error, double dt) {
    if (dt <= 0.0) return 0.0;

    double p_term = params.kp * error;

    integral_sum += error * dt;
    integral_sum = std::clamp(integral_sum, -params.integral_max, params.integral_max);
    double i_term = params.ki * integral_sum;

    // Implemet a low-pass filter
    double derivative = (error - prev_error) / dt;
    double d_term = params.kd * derivative;

    prev_error = error;

    double output = p_term + i_term + d_term;

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
