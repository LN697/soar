#pragma once

#include "types.h"
#include <random>

struct IMUData {
    Vec3 accel; // m/s^2
    Vec3 gyro;  // rad/s
};

struct IMUParams {
    double accel_noise_std = 0.05;  // m/s^2
    double gyro_noise_std  = 0.002; // rad/s
    double gyro_bias_std   = 0.005; // rad/s (constant random bias)
};

class IMU {
public:
    IMU(const IMUParams& params);

    // Feed "True" physics state to generate sensor readings
    void update(const Vec3& true_accel_body, const Vec3& true_omega_body, double dt);

    IMUData read() const;

private:
    IMUParams params;
    
    Vec3 gyro_bias;
    
    // Sensor readings
    Vec3 meas_accel;
    Vec3 meas_gyro;

    // Random Number Generation
    std::default_random_engine generator;
    std::normal_distribution<double> dist_accel;
    std::normal_distribution<double> dist_gyro;
};