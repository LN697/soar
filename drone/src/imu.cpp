#include "imu.h"

IMU::IMU(const IMUParams& p) : params(p) {
    // Initialize random bias for this run
    std::normal_distribution<double> bias_dist(0.0, params.gyro_bias_std);
    gyro_bias = Vec3(bias_dist(generator), bias_dist(generator), bias_dist(generator));

    dist_accel = std::normal_distribution<double>(0.0, params.accel_noise_std);
    dist_gyro  = std::normal_distribution<double>(0.0, params.gyro_noise_std);
}

void IMU::update(const Vec3& true_accel_body, const Vec3& true_omega_body, double dt) {
    (void)dt; // Unused for now, but useful if we add random walk drift later

    // 1. Accelerometer Model
    // Accel measures (Acceleration - Gravity) in Body Frame.
    // However, in physics engines, "true_accel_body" usually implies kinematic acceleration.
    // The sensor measures Reaction Force.
    // If the drone is hovering, accel measures +1g UP (reaction to gravity).
    // Ideally: Sensor = R_world_to_body * (a_world + g_world) + Noise
    // We assume the caller passes the proper "Proper Acceleration" or we handle it here.
    // Let's assume input 'true_accel_body' IS the proper acceleration (F_total / mass).
    
    meas_accel.x() = true_accel_body.x() + dist_accel(generator);
    meas_accel.y() = true_accel_body.y() + dist_accel(generator);
    meas_accel.z() = true_accel_body.z() + dist_accel(generator);

    // 2. Gyroscope Model
    meas_gyro.x() = true_omega_body.x() + gyro_bias.x() + dist_gyro(generator);
    meas_gyro.y() = true_omega_body.y() + gyro_bias.y() + dist_gyro(generator);
    meas_gyro.z() = true_omega_body.z() + gyro_bias.z() + dist_gyro(generator);
}

IMUData IMU::read() const {
    return {meas_accel, meas_gyro};
}