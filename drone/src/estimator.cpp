#include "estimator.h"
#include <cmath>

Estimator::Estimator() {}

void Estimator::initialize(const Quat& initial_ori, const Vec3& initial_pos) {
    est_ori = initial_ori;
    est_pos = initial_pos;
}

void Estimator::updatePositionExternal(const Vec3& pos, const Vec3& vel) {
    est_pos = pos;
    est_vel = vel;
}

void Estimator::update(double dt, const IMUData& imu_data) {
    // 1. Prediction (Gyro Integration)
    est_omega = imu_data.gyro; // Use raw gyro for rates (or bias corrected if we estimated bias)
    
    // Quaternion Derivative: q_dot = 0.5 * q * omega
    Quat q_dot;
    q_dot.w() = 0;
    q_dot.vec() = est_omega;
    
    Quat prediction = est_ori * q_dot;
    
    Quat q_pred = est_ori;
    q_pred.w() += prediction.w() * 0.5 * dt;
    q_pred.vec() += prediction.vec() * 0.5 * dt;
    q_pred.normalize();

    // 2. Correction (Accelerometer Gravity Vector)
    // Only correct Roll/Pitch. Yaw is unobservable by Accel.
    // Calculate "Up" vector predicted by orientation
    // Gravity is Down (-Z), so Accel measures Up (+Z) in body frame when flat.
    
    Vec3 expected_g_direction = q_pred.inverse()._transformVector(Vec3(0,0,1));
    Vec3 measured_g_direction = imu_data.accel.normalized();

    // Calculate error angle between expected and measured
    // Simple Complementary: Interpolate the Quaternions or mix the angles?
    // Let's use the Classic Complementary approach on Euler Angles for simplicity,
    // OR a Tilt-Correction Quaternion approach.
    
    // -- Simplified Quaternion SLERP-like approach --
    // We want to rotate 'expected' to 'measured' by a small factor.
    // Ideally, we implement Mahony or Madgwick here. 
    // For this demo, let's stick to the Classic Alpha mix:
    // New_Q = Interpolate(Gyro_Q, Accel_Q, factor)
    
    // Calculating Accel_Q (Pitch/Roll only):
    Vec3 axis = expected_g_direction.cross(measured_g_direction);
    double sin_angle = axis.norm();
    double cos_angle = expected_g_direction.dot(measured_g_direction);
    double angle = std::atan2(sin_angle, cos_angle);
    
    // Apply a small fraction of this correction
    // (1 - alpha) is the gain for the accelerometer
    double correction_factor = (1.0 - alpha) * dt * 10.0; // Scaled gain
    
    if (sin_angle > 1e-6) {
        Vec3 axis_norm = axis.normalized();
        Quat correction(Eigen::AngleAxisd(angle * correction_factor, axis_norm));
        est_ori = q_pred * correction; // Apply local correction
    } else {
        est_ori = q_pred;
    }
    
    est_ori.normalize();
}