#pragma once

#include "types.h"
#include "imu.h"

class Estimator {
public:
    Estimator();

    void initialize(const Quat& initial_ori, const Vec3& initial_pos);
    
    // Prediction Step (IMU Integration)
    // Correction Step (Accel Tilt Correction)
    void update(double dt, const IMUData& imu_data);

    // Getters
    Quat getAttitude() const { return est_ori; }
    Vec3 getAngularVelocity() const { return est_omega; }
    
    // Placeholder for Position Estimation (Pass-through or simple integration)
    // In a real system, this would Fuse GPS/Baro.
    // Here we will keep it simple or allow external "GPS" injection.
    void updatePositionExternal(const Vec3& pos, const Vec3& vel);
    Vec3 getPosition() const { return est_pos; }
    Vec3 getVelocity() const { return est_vel; }

private:
    Quat est_ori = Quat::Identity();
    Vec3 est_omega = Vec3::Zero();
    Vec3 est_pos = Vec3::Zero();
    Vec3 est_vel = Vec3::Zero();

    // Complementary Filter Gain
    // Alpha close to 1.0 trusts Gyro more (High pass)
    // Alpha close to 0.0 trusts Accel more (Low pass)
    const double alpha = 0.98; 
};