#pragma once

#include "types.h"
#include "imu.h"

class Estimator {
    public:
        Estimator();

        void initialize(const Quat& initial_ori, const Vec3& initial_pos);

        void update(double dt, const IMUData& imu_data);

        Quat getAttitude() const { return est_ori; }
        Vec3 getAngularVelocity() const { return est_omega; }

        void updatePositionExternal(const Vec3& pos, const Vec3& vel);
        Vec3 getPosition() const { return est_pos; }
        Vec3 getVelocity() const { return est_vel; }

    private:
        Quat est_ori = Quat::Identity();
        Vec3 est_omega = Vec3::Zero();
        Vec3 est_pos = Vec3::Zero();
        Vec3 est_vel = Vec3::Zero();

        const double alpha = 0.98; 
};