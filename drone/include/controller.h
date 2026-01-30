#pragma once

#include "types.h"
#include "pid.h"

#include <array>

struct ControlInputs {
    Vec3 target_pos;
    double target_yaw;
};

struct MotorMix {
    std::array<double, 4> throttle_01; // 0.0 to 1.0
};

class Controller {
    public:
        Controller();

        void init(double dt);
        
        MotorMix update(double dt, const ControlInputs& inputs, const Vec3& est_pos, const Vec3& est_vel, const Quat& est_ori, const Vec3& est_gyro);

    private:
        PID pid_x, pid_y, pid_z;
        
        PID pid_vx, pid_vy, pid_vz;

        PID pid_roll, pid_pitch, pid_yaw;

        PID pid_p, pid_q, pid_r;

        Vec3 computeDesiredAcceleration(double dt, const Vec3& target_pos, const Vec3& est_pos, const Vec3& est_vel);
        Vec3 computeDesiredAngles(const Vec3& acc_des, double yaw_target);
        Vec3 computeDesiredRates(double dt, const Vec3& target_euler, const Quat& est_ori);
        
        const double GRAVITY = 9.81;
        const double MASS = 1.0; // kg, needs to match Drone
        const double MAX_TILT = 0.5; // rad (~30 deg)
};