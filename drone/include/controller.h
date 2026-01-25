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

    // Initialize PIDs
    void init(double dt);

    // Main Run Function
    // Takes: Desired State (Position, Yaw), Estimated State (Pos, Vel, Ori, Gyro)
    // Returns: Motor Throttles (0-1)
    MotorMix update(double dt, 
                    const ControlInputs& inputs, 
                    const Vec3& est_pos, const Vec3& est_vel, 
                    const Quat& est_ori, const Vec3& est_gyro);

private:
    // -- PIDs --
    // Position Loop: Input(m) -> Output(Velocity m/s)
    PID pid_x, pid_y, pid_z;
    
    // Velocity Loop: Input(m/s) -> Output(Acceleration m/s^2 -> Angle)
    PID pid_vx, pid_vy, pid_vz;

    // Attitude Loop: Input(Angle rad) -> Output(Rate rad/s)
    PID pid_roll, pid_pitch, pid_yaw;

    // Rate Loop: Input(Rate rad/s) -> Output(Torque/Thrust mix)
    PID pid_p, pid_q, pid_r;

    // Helpers
    Vec3 computeDesiredAcceleration(double dt, const Vec3& target_pos, const Vec3& est_pos, const Vec3& est_vel);
    Vec3 computeDesiredAngles(const Vec3& acc_des, double yaw_target);
    Vec3 computeDesiredRates(double dt, const Vec3& target_euler, const Quat& est_ori);
    
    const double GRAVITY = 9.81;
    const double MASS = 1.0; // kg, needs to match Drone
    const double MAX_TILT = 0.5; // rad (~30 deg)
};