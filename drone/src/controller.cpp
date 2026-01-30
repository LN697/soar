#include "controller.h"
#include <cmath>
#include <algorithm>
#include <iostream>

Controller::Controller() 
    : pid_x({0.0, 0.0, 0.0}), pid_y({0.0, 0.0, 0.0}), pid_z({0.0, 0.0, 0.0}),
      pid_vx({0.0, 0.0, 0.0}), pid_vy({0.0, 0.0, 0.0}), pid_vz({0.0, 0.0, 0.0}),
      pid_roll({0.0, 0.0, 0.0}), pid_pitch({0.0, 0.0, 0.0}), pid_yaw({0.0, 0.0, 0.0}),
      pid_p({0.0, 0.0, 0.0}), pid_q({0.0, 0.0, 0.0}), pid_r({0.0, 0.0, 0.0})
{
    init(0.01);
}

void Controller::init(double dt) {
    (void)dt;
    
    double pos_kp = 1.0;
    pid_x = PID({pos_kp, 0, 0, -5, 5}); // Max 5 m/s
    pid_y = PID({pos_kp, 0, 0, -5, 5});
    pid_z = PID({1.5, 0.05, 0, -2, 2}); // Altitude

    double vel_kp = 2.0;
    double vel_ki = 0.1;
    double vel_kd = 0.5;
    pid_vx = PID({vel_kp, vel_ki, vel_kd, -5, 5}); // Max 5 m/s^2
    pid_vy = PID({vel_kp, vel_ki, vel_kd, -5, 5});
    pid_vz = PID({4.0, 1.0, 0.5, -10, 10}); 

    double att_kp = 6.0;
    pid_roll  = PID({att_kp, 0, 0, -10, 10});
    pid_pitch = PID({att_kp, 0, 0, -10, 10});
    pid_yaw   = PID({2.0,    0, 0, -3, 3});

    double rate_kp = 0.15; 
    double rate_ki = 0.05;
    double rate_kd = 0.003;
    pid_p = PID({rate_kp, rate_ki, rate_kd, -1, 1});
    pid_q = PID({rate_kp, rate_ki, rate_kd, -1, 1});
    pid_r = PID({0.2,     0.05,    0.0,    -1, 1});
}

Vec3 quatToEuler(const Quat& q) {
    Vec3 euler;
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    euler.x() = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        euler.y() = std::copysign(3.14159 / 2, sinp); 
    else
        euler.y() = std::asin(sinp);

    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    euler.z() = std::atan2(siny_cosp, cosy_cosp);

    return euler;
}

MotorMix Controller::update(double dt, const ControlInputs& inputs, const Vec3& est_pos, const Vec3& est_vel, const Quat& est_ori, const Vec3& est_gyro) {
    Vec3 pos_err = inputs.target_pos - est_pos;
    
    Vec3 vel_des;
    vel_des.x() = pid_x.update(pos_err.x(), dt);
    vel_des.y() = pid_y.update(pos_err.y(), dt);
    vel_des.z() = pid_z.update(pos_err.z(), dt);

    Vec3 vel_err = vel_des - est_vel;

    Vec3 acc_des;
    acc_des.x() = pid_vx.update(vel_err.x(), dt);
    acc_des.y() = pid_vy.update(vel_err.y(), dt);
    acc_des.z() = pid_vz.update(vel_err.z(), dt);

    double thrust_force_z = MASS * (GRAVITY + acc_des.z());
    
    double throttle_base = thrust_force_z / (MASS * GRAVITY * 2.0); 
    throttle_base = std::clamp(throttle_base, 0.0, 1.0);

    
    double yaw = quatToEuler(est_ori).z();
    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);

    double acc_forward = acc_des.x() * cos_y + acc_des.y() * sin_y;
    double acc_left    = -acc_des.x() * sin_y + acc_des.y() * cos_y; // +Y is Left

    double pitch_des = acc_forward / GRAVITY;
    double roll_des  = acc_left / GRAVITY;  
    
    pitch_des = std::clamp(pitch_des, -MAX_TILT, MAX_TILT);
    roll_des  = std::clamp(roll_des, -MAX_TILT, MAX_TILT);

    Vec3 current_euler = quatToEuler(est_ori);
    
    double yaw_err = inputs.target_yaw - current_euler.z();
    if(yaw_err > M_PI) yaw_err -= 2*M_PI;
    if(yaw_err < -M_PI) yaw_err += 2*M_PI;

    double p_des = pid_roll.update(roll_des - current_euler.x(), dt);
    double q_des = pid_pitch.update(pitch_des - current_euler.y(), dt);
    double r_des = pid_yaw.update(yaw_err, dt);

    double cmd_roll_moment  = pid_p.update(p_des - est_gyro.x(), dt);
    double cmd_pitch_moment = pid_q.update(q_des - est_gyro.y(), dt);
    double cmd_yaw_moment   = pid_r.update(r_des - est_gyro.z(), dt);

    
    MotorMix mix;
    double t = throttle_base;
    double r = cmd_roll_moment;   // +r = Right Wing Down (+X Torque)
    double p = cmd_pitch_moment;  // +p = Nose Down (+Y Torque)
    double y = cmd_yaw_moment;    // +y = Nose Left (+Z Torque)


    // FR (0): Front, Right, CCW
    mix.throttle_01[0] = t - p - r - y; 
    
    // RL (1): Rear, Left, CCW
    mix.throttle_01[1] = t + p + r - y; 
    
    // FL (2): Front, Left, CW
    mix.throttle_01[2] = t - p + r + y; 
    
    // RR (3): Rear, Right, CW
    mix.throttle_01[3] = t + p - r + y; 

    for(auto& v : mix.throttle_01) v = std::clamp(v, 0.0, 1.0);

    return mix;
}