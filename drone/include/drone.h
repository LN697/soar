#pragma once

#include <vector>
#include <array>
#include "types.h"
#include "motor.h"
#include "battery.h"
#include "propeller.h"
#include "esc.h"
#include "imu.h"
#include "estimator.h"
#include "controller.h"

struct Telemetry {
    double time;
    
    // True State
    Vec3 pos_true;
    Vec3 vel_true;
    Quat ori_true;
    
    // Estimated State
    Vec3 pos_est;
    Vec3 vel_est;
    Quat ori_est;
    
    // Targets
    Vec3 target_pos;

    // Physics Debug
    Vec3 force_drag;
    Vec3 moment_gyro;

    std::array<double,4> motor_rpm;
    std::array<double,4> motor_current;
};

class Drone {
    public:
        Drone();

        // High Level Step (Runs control + physics)
        void update(Scalar dt, const ControlInputs& inputs);

        Telemetry getTelemetry(double t) const;

    private:
        // Components
        Battery battery;
        std::vector<Motor> motors;
        std::vector<Propeller> propellers;
        std::vector<ESC> escs;
        
        // Systems
        IMU imu;
        Estimator estimator;
        Controller controller;

        // State
        Vec3 pos = Vec3::Zero();
        Vec3 vel = Vec3::Zero();
        Quat ori = Quat::Identity();
        Vec3 ang_vel = Vec3::Zero(); // Body Rates

        // Configuration
        const Scalar MASS = 1.0;
        const Scalar ARM_LENGTH = 0.25; // Center to motor
        const Mat3 INERTIA_TENSOR;
        
        // Telemetry Cache
        ControlInputs last_inputs;
        Vec3 last_drag = Vec3::Zero();
        Vec3 last_gyro_moment = Vec3::Zero();
};