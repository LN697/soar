#pragma once

#include <vector>
#include <array>
#include "types.h"
#include "motor.h"
#include "battery.h"
#include "propeller.h"
#include "esc.h"
#include "pid.h"

struct Telemetry {
    double time;
    double voltage_battery;
    double current_total;
    double rpm_motor_0;
    
    Vec3 position;
    Vec3 velocity;
    Quat orientation;
    
    Vec3 force_thrust_world;
    Vec3 force_gravity;
    Vec3 force_wind;
};

struct DroneState {
    Vec3 pos = Vec3::Zero();
    Vec3 vel = Vec3::Zero();
    Quat ori = Quat::Identity();
    Vec3 ang = Vec3::Zero(); 
};

class Drone {
    public:
        Drone();

        // New High-Level Control Interface
        // Returns the throttle command (Voltage) it decided to use
        Scalar updateAltitude(Scalar dt, Scalar target_altitude);

        // Physics Step
        void step(Scalar dt, const std::array<Scalar, 4>& inputs, const Vec3& wind_vector = Vec3::Zero());

        Telemetry getTelemetry(double t) const;

    private:
        Battery battery;
        std::vector<Motor> motors;
        std::vector<Propeller> propellers;
        std::vector<ESC> escs;

        // Embedded Flight Controller
        PID altitudePID; 

        DroneState s;

        const Scalar MASS = 1.0;
        const Scalar ARM_LENGTH = 0.25;
        const Mat3 INERTIA_TENSOR;

        // Visualization Helpers
        Vec3 last_wind_vector = Vec3::Zero();
        Vec3 last_thrust_vector = Vec3::Zero();
};
