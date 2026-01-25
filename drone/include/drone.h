#pragma once

#include <vector>
#include "types.h"
#include "motor.h"

struct Telemetry {
    double time;
    double voltage_in;
    double current_total;
    double rpm_motor_0;
    double thrust_total;
    double pos_z;
    double vel_z;
    double accel_z_body;
};

struct DroneState {
    Vec3 pos = Vec3::Zero();        // Position in 3D space (m)
    Vec3 vel = Vec3::Zero();        // Velocity (m/s)
    Quat ori = Quat::Identity();    // Quaternions
    Vec3 ang = Vec3::Zero();        // Angular velocity (rad/s)
};

class Drone {
    public:
        Drone(); 
        
        void step(Scalar dt, const std::array<Scalar, 4>& inputs, const Vec3& wind_vector);

        const DroneState& getState() const { return s; }

        Telemetry getTelemetry(double t, double voltage_in) const;

    private:
        const Scalar MASS = 1.0;
        const Mat3 INERTIA_TENSOR; 
        const Scalar ARM_LENGTH = 0.25; 
        
        std::vector<Motor> motors; 

        DroneState s;

        // Internal Helpers (Optional, can be removed if not defined yet)
        // Vec3 calculateGravityForce();
        // Vec3 calculateAerodynamicDrag(const Vec3& wind);
};
