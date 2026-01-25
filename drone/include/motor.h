#pragma once

#include "types.h"

struct MotorParams {
    Scalar Kv;          // Velocity constant (RPM/Volt)
    Scalar Rm;          // Internal Resistance (Ohms)
    Scalar I0;          // No-load current (Amps)
    Scalar MoI;         // Rotor Moment of Inertia (kg * m^2)
    Scalar dragCoeff;   // Propeller drag factor (relates RPM to Torque Load)
};

class Motor {
    public:
        Motor(const MotorParams& params);
        
        std::pair<Scalar, Scalar> step(Scalar dt, Scalar v_in);
        
        Scalar getRPM() const { return state_omega * 9.5493; }  // rad/s to RPM
        Scalar getTemp() const { return state_temp; }
        Scalar getCurrent() const { return state_current; }

    private:
        const MotorParams params;

        // Internal State
        Scalar state_omega   = 0.0;     // Angular Velocity (rad/s)
        Scalar state_current = 0.0;     // Current (Amps) - modeled for battery sag
        Scalar state_temp    = 293.15;  // Temperature (Kelvin) - starts at 20C
};
