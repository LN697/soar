#pragma once

#include "types.h"

struct MotorParams {
    Scalar Kv;          // RPM/Volt
    Scalar Rm;          // Internal resistance (Ohms)
    Scalar I0;          // No-load current
    Scalar MoI;         // Rotor inertia
};

class Motor {
public:
    Motor(const MotorParams& params);
    
    // Step physics: Input Voltage + Load (Air Resistance) -> Output new RPM & Current
    void update(Scalar dt, Scalar v_in, Scalar load_torque_Nm);
    
    Scalar getRPM() const;
    Scalar getCurrent() const { return state_current; }

private:
    const MotorParams params;
    Scalar state_omega = 0.0;   // rad/s
    Scalar state_current = 0.0; // Amps
};
