#include "motor.h"
#include <cmath>
#include <iostream> 
#include <algorithm> // for std::max, std::min

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Motor::Motor(const MotorParams& p) : params(p) {}

void Motor::update(Scalar dt, Scalar v_in, Scalar load_torque_Nm) {
    // 1. Constants
    Scalar Kv_SI = params.Kv * (2.0 * M_PI / 60.0);
    Scalar Kt = 1.0 / Kv_SI; 

    // 2. Electrical Model (Pure Physics)
    Scalar back_emf = state_omega * Kt; 
    
    // I = (V - EMF) / R
    Scalar effective_voltage = v_in - back_emf;
    state_current = effective_voltage / params.Rm;

    // 3. Mechanical Model
    Scalar torque_motor = Kt * state_current;
    
    // Friction Model
    Scalar torque_friction = params.I0 * Kt;
    if (state_omega > 0) torque_friction *= 1.0; 
    else torque_friction = 0;

    Scalar torque_net = torque_motor - load_torque_Nm - torque_friction;

    // 4. Integration
    Scalar alpha = torque_net / params.MoI;
    state_omega += alpha * dt;

    if (state_omega < 0) state_omega = 0;
}

Scalar Motor::getRPM() const {
    return state_omega * 60.0 / (2.0 * M_PI);
}