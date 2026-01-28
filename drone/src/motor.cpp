#include "motor.h"
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

BLDCMotor::BLDCMotor(const MotorConfig& cfg) 
    : config(cfg), 
      state_omega(0.0), 
      state_current(0.0), 
      state_angle(0.0),
      temperature_K(cfg.initial_temp_K),
      current_torque(0.0),
      efficiency(0.0) 
{
    if (config.initial_temp_K <= 0) temperature_K = 298.15; 
    if (config.inductance_H <= 1e-9) const_cast<Scalar&>(config.inductance_H) = 0.00001;
    if (config.pole_pairs < 1) const_cast<Scalar&>(config.pole_pairs) = 7; 

    // Noise Configuration
    // RPM Noise: +/- 50 RPM jitter (common for Hall/EMF sensors)
    noise_rpm = std::normal_distribution<Scalar>(0.0, 50.0);
    // Current Noise: +/- 0.5A noise (PWM switching noise)
    noise_current = std::normal_distribution<Scalar>(0.0, 0.5); 
}

void BLDCMotor::update(Scalar dt, Scalar v_in, Scalar load_torque_Nm, Scalar ambient_temp_K) {
    // 1. Constants
    Scalar Kv_rads = config.Kv * (2.0 * M_PI / 60.0);
    Scalar Kt = 1.0 / Kv_rads; 
    Scalar R_dynamic = calculateDynamicResistance();

    // 2. Electrical (RL Circuit)
    Scalar back_emf = state_omega * Kt; 
    Scalar voltage_across_inductor = v_in - (state_current * R_dynamic) - back_emf;
    Scalar di_dt = voltage_across_inductor / config.inductance_H;
    state_current += di_dt * dt;

    // 3. Thermal
    Scalar power_heat_W = state_current * state_current * R_dynamic;
    Scalar convection_factor = 1.0 + (state_omega * config.cooling_rpm_factor);
    Scalar cooling_rate = config.thermal_conduct_W_K * convection_factor;
    Scalar net_thermal = power_heat_W - (temperature_K - ambient_temp_K) * cooling_rate;
    temperature_K += (net_thermal * dt) / (config.mass_kg * config.specific_heat_JkgK);

    // 4. Mechanical (Torque & Saturation)
    Scalar torque_generated = Kt * state_current;
    if (std::abs(state_current) > config.I_stall * 0.8) {
        Scalar ratio = std::abs(state_current) / (config.I_stall * 0.8);
        torque_generated *= (1.0 / (1.0 + 0.1 * (ratio - 1.0)));
    }
    current_torque = torque_generated;

    // Cogging & Friction
    Scalar elec_angle = state_angle * config.pole_pairs;
    Scalar torque_cogging = config.cogging_torque_Nm * std::sin(6.0 * elec_angle);
    
    Scalar t_static = config.I0 * Kt;
    Scalar t_viscous = config.viscous_damping * state_omega;
    Scalar t_windage = 1.0e-7 * (state_omega * state_omega); // Fan drag
    Scalar t_drag = t_static + t_viscous + t_windage;

    Scalar torque_net = torque_generated + torque_cogging - load_torque_Nm;
    
    // Stiction Logic
    if (std::abs(state_omega) < 1.0 && std::abs(torque_net) < t_static) {
        torque_net = 0; state_omega = 0;
    } else {
        torque_net -= (state_omega > 0 ? t_drag : -t_drag);
    }

    // Integration
    Scalar alpha = torque_net / config.MoI;
    state_omega += alpha * dt;
    if (state_omega < 0) state_omega = 0;

    state_angle += state_omega * dt;
    if (state_angle > 2.0 * M_PI) state_angle -= 2.0 * M_PI;

    // Efficiency
    Scalar p_out = torque_generated * state_omega;
    Scalar p_in = v_in * state_current;
    efficiency = (p_in > 0.001) ? (p_out / p_in) : 0.0;
}

Scalar BLDCMotor::calculateDynamicResistance() const {
    const Scalar ALPHA_CU = 0.00393;
    const Scalar REF_TEMP_K = 293.15;
    Scalar delta_T = temperature_K - REF_TEMP_K;
    return std::max(config.Rm_nominal * 0.5, config.Rm_nominal * (1.0 + ALPHA_CU * delta_T));
}

Scalar BLDCMotor::getRPM() const {
    return state_omega * 60.0 / (2.0 * M_PI);
}

// --- NOISY SENSORS ---
Scalar BLDCMotor::getRPM_Measured() const {
    Scalar truth = getRPM();
    // Add Noise & Quantize to 10 RPM steps
    Scalar noisy = truth + noise_rpm(generator);
    return std::round(noisy / 10.0) * 10.0;
}

Scalar BLDCMotor::getCurrent_Measured() const {
    Scalar truth = state_current;
    // Add Noise & Quantize to 0.01A steps
    Scalar noisy = truth + noise_current(generator);
    return std::round(noisy * 100.0) / 100.0;
}