/*
 * test_runner.cpp - Decoupled Motor & Propeller Physics Test
 * Compile: g++ -std=c++17 test_runner.cpp motor.cpp propeller.cpp -o propulsion_test
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <tuple>
#include <algorithm>
#include "motor.h"
#include "propeller.h"

#ifndef TYPES_H
using Scalar = double;
#endif

// --- TEST PROFILE GENERATORS ---

// 1. Voltage Source (Ideal)
Scalar get_voltage(Scalar t, const std::string& p) {
    if (p == "step")      return (t > 0.05) ? 16.0 : 0.0;
    if (p == "ramp")      return std::min(16.0, t * 2.0); // 0-16V in 8s
    if (p == "stall")     return 16.0; // Max voltage immediately
    if (p == "cogging")   return 0.5;  // Just enough to overcome static friction
    if (p == "mach")      return std::min(30.0, t * 5.0); // Overdrive to 30V
    if (p == "thermal")   return 16.0; // Endurance
    if (p == "noise")     return 12.0; // Steady Hover voltage
    return 16.0; // Default
}

// 2. Airspeed (Forward Velocity)
Scalar get_airspeed(Scalar t, const std::string& p) {
    if (p == "h_force")   return t * 8.0; // 0 to 40+ m/s
    if (p == "dyn_thrust") return t * 5.0;
    return 0.0;
}

// 3. Vertical Velocity (Descent/Climb)
Scalar get_vertical_vel(Scalar t, const std::string& p) {
    if (p == "vrs") {
        if (t < 1.0) return 0.0;
        return -(t - 1.0) * 2.5; // Accelerating descent (0 to -10 m/s)
    }
    return 0.0;
}

// 4. Altitude (Ground Effect)
Scalar get_height(Scalar t, const std::string& p) {
    if (p == "ground") {
        // Start at 0m, slowly climb to 2m
        return t * 0.2; 
    }
    return 100.0; // High altitude (Out of ground effect)
}

int main(int argc, char* argv[]) {
    // --- CONFIGURATION ---
    
    // Motor: High Performance 2306 2400Kv
    MotorConfig m_cfg;
    m_cfg.Kv = 2400; 
    m_cfg.Rm_nominal = 0.045; // 45 mOhm
    m_cfg.I0 = 1.2; 
    m_cfg.I_stall = 150.0;
    m_cfg.inductance_H = 0.00003; // 30uH
    m_cfg.pole_pairs = 7; 
    m_cfg.cogging_torque_Nm = 0.025;
    m_cfg.MoI = 0.00003; 
    m_cfg.viscous_damping = 0.0001; 
    m_cfg.mass_kg = 0.035; 
    m_cfg.specific_heat_JkgK = 500.0; 
    m_cfg.thermal_conduct_W_K = 0.5; 
    m_cfg.cooling_rpm_factor = 0.005; 
    m_cfg.initial_temp_K = 298.15; // 25C

    // Propeller: 5x4.3x3
    PropParams p_cfg;
    p_cfg.diameter_inch = 5.0; 
    p_cfg.pitch_inch = 4.3;
    p_cfg.thrust_coeff_static = 0.12; 
    p_cfg.power_coeff_static = 0.06;
    p_cfg.ct_decay_linear = 0.15; 
    p_cfg.cp_decay_linear = 0.08;
    p_cfg.mach_limit_start = 0.6; // Tips lose efficiency at Mach 0.6
    p_cfg.mach_limit_slope = 2.0; 
    p_cfg.ground_effect_factor = 1.4; 
    p_cfg.tau_flow_sec = 0.08; // Dynamic Inflow Lag

    auto motor = std::make_unique<BLDCMotor>(m_cfg);
    auto prop = std::make_unique<Propeller>(p_cfg);

    // --- CLI PARSING ---
    std::string profile = "step";
    if (argc >= 2) profile = argv[1];
    Scalar duration = 2.0;
    if (argc >= 3) duration = std::stod(argv[2]);

    // High fidelity time step for inductance/noise
    Scalar dt = 0.0001; // 0.1ms (10kHz simulation)

    // Output Header
    std::cout << "time,voltage,rpm_truth,rpm_sens,current_truth,current_sens,"
              << "thrust_N,torque_Nm,temp_K,efficiency,h_force_N,mach,angle_rad,"
              << "env_airspeed,env_vel_z,env_height\n";

    // --- MAIN LOOP ---
    for (Scalar t = 0; t <= duration; t += dt) {
        // 1. Get Environmental Conditions
        Scalar v_in = get_voltage(t, profile);
        Scalar airspeed = get_airspeed(t, profile);
        Scalar vel_z = get_vertical_vel(t, profile);
        Scalar h = get_height(t, profile);
        
        // 2. Propeller Physics (Aerodynamics)
        // Update airflow state (lag)
        prop->update(dt, motor->getRPM(), vel_z);
        
        // Calculate Loads
        // Note: For Stall test, we force RPM to 0 in physics or assume prop is locked?
        // Actually, 'stall' usually means the prop is stuck. 
        // We can simulate this by overriding the load torque to be infinite if profile is 'stall'
        Scalar load_torque = 0.0;
        Scalar thrust = 0.0;
        Scalar h_force = 0.0;
        
        if (profile == "stall") {
            load_torque = 1000.0; // Infinite load (Locked Rotor)
            thrust = 0.0;
        } else {
            // Normal Prop Physics
            Scalar total_axial = airspeed + vel_z; 
            // If H-Force or Dyn Thrust test, assume airspeed is relevant inflow
            if (profile == "vrs") total_axial = vel_z;
            
            auto aero = prop->getAerodynamics(motor->getRPM(), total_axial, h);
            thrust = std::get<0>(aero);
            load_torque = std::get<1>(aero);
            h_force = std::get<2>(aero);
        }

        // 3. Motor Physics (Electromechanical)
        // Use ideal voltage source (no battery sag)
        motor->update(dt, v_in, load_torque, 298.15); // Ambient 25C

        // 4. Telemetry Logging (Decimate to 200Hz for CSV size)
        static int c = 0;
        if (c++ % 50 == 0) {
            // Calculate Mach
            Scalar tipspeed = (motor->getRPM() * 0.10472 * 0.0635) + airspeed; 
            Scalar mach = tipspeed / 343.0;

            std::cout << t << "," << v_in << "," 
                      << motor->getRPM() << "," << motor->getRPM_Measured() << ","
                      << motor->getCurrent() << "," << motor->getCurrent_Measured() << ","
                      << thrust << "," << load_torque << "," 
                      << motor->getTemperature() << "," << motor->getEfficiency() << ","
                      << h_force << "," << mach << "," 
                      << motor->getPositionAngle() << ","
                      << airspeed << "," << vel_z << "," << h << "\n";
        }
    }

    return 0;
}