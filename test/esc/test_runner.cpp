/*
 * test_runner.cpp - Universal ESC "Digital Twin" Test Suite
 * Compile: g++ -std=c++17 test_runner.cpp esc.cpp motor.cpp propeller.cpp battery.cpp -o powertrain_test
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <tuple>
#include <algorithm>
#include "esc.h"
#include "motor.h"
#include "propeller.h"
#include "battery.h"

#ifndef TYPES_H
using Scalar = double;
#endif

// --- 1. THROTTLE PROFILE GENERATOR ---
Scalar get_throttle(Scalar t, const std::string& p) {
    // Basic Dynamics
    if (p == "step")          return (t > 0.1) ? 1.0 : 0.0;
    if (p == "ramp")          return std::min(1.0, t * 0.5);
    
    // Thermal & Endurance
    if (p == "thermal")       return 1.0; 
    if (p == "ripple")        return 0.5; // Max Ripple at 50% duty cycle
    
    // Protection Logic
    if (p == "desync_test")   return (t > 0.1) ? 1.0 : 0.0; // Fast step to trigger limiting
    if (p == "lvc_test")      return 1.0; 
    if (p == "soft_limit")    return (t > 0.1) ? 1.0 : 0.0; 
    if (p == "braking")       return (t < 0.5) ? 1.0 : 0.0; // Hard cut for regen
    
    // Signal Integrity
    if (p == "quantization")  return t * 0.2; // Slow ramp to see steps
    if (p == "jitter")        return 0.5;
    
    // Fault Injection
    if (p == "brownout")      return 0.5; 
    if (p == "gate_sag")      return 1.0; 
    if (p == "desync")        return (t < 1.0) ? t : 1.0; // Ramp into instability
    if (p == "latency")       return (t > 0.1) ? 1.0 : 0.0;
    if (p == "spike")         return (t > 0.1 && t < 0.2) ? 1.0 : 0.0;

    return 0.0;
}

// --- 2. BATTERY VOLTAGE OVERRIDE (FAULT INJECTION) ---
// Allows simulating external power supply faults (like a bench supply dying)
Scalar get_battery_override(Scalar t, Scalar nominal_v, const std::string& p) {
    if (p == "brownout") {
        // Dip to 2.5V (below MCU reset) at t=0.5s, recover at t=0.6s
        if (t > 0.5 && t < 0.6) return 2.5; 
    }
    if (p == "gate_sag") {
        // Ramp down from 16V to 4V to test gate drive efficiency loss
        return std::max(4.0, 16.0 - t * 5.0); 
    }
    return nominal_v;
}

int main(int argc, char* argv[]) {
    // --- DEFAULT CONFIGURATION ---
    
    // 1. Battery (Standard 4S 1500mAh)
    BatteryConfig b_cfg;
    b_cfg.cell_count = 4; b_cfg.capacity_mAh = 1500; b_cfg.internal_R_mOhm = 5.0; 
    b_cfg.mass_kg = 0.200; b_cfg.initial_charge_pct = 1.0; b_cfg.initial_temp_K = 298.15;

    // 2. Motor (2400Kv Racing Motor)
    MotorConfig m_cfg;
    m_cfg.Kv = 2400; m_cfg.Rm_nominal = 0.045; m_cfg.I0 = 1.2; m_cfg.I_stall = 150.0;
    m_cfg.inductance_H = 0.00003; m_cfg.pole_pairs = 7; m_cfg.cogging_torque_Nm = 0.02;
    m_cfg.MoI = 0.00003; m_cfg.viscous_damping = 0.0001; 
    m_cfg.mass_kg = 0.035; m_cfg.specific_heat_JkgK = 500.0;
    m_cfg.thermal_conduct_W_K = 0.5; m_cfg.cooling_rpm_factor = 0.005; m_cfg.initial_temp_K = 298.15;

    // 3. ESC (Advanced 40A BLHeli_32 style)
    EscParams e_cfg;
    e_cfg.max_current_A = 40.0; e_cfg.max_voltage_V = 26.0; e_cfg.internal_R_Ohms = 0.003;   
    e_cfg.temp_coeff_r = 0.004; 
    
    // Protection
    e_cfg.hard_ocp_limit_A = 100.0; // Breaker
    e_cfg.soft_current_limit_A = 60.0; // Active Governor
    e_cfg.ocp_delay_s = 0.05;
    e_cfg.protection_voltage_V = 17.5; // Regen Clamp
    
    // LVC
    e_cfg.lvc_voltage_V = 13.2; e_cfg.lvc_soft_range_V = 1.0; e_cfg.lvc_hysteresis_recovery = 0.5;
    
    // Logic & Gate
    e_cfg.mcu_brownout_V = 3.0; e_cfg.mcu_boot_time_s = 1.5;
    e_cfg.gate_drive_min_V = 10.0; e_cfg.gate_drive_sensitivity = 0.5; e_cfg.switching_speed_sensitivity = 2.0;

    // Caps & Cables
    e_cfg.cap_capacitance_F = 0.00047; e_cfg.cap_esr_Ohms = 0.05; e_cfg.cap_temp_coeff_esr = 0.01;
    e_cfg.cap_mass_kg = 0.002; e_cfg.cap_thermal_conduct_W_K = 0.05;
    e_cfg.cable_resistance_Ohms = 0.01; e_cfg.cable_inductance_H = 0.000005;

    // Firmware / Signal
    e_cfg.est_motor_Kv = 2400; e_cfg.est_motor_R = 0.045;
    e_cfg.signal_deadband = 0.05; e_cfg.signal_jitter = 0.002; e_cfg.input_resolution = 2000.0;
    e_cfg.input_delay_s = 0.002; e_cfg.active_freewheeling = true; e_cfg.diode_drop_V = 0.7;
    e_cfg.bemf_cutoff_rpm = 2000.0; e_cfg.startup_jitter = 0.3; e_cfg.desync_noise_threshold = 0.2;
    e_cfg.slew_rate_limit_V_s = 2000.0; e_cfg.dead_time_loss_V = 0.2;
    
    e_cfg.mass_kg = 0.015; e_cfg.specific_heat_JkgK = 900.0; 
    e_cfg.thermal_conduct_W_K = 1.5; e_cfg.initial_temp_K = 298.15; e_cfg.temp_cutoff_K = 373.15;    

    // --- CLI PARSING & OVERRIDES ---
    std::string profile = "step";
    if (argc >= 2) profile = argv[1];
    Scalar duration = 1.0;
    if (argc >= 3) duration = std::stod(argv[2]);

    // Apply Test-Specific Configurations
    if (profile == "lvc_test") {
        b_cfg.capacity_mAh = 100; // Tiny battery to drain fast
    }
    if (profile == "quantization") {
        e_cfg.input_resolution = 50.0; // Low res to show steps
        e_cfg.signal_jitter = 0.0;
        e_cfg.signal_deadband = 0.0;
    }
    if (profile == "ripple") {
        e_cfg.cap_esr_Ohms = 0.1; // Bad caps to show heating
        e_cfg.cap_thermal_conduct_W_K = 0.01;
    }
    if (profile == "desync_test") {
        // Mismatch config to trigger bad behavior
        e_cfg.est_motor_Kv = 1000; 
    }
    if (profile == "desync") {
        // High resistance motor creates more noise
        m_cfg.Rm_nominal = 0.15; 
        // Sensitive ESC
        e_cfg.desync_noise_threshold = 0.3;
    }
    if (profile == "latency") {
        e_cfg.input_delay_s = 0.005; // 5ms lag
    }
    if (profile == "soft_limit") {
        e_cfg.soft_current_limit_A = 50.0;
    }

    auto battery = std::make_unique<LiPoBattery>(b_cfg);
    auto motor = std::make_unique<BLDCMotor>(m_cfg);
    auto esc = std::make_unique<ESC>(e_cfg);
    auto prop = std::make_unique<Propeller>(PropParams{5.0, 4.3, 0.12, 0.06});

    // --- SIMULATION LOOP ---
    Scalar dt = 0.0001; // 10kHz simulation
    
    // Header for CSV
    std::cout << "time,throttle,batt_v,esc_in_v,esc_out_v,current,rpm,"
              << "esc_temp,cap_temp,is_shutdown,is_limiting,is_stalled,"
              << "is_braking,is_rebooting,resistance,efficiency\n";

    int c = 0;
    for (Scalar t = 0; t <= duration; t += dt) {
        Scalar throttle = get_throttle(t, profile);
        
        // 1. Physics Updates
        prop->update(dt, motor->getRPM(), 0.0);
        auto aero = prop->getAerodynamics(motor->getRPM(), 0.0, 100.0);
        
        battery->update(motor->getCurrent(), dt, 298.15);
        
        // 2. Voltage Fault Injection
        Scalar v_supply = battery->getVoltage();
        if (profile == "brownout" || profile == "gate_sag") {
            v_supply = get_battery_override(t, v_supply, profile);
        }

        // 3. Component Updates
        Scalar v_applied = esc->update(dt, throttle, v_supply, motor->getRPM(), motor->getCurrent());
        motor->update(dt, v_applied, std::get<1>(aero), 298.15);

        // 4. Logging (Decimate to ~200Hz to save file size)
        if (c++ % 50 == 0) {
            std::cout << t << "," << throttle << "," << v_supply << ","
                      << esc->getInputVoltage() << "," << esc->getVoltage() << ","
                      << motor->getCurrent() << "," << motor->getRPM() << ","
                      << esc->getTemperature() << "," << esc->getCapTemperature() << ","
                      << esc->isShutdown() << "," << esc->isLimitingCurrent() << ","
                      << esc->isStalled() << "," << esc->isBrakingLimited() << ","
                      << esc->isRebooting() << "," << esc->getDynamicResistance()*1000.0 << ","
                      << esc->getEfficiency() << "\n";
        }
    }
    return 0;
}