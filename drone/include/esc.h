#pragma once

#include "types.h"
#include <algorithm>
#include <random>
#include <deque>

struct EscParams {
    // --- Electrical ---
    Scalar max_current_A;       
    Scalar max_voltage_V;       
    Scalar internal_R_Ohms;     
    Scalar temp_coeff_r;        

    // --- Logic & Gate Drive (New Physics) ---
    Scalar mcu_brownout_V;          // Voltage below which MCU resets (e.g. 3.0V)
    Scalar mcu_boot_time_s;         // Reboot delay (e.g. 1.5s)
    Scalar gate_drive_min_V;        // Voltage required for full gate saturation (e.g. 10V)
    Scalar gate_drive_sensitivity;  // Resistance increase per volt sag (e.g. 0.5)
    Scalar switching_speed_sensitivity; // Switching loss multiplier per volt sag (e.g. 2.0)

    // --- Capacitor Bank ---
    Scalar cap_capacitance_F;   
    Scalar cap_esr_Ohms;        
    Scalar cap_temp_coeff_esr;  
    Scalar cap_mass_kg;         
    Scalar cap_thermal_conduct_W_K; 

    // --- Protection ---
    Scalar protection_voltage_V;    // Regenerative braking voltage clamp (e.g. 26V)
    Scalar soft_current_limit_A;    // Active Governor: Throttles down to hold this current (e.g. 50A)
    Scalar hard_ocp_limit_A;        // Circuit Breaker: Shuts down if exceeded (e.g. 80A)
    Scalar ocp_delay_s;             // Time allowed above hard limit before trip
    
    // --- LVC (Low Voltage Cutoff) ---
    Scalar lvc_voltage_V;       
    Scalar lvc_soft_range_V;    
    Scalar lvc_hysteresis_recovery;
    
    // --- Cable Impedance ---
    Scalar cable_resistance_Ohms; 
    Scalar cable_inductance_H;    

    // --- Topology ---
    bool active_freewheeling;   
    Scalar diode_drop_V;        
    
    // --- Sensorless Sensing ---
    Scalar bemf_cutoff_rpm;     
    Scalar startup_jitter;
    Scalar desync_noise_threshold;  // SNR ratio below which desync occurs (e.g. 0.3)
    
    // --- Firmware / Signal ---
    Scalar est_motor_Kv;        
    Scalar est_motor_R;         
    Scalar signal_deadband;     
    Scalar signal_jitter;       
    Scalar input_resolution;    
    Scalar input_delay_s;       
    Scalar timing_advance_deg;  
    
    // --- Dynamics ---
    Scalar slew_rate_limit_V_s; 
    Scalar dead_time_loss_V;    
    
    // --- Thermal ---
    Scalar mass_kg;             
    Scalar specific_heat_JkgK;  
    Scalar thermal_conduct_W_K; 
    Scalar initial_temp_K;      
    Scalar temp_cutoff_K;       
};

class ESC {
public:
    ESC(const EscParams& params);

    Scalar update(Scalar dt, 
                  Scalar throttle_signal, 
                  Scalar battery_voltage, 
                  Scalar motor_rpm, 
                  Scalar motor_current);

    // --- Telemetry ---
    Scalar getTemperature() const { return temperature_K; }
    Scalar getCapTemperature() const { return cap_temperature_K; }
    Scalar getVoltage() const { return current_output_voltage; }
    Scalar getPowerLoss() const { return power_loss_W; }
    Scalar getEfficiency() const { return efficiency; }
    Scalar getInputVoltage() const { return esc_input_voltage; } 
    Scalar getDynamicResistance() const { return current_R_ohms; }
    
    // --- Status Flags ---
    bool isShutdown() const { return ocp_latched; }
    bool isLVCActive() const { return lvc_factor < 0.99; }
    bool isLimitingCurrent() const { return current_limiter_active; }
    bool isStalled() const { return comm_loss; } 
    bool isBrakingLimited() const { return braking_limited; }
    bool isRebooting() const { return reboot_timer > 0; }

private:
    const EscParams params;
    
    // State
    Scalar temperature_K;
    Scalar current_output_voltage;
    Scalar power_loss_W;
    Scalar efficiency;
    Scalar current_R_ohms;
    Scalar cap_temperature_K;
    Scalar current_cap_esr;
    Scalar esc_input_voltage; 
    Scalar prev_motor_current; 
    
    // Logic State
    Scalar ocp_timer;
    bool ocp_latched;
    Scalar lvc_factor;
    bool comm_loss;
    bool braking_limited;
    Scalar reboot_timer;
    bool current_limiter_active;
    Scalar desync_integrator;

    // Buffers
    std::deque<std::pair<Scalar, Scalar>> input_buffer; 
    Scalar internal_time;

    // Noise Generators
    mutable std::default_random_engine generator;
    mutable std::normal_distribution<Scalar> switching_noise;
    mutable std::normal_distribution<Scalar> input_noise;
    mutable std::normal_distribution<Scalar> bemf_noise;

    Scalar calculatePowerLoss(Scalar current, Scalar rpm, Scalar duty_cycle, Scalar input_v);
    Scalar getDelayedInput(Scalar dt, Scalar raw_signal);
};