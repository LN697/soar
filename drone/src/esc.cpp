#include "esc.h"

#include <cmath>
#include <iostream>

ESC::ESC(const EscParams& p) 
    : params(p), 
      temperature_K(p.initial_temp_K),
      current_output_voltage(0.0),
      power_loss_W(0.0),
      efficiency(1.0),
      current_R_ohms(p.internal_R_Ohms),
      cap_temperature_K(p.initial_temp_K),
      current_cap_esr(p.cap_esr_Ohms),
      esc_input_voltage(0.0),
      prev_motor_current(0.0),
      ocp_timer(0.0),
      ocp_latched(false),
      lvc_factor(1.0),
      comm_loss(false),
      braking_limited(false),
      reboot_timer(0.0),
      current_limiter_active(false),
      desync_integrator(0.0),
      internal_time(0.0) {
    if (temperature_K <= 0) temperature_K = 298.15;
    switching_noise = std::normal_distribution<Scalar>(0.0, 0.05); 
    input_noise = std::normal_distribution<Scalar>(0.0, p.signal_jitter);
    bemf_noise = std::normal_distribution<Scalar>(0.0, 1.0);
}

Scalar ESC::getDelayedInput(Scalar dt, Scalar raw_signal) {
    internal_time += dt;
    input_buffer.push_back({internal_time, raw_signal});
    Scalar delayed_val = raw_signal; 
    
    while (!input_buffer.empty()) {
        if (internal_time - input_buffer.front().first >= params.input_delay_s) {
            delayed_val = input_buffer.front().second;
            if (input_buffer.size() > 1) input_buffer.pop_front();
            else break; 
        } else {
            delayed_val = input_buffer.front().second;
            break; 
        }
    }
    return delayed_val;
}

Scalar ESC::update(Scalar dt, 
                   Scalar throttle_signal, 
                   Scalar battery_voltage, 
                   Scalar motor_rpm, 
                   Scalar motor_current) {
    Scalar di_dt = (motor_current - prev_motor_current) / dt;
    prev_motor_current = motor_current;
    
    Scalar cable_drop_R = motor_current * params.cable_resistance_Ohms;
    Scalar cable_drop_L = params.cable_inductance_H * di_dt;
    
    Scalar cap_delta_T = cap_temperature_K - 298.15;
    current_cap_esr = params.cap_esr_Ohms * (1.0 + params.cap_temp_coeff_esr * cap_delta_T);
    Scalar damping_factor = 1.0 / (1.0 + (0.01 / current_cap_esr));

    esc_input_voltage = battery_voltage - cable_drop_R - (cable_drop_L * damping_factor);
    
    if (esc_input_voltage < params.mcu_brownout_V) {
        reboot_timer = params.mcu_boot_time_s;
    }
    if (reboot_timer > 0.0) {
        reboot_timer -= dt; 
        current_output_voltage = 0.0; 
        power_loss_W = 0.0; 
        efficiency = 0.0;
        return 0.0; 
    }
    
    if (ocp_latched) { 
        current_output_voltage = 0.0; 
        power_loss_W = 0.0; 
        return 0.0; 
    }
    if (std::abs(motor_current) > params.hard_ocp_limit_A) {
        ocp_timer += dt; 
        if(ocp_timer > params.ocp_delay_s) ocp_latched = true;
    } else {
        ocp_timer = std::max(0.0, ocp_timer - dt);
    }
    
    Scalar current_limit_factor = 1.0;
    current_limiter_active = false;
    if (std::abs(motor_current) > params.soft_current_limit_A) {
        current_limiter_active = true;
        Scalar excess = std::abs(motor_current) - params.soft_current_limit_A;
        // Simple P-controller: Reduce throttle by 5% per Amp over limit
        current_limit_factor = std::max(0.0, 1.0 - (excess * 0.05)); 
    }

    // --- 5. LOW VOLTAGE CUTOFF (LVC) ---
    Scalar lvc_target = 1.0;
    if (esc_input_voltage < (params.lvc_voltage_V + params.lvc_soft_range_V)) {
        Scalar sag = (params.lvc_voltage_V + params.lvc_soft_range_V) - esc_input_voltage;
        lvc_target = std::max(0.0, 1.0 - (sag / params.lvc_soft_range_V));
    }
    // Hysteresis: Drop fast, recover slow
    if(lvc_target < lvc_factor) lvc_factor = lvc_target;
    else { 
        lvc_factor += params.lvc_hysteresis_recovery * dt; 
        lvc_factor = std::min(lvc_factor, 1.0); 
    }

    // --- 6. TARGET VOLTAGE CALCULATION ---
    // Pipeline: Input -> Delay -> Noise -> Deadband -> Quantization -> Limits
    Scalar delayed_throttle = getDelayedInput(dt, throttle_signal);
    Scalar noisy = delayed_throttle + input_noise(generator);
    if(noisy < params.signal_deadband) noisy = 0.0;
    Scalar quantized = std::floor(noisy * params.input_resolution) / params.input_resolution;
    
    // Thermal Throttling
    Scalar thermal_lim = 1.0;
    if (temperature_K > params.temp_cutoff_K) {
        Scalar ovr = temperature_K - params.temp_cutoff_K;
        thermal_lim = std::max(0.0, 1.0 - (ovr / 15.0));
    }
    
    // Combine Limits: Throttle * Thermal * LVC * CurrentGovernor
    Scalar final_throttle = std::clamp(quantized, 0.0, 1.0) 
                          * thermal_lim * lvc_factor * current_limit_factor;
    
    Scalar target_voltage = final_throttle * esc_input_voltage;

    // --- 7. DESYNC PHYSICS (SNR & Commutation Loss) ---
    // Signal = RPM (Back-EMF strength). Noise = Current * Resistance + Switching Jitter.
    // Desync occurs if Noise overwhelms Signal for > 50ms.
    Scalar bemf_signal = std::abs(motor_rpm) / params.est_motor_Kv;
    Scalar system_noise = std::abs(motor_current) * current_R_ohms * 10.0; 
    
    Scalar snr = (bemf_signal + 0.1) / (system_noise + 0.1); // Avoid div/0
    
    if (snr < params.desync_noise_threshold) {
        desync_integrator += dt;
        if (desync_integrator > 0.05) comm_loss = true; // Desync Triggered
    } else {
        desync_integrator = std::max(0.0, desync_integrator - dt);
        comm_loss = false;
    }
    
    // Failure Mode: Random voltage cutouts ("Stutter") if desync active
    if (comm_loss) target_voltage *= (0.5 + 0.5 * bemf_noise(generator)); 

    // --- 8. REGENERATIVE BRAKING CLAMP ---
    // If generating (I < 0) and Bus Voltage > Protection Limit, clamp braking force.
    braking_limited = false;
    if (motor_current < -1.0 && esc_input_voltage > params.protection_voltage_V) {
        braking_limited = true;
        Scalar excess = esc_input_voltage - params.protection_voltage_V;
        // Reduce "braking ability" by raising the minimum allowed target voltage
        Scalar brake_reduction = std::max(0.0, 1.0 - (excess * 2.0));
        Scalar min_bemf = (motor_rpm / params.est_motor_Kv);
        
        // Target voltage cannot go fully to 0; must stay near BEMF to reduce braking current
        target_voltage = std::max(target_voltage, min_bemf * (1.0 - brake_reduction));
    }

    // --- 9. OUTPUT PHYSICS (Slew Rate + Gate Drive + Timing) ---
    // Slew Rate
    Scalar max_change = params.slew_rate_limit_V_s * dt;
    Scalar delta_v = target_voltage - current_output_voltage;
    if (std::abs(delta_v) > max_change) current_output_voltage += (delta_v > 0 ? max_change : -max_change);
    else current_output_voltage = target_voltage;

    // Gate Drive Sag: If input < 10V, Gate drive weakens -> R_ds_on rises
    Scalar delta_T = temperature_K - 298.15;
    Scalar gate_health = 1.0;
    if (esc_input_voltage < params.gate_drive_min_V) {
        gate_health = 1.0 + (params.gate_drive_min_V - esc_input_voltage) * params.gate_drive_sensitivity;
    }
    current_R_ohms = params.internal_R_Ohms * (1.0 + params.temp_coeff_r * delta_T) * gate_health;
    
    Scalar resistive_drop = motor_current * current_R_ohms;
    Scalar dead_time = (target_voltage > 0.1) ? params.dead_time_loss_V : 0.0;
    
    // Timing Loss due to Load (Armature Reaction)
    // High current shifts magnetic field, effectively retarding timing -> loss of voltage vector
    Scalar load_shift = std::abs(motor_current) * 0.05; // deg/A
    Scalar optimal_timing = std::abs(motor_rpm) * 0.0005; // simplified optimal curve
    Scalar timing_err = std::abs(optimal_timing - (params.timing_advance_deg - load_shift));
    Scalar timing_eff = std::cos(timing_err * (M_PI / 180.0));

    // Calculate Final Effective Voltage seen by coils
    Scalar effective_v = (current_output_voltage - dead_time) * timing_eff;
    Scalar final_voltage = std::max(0.0, effective_v - resistive_drop);
    if (final_voltage > 0) final_voltage += switching_noise(generator);

    // --- 10. THERMAL (Capacitor & MOSFET) ---
    // Duty Cycle approximation
    Scalar D = std::clamp(final_voltage / (esc_input_voltage + 0.001), 0.0, 1.0);
    
    // MOSFET Heat
    power_loss_W = calculatePowerLoss(motor_current, motor_rpm, D, esc_input_voltage);
    // Add extra heat from timing misalignment (power factor loss)
    power_loss_W += (current_output_voltage - effective_v) * std::abs(motor_current);

    // Capacitor Heat (Ripple)
    // I_ripple ~ I_motor * sqrt(D * (1-D))
    Scalar ripple_current = std::abs(motor_current) * std::sqrt(D * (1.0 - D));
    Scalar cap_heat = (ripple_current * ripple_current) * current_cap_esr;
    
    // Thermal Integration
    cap_temperature_K += ((cap_heat - (cap_temperature_K - 298.15) * params.cap_thermal_conduct_W_K) * dt) 
                       / (params.cap_mass_kg * 900.0); // 900 J/kgK approx for Al/Lyte

    temperature_K += ((power_loss_W - (temperature_K - 298.15) * params.thermal_conduct_W_K) * dt) 
                   / (params.mass_kg * params.specific_heat_JkgK);

    Scalar p_out = final_voltage * motor_current;
    Scalar p_in = p_out + power_loss_W;
    efficiency = (p_in > 0.001) ? (p_out / p_in) : 0.0;

    return final_voltage;
}

Scalar ESC::calculatePowerLoss(Scalar current, Scalar rpm, Scalar duty_cycle, Scalar input_v) {
    // Conduction Loss (Active Phase)
    Scalar on_loss = (current * current) * current_R_ohms * duty_cycle;
    
    // Freewheeling Loss (Off Phase)
    Scalar off_loss = 0.0;
    if (params.active_freewheeling) {
        // Sync Rectification: Current goes through FET (Low R)
        off_loss = (current * current) * current_R_ohms * (1.0 - duty_cycle);
    } else {
        // Diode Mode: Current goes through Body Diode (0.7V Drop)
        off_loss = std::abs(current) * params.diode_drop_V * (1.0 - duty_cycle);
    }
    
    // Switching Loss with Gate Sag Penalty
    // If gate voltage is low, switching is slower -> Time spent in linear region increases -> More heat
    Scalar switch_degrade = 1.0;
    if (input_v < params.gate_drive_min_V) {
        switch_degrade = 1.0 + (params.gate_drive_min_V - input_v) * params.switching_speed_sensitivity;
    }
    
    Scalar switching = 0.00005 * std::abs(rpm) * std::abs(current) * switch_degrade;
    
    return on_loss + off_loss + switching;
}