#include "battery.h"

#include <algorithm>
#include <cmath>
#include <iostream>

LiPoBattery::LiPoBattery(const BatteryConfig& cfg)
    : config(cfg),
      current_charge_mAh(cfg.capacity_mAh * cfg.initial_charge_pct),
      temperature_K(cfg.initial_temp_K),
      current_voltage(0.0),
      last_current_draw(0.0),
      health_damage_factor(0.0),
      last_heat_power_W(0.0),
      total_energy_consumed_J(0.0),
      hysteresis_state_v(0.0) {
    if (config.peukert_exponent < 1.0) config.peukert_exponent = 1.05;
    if (config.coulombic_efficiency <= 0.0) config.coulombic_efficiency = 0.90;
    if (config.hysteresis_max_v <= 0.0) config.hysteresis_max_v = 0.05; // 50mV gap is standard

    ocv_curve = {
        {1.00, 4.20}, {0.95, 4.15}, {0.90, 4.11},
        {0.80, 4.02}, {0.70, 3.92}, {0.60, 3.85},
        {0.50, 3.80}, {0.40, 3.76}, {0.30, 3.72},
        {0.20, 3.68}, {0.10, 3.55}, {0.05, 3.30}, 
        {0.00, 3.00}
    };
    
    filtered_voltage = config.cell_count * 4.2;
    voltage_noise = std::normal_distribution<Scalar>(0.0, 0.05); 
    current_noise = std::normal_distribution<Scalar>(0.0, 0.10); 
}

void LiPoBattery::update(Scalar input_current, Scalar dt, Scalar ambient_temp_K) {
    last_current_draw = input_current;
    
    Scalar target_hyst = 0.0;
    if (input_current > 0.1) target_hyst = -config.hysteresis_max_v;
    else if (input_current < -0.1) target_hyst = config.hysteresis_max_v;
    else {
        target_hyst = 0.0; 
    }

    // Rate of change for hysteresis depends on the throughput of current relative to capacity. A simple low-pass filter approach for the "drag".
    Scalar hyst_rate = 0.1 * dt;
    
    if (input_current == 0) hyst_rate = 0.001 * dt;
    
    Scalar diff = target_hyst - hysteresis_state_v;
    if (std::abs(diff) > hyst_rate) {
        hysteresis_state_v += (diff > 0 ? hyst_rate : -hyst_rate);
    } else {
        hysteresis_state_v = target_hyst;
    }
    
    Scalar effective_current = input_current;
    Scalar efficiency_loss_power = 0.0;

    if (input_current < 0) {
        Scalar charging_eff = config.coulombic_efficiency;
        if (temperature_K < 288.15) charging_eff *= 0.8;
        effective_current = input_current * charging_eff; 
        Scalar lost_current = input_current * (1.0 - charging_eff); 
        efficiency_loss_power = std::abs(lost_current * 1.0); 
    } else {
        Scalar rated_current = config.capacity_mAh / 1000.0;
        if (input_current > rated_current) {
            Scalar discharge_ratio = input_current / rated_current;
            effective_current = input_current * std::pow(discharge_ratio, config.peukert_exponent - 1.0);
        }
    }
    
    Scalar drained_mAh = (effective_current * 1000.0) * (dt / 3600.0);
    current_charge_mAh -= drained_mAh;
    if (current_charge_mAh < 0) current_charge_mAh = 0;
    if (current_charge_mAh > config.capacity_mAh) current_charge_mAh = config.capacity_mAh;
    
    Scalar R_total = calculateTotalResistance();
    last_heat_power_W = ((input_current * input_current) * R_total) + efficiency_loss_power;
    Scalar power_cool_W = (temperature_K - ambient_temp_K) * config.thermal_conduct_W_K;
    Scalar net_power = last_heat_power_W - power_cool_W;
    Scalar delta_temp = (net_power * dt) / (config.mass_kg * config.specific_heat_JkgK);
    temperature_K += delta_temp;

    if (temperature_K > 333.15) {
        Scalar excess = temperature_K - 333.15;
        health_damage_factor += (0.0001 * excess * dt);
    }
    
    R_total = calculateTotalResistance(); 
    Scalar soc = getSoC();
    
    Scalar cell_base_ocv = interpolateOCV(soc);
    Scalar cell_final_ocv = cell_base_ocv + hysteresis_state_v;
    
    Scalar pack_ocv = cell_final_ocv * config.cell_count;
    Scalar sag = input_current * R_total;
    Scalar raw_voltage = std::max(0.0, pack_ocv - sag);

    // Relaxation Filter
    const Scalar tau = 5.0; 
    Scalar alpha = dt / (tau + dt);
    filtered_voltage = (alpha * raw_voltage) + ((1.0 - alpha) * filtered_voltage);
    current_voltage = filtered_voltage;
    
    if (input_current > 0) {
        total_energy_consumed_J += (current_voltage * input_current) * dt;
    }
}

Scalar LiPoBattery::getVoltageNoisy() const {
    return current_voltage + voltage_noise(generator);
}
Scalar LiPoBattery::getCurrentNoisy() const {
    return last_current_draw + current_noise(generator);
}
Scalar LiPoBattery::interpolateOCV(Scalar soc) const {
    auto it = ocv_curve.lower_bound(soc);
    if (it != ocv_curve.end() && it->first == soc) return it->second;
    if (it == ocv_curve.begin()) return it->second;
    if (it == ocv_curve.end()) return ocv_curve.rbegin()->second;
    auto prev = std::prev(it);

    return prev->second + (soc - prev->first) * (it->second - prev->second) / (it->first - prev->first);
}
Scalar LiPoBattery::calculateTotalResistance() const {
    Scalar base_R = (config.internal_R_mOhm / 1000.0); 
    const Scalar REF_TEMP_K = 298.15; 
    Scalar temp_factor = 1.0;
    if (temperature_K < REF_TEMP_K) {
        temp_factor = 1.0 + std::pow((REF_TEMP_K - temperature_K) / 15.0, 2.0); 
    } 
    Scalar soc = getSoC();
    Scalar soc_factor = 1.0;
    if (soc < 0.20) {
        Scalar empty_ratio = (0.20 - soc) / 0.20; 
        soc_factor = 1.0 + (std::pow(empty_ratio, 3.0) * 10.0);
    }
    Scalar health_multiplier = 1.0 + health_damage_factor;
    
    return base_R * temp_factor * soc_factor * health_multiplier * config.cell_count;
}