#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include "battery.h"

#ifndef TYPES_H
using Scalar = double;
#endif

Scalar get_current_load(Scalar time, const std::string& profile, Scalar peak_amps) {
    if (profile == "constant") return peak_amps;
    else if (profile == "pulse") return (fmod(time, 10.0) < 5.0) ? peak_amps : 0.0;
    else if (profile == "step") return (time < 60.0) ? (peak_amps * 0.2) : peak_amps;
    else if (profile == "abuse") {
        if (time < 150.0) return peak_amps; 
        if (time < 300.0) return 0.0;       
        return (fmod(time, 10.0) < 5.0) ? 10.0 : 0.0;
    }
    else if (profile == "regen") {
        if (time < 50.0) return peak_amps;
        if (time < 100.0) return -peak_amps * 0.5; 
        return peak_amps * 0.2; 
    }
    else if (profile == "mission") {
        if (time < 10.0) return 0.5;             
        if (time < 20.0) return peak_amps;       
        if (time < 120.0) return peak_amps * 0.4;
        if (time < 150.0) return peak_amps * 0.3;
        if (time < 250.0) return peak_amps * 0.4;
        if (time < 260.0) return peak_amps * 0.8;
        return 0.0;                              
    }
    else if (profile == "sine") {
        Scalar base = peak_amps * 0.5;
        Scalar amp = peak_amps * 0.2;
        Scalar freq = 0.5; 
        return base + amp * std::sin(time * freq * 6.28);
    }
    // NEW: Hysteresis Loop
    else if (profile == "hysteresis") {
        // 0-100s: Discharge (Push voltage down)
        if (time < 100.0) return 10.0;
        // 100-200s: Rest (Settle at LOW OCV)
        if (time < 200.0) return 0.0;
        // 200-250s: Charge (Push voltage up)
        if (time < 250.0) return -10.0;
        // 250-350s: Rest (Settle at HIGH OCV)
        if (time < 350.0) return 0.0;
        // 350+: Discharge again
        return 5.0;
    }
    return 0.0;
}

int main(int argc, char* argv[]) {
    BatteryConfig cfg;
    cfg.cell_count = 4;
    cfg.capacity_mAh = 1500;
    cfg.internal_R_mOhm = 5.0;
    cfg.mass_kg = 0.200;
    cfg.specific_heat_JkgK = 1200;
    cfg.thermal_conduct_W_K = 0.5;
    cfg.initial_charge_pct = 1.0;
    cfg.initial_temp_K = 298.15; 
    cfg.peukert_exponent = 1.05;
    cfg.coulombic_efficiency = 0.90;
    cfg.hysteresis_max_v = 0.05; // 50mV Hysteresis

    std::string profile = "constant";
    Scalar peak_amps = 20.0;
    Scalar ambient_temp_K = 298.15; 
    Scalar duration = 300.0;
    
    if (argc >= 2) profile = argv[1];
    if (argc >= 3) peak_amps = std::stod(argv[2]);
    if (argc >= 4) ambient_temp_K = std::stod(argv[3]);
    if (argc >= 5) cfg.initial_temp_K = std::stod(argv[4]);
    if (argc >= 6) duration = std::stod(argv[5]);

    auto batt = std::make_unique<LiPoBattery>(cfg);

    // Added 'hyst_v' column
    std::cout << "time,voltage,voltage_noisy,current,current_noisy,temperature_K,soc,resistance,health,power_W,heat_W,energy_J,hyst_v\n";

    Scalar dt = 0.1; 
    for (Scalar t = 0; t <= duration; t += dt) {
        Scalar load = get_current_load(t, profile, peak_amps);
        
        batt->update(load, dt, ambient_temp_K);

        std::cout << t << "," 
                  << batt->getVoltage() << ","
                  << batt->getVoltageNoisy() << ","
                  << load << ","
                  << batt->getCurrentNoisy() << ","
                  << batt->getTemperature() << ","
                  << batt->getSoC() << ","
                  << batt->getInternalResistance() << ","
                  << batt->getHealth() << ","
                  << batt->getPower() << ","
                  << batt->getHeatingPower() << ","
                  << batt->getEnergyConsumed() << ","
                  << batt->getHysteresisVoltage() << "\n"; // Log Hysteresis

        if (batt->getVoltage() <= 0.1) break;
    }
    return 0;
}