#pragma once

#include "types.h"

#include <memory>
#include <vector>
#include <map>
#include <random>

struct BatteryConfig {
    Scalar cell_count;
    Scalar capacity_mAh;
    Scalar internal_R_mOhm;
    Scalar mass_kg;
    Scalar specific_heat_JkgK;
    Scalar thermal_conduct_W_K;
    Scalar initial_temp_K;
    Scalar initial_charge_pct;
    
    Scalar peukert_exponent;
    Scalar coulombic_efficiency;
    Scalar hysteresis_max_v; 
};

class Battery {
public:
    virtual ~Battery() = default;
    virtual void update(Scalar input_current, Scalar dt, Scalar ambient_temp_K) = 0;

    // Getters
    virtual Scalar getVoltage() const = 0;              
    virtual Scalar getTemperature() const = 0;          
    virtual Scalar getSoC() const = 0;                  
    virtual Scalar getChargeLevelmAh() const = 0;       
    virtual Scalar getInternalResistance() const = 0;   
    virtual Scalar getHealth() const = 0; 
    virtual Scalar getPower() const = 0;         
    virtual Scalar getHeatingPower() const = 0;  
    virtual Scalar getEnergyConsumed() const = 0;
    virtual Scalar getVoltageNoisy() const = 0;
    virtual Scalar getCurrentNoisy() const = 0;
    virtual Scalar getHysteresisVoltage() const = 0;
};

class LiPoBattery : public Battery {
public:
    LiPoBattery(const BatteryConfig& cfg);

    void update(Scalar input_current, Scalar dt, Scalar ambient_temp_K) override;
    
    Scalar getVoltage() const override { return current_voltage; }
    Scalar getTemperature() const override { return temperature_K; }
    Scalar getChargeLevelmAh() const override { return current_charge_mAh; }
    Scalar getSoC() const override { return current_charge_mAh / config.capacity_mAh; }
    Scalar getInternalResistance() const override { return calculateTotalResistance(); }
    Scalar getHealth() const override { return health_damage_factor; }
    Scalar getPower() const override { return current_voltage * last_current_draw; }
    Scalar getHeatingPower() const override { return last_heat_power_W; }
    Scalar getEnergyConsumed() const override { return total_energy_consumed_J; }
    Scalar getVoltageNoisy() const override;
    Scalar getCurrentNoisy() const override;
    Scalar getHysteresisVoltage() const override { return hysteresis_state_v; }

private:
    BatteryConfig config;
    Scalar current_charge_mAh;
    Scalar temperature_K;
    Scalar current_voltage;
    Scalar filtered_voltage; 
    Scalar last_current_draw;
    Scalar health_damage_factor; 
    Scalar last_heat_power_W;
    Scalar total_energy_consumed_J;
    Scalar hysteresis_state_v; 

    std::map<Scalar, Scalar> ocv_curve;
    
    mutable std::default_random_engine generator;
    mutable std::normal_distribution<Scalar> voltage_noise;
    mutable std::normal_distribution<Scalar> current_noise;

    Scalar interpolateOCV(Scalar soc) const;
    Scalar calculateTotalResistance() const;
};