#pragma once

#include "types.h"

#include <random>

struct MotorConfig {
    Scalar Kv;              
    Scalar Rm_nominal;      
    Scalar I0;              
    Scalar I_stall;         
    Scalar inductance_H;    
    Scalar pole_pairs;       
    Scalar cogging_torque_Nm;
    
    Scalar MoI;             
    Scalar viscous_damping; 
    Scalar mass_kg;             
    Scalar specific_heat_JkgK;  
    Scalar thermal_conduct_W_K; 
    Scalar cooling_rpm_factor;  
    Scalar initial_temp_K;      
};

class Motor {
public:
    virtual ~Motor() = default;
    virtual void update(Scalar dt, Scalar v_in, Scalar load_torque_Nm, Scalar ambient_temp_K) = 0;

    // Getters
    virtual Scalar getRPM() const = 0;
    virtual Scalar getCurrent() const = 0;
    virtual Scalar getTorque() const = 0;
    virtual Scalar getTemperature() const = 0;
    virtual Scalar getEfficiency() const = 0;
    virtual Scalar getPositionAngle() const = 0;
    virtual Scalar getRPM_Measured() const = 0;
    virtual Scalar getCurrent_Measured() const = 0;
};

class BLDCMotor : public Motor {
public:
    BLDCMotor(const MotorConfig& cfg);
    void update(Scalar dt, Scalar v_in, Scalar load_torque_Nm, Scalar ambient_temp_K) override;

    Scalar getRPM() const override;
    Scalar getCurrent() const override { return state_current; }
    Scalar getTorque() const override { return current_torque; }
    Scalar getTemperature() const override { return temperature_K; }
    Scalar getEfficiency() const override { return efficiency; }
    Scalar getPositionAngle() const override { return state_angle; }
    Scalar getRPM_Measured() const override;
    Scalar getCurrent_Measured() const override;

private:
    MotorConfig config;
    Scalar state_omega;    
    Scalar state_current;  
    Scalar state_angle;     
    Scalar temperature_K;  
    Scalar current_torque;
    Scalar efficiency;

    mutable std::default_random_engine generator;
    mutable std::normal_distribution<Scalar> noise_rpm;
    mutable std::normal_distribution<Scalar> noise_current;

    Scalar calculateDynamicResistance() const;
};