#include "propeller.h"

#include <cmath>
#include <algorithm>

Propeller::Propeller(const PropParams& p) 
    : params(p), state_induced_velocity(0.0) {
    diameter_m = params.diameter_inch * 0.0254;
}

void Propeller::update(Scalar dt, Scalar rpm) {
    Scalar n = std::abs(rpm / 60.0);
    Scalar static_thrust = params.thrust_coeff_static * 1.225 * (n*n) * std::pow(diameter_m, 4);
    
    Scalar area = M_PI * std::pow(diameter_m/2.0, 2.0);
    Scalar target_v_i = 0.0;
    if (static_thrust > 0) {
        target_v_i = std::sqrt(static_thrust / (2.0 * 1.225 * area));
    }

    Scalar alpha = dt / (params.tau_flow_sec + dt);
    state_induced_velocity = (alpha * target_v_i) + ((1.0 - alpha) * state_induced_velocity);
}

std::tuple<Scalar, Scalar, Scalar> Propeller::getAerodynamics(
    Scalar rpm, Scalar axial_vel_mps, Scalar height_ground_m, Scalar air_density
) const {
    Scalar n = rpm / 60.0;
    if (n < 0.1) n = 0.1; 
    Scalar omega = rpm * 0.10472;

    Scalar total_inflow = std::abs(axial_vel_mps) + state_induced_velocity;
    Scalar J_effective = total_inflow / (n * diameter_m);

    Scalar Ct = std::max(0.0, params.thrust_coeff_static - (params.ct_decay_linear * J_effective));
    Scalar Cp = std::max(0.0, params.power_coeff_static - (params.cp_decay_linear * J_effective));

    Scalar tip_speed_mps = (omega * diameter_m / 2.0) + std::abs(axial_vel_mps);
    Scalar mach = tip_speed_mps / 343.0;
    if (mach > params.mach_limit_start) {
        Scalar excess = mach - params.mach_limit_start;
        Scalar deg = 1.0 - (excess * params.mach_limit_slope);
        if (deg < 0.2) deg = 0.2;
        Ct *= deg;
        Cp *= (1.0 + excess); 
    }

    if (axial_vel_mps < 0) {
        Scalar severity = std::abs(axial_vel_mps) / (state_induced_velocity + 0.1); 
        if (severity > 0.5 && severity < 1.5) {
            Ct *= 0.6; 
            Cp *= 1.2; 
        }
    }
    
    Scalar ground_factor = 1.0;
    if (height_ground_m < diameter_m) {
        Scalar h_effective = std::max(height_ground_m, 0.05 * diameter_m);
        
        Scalar ratio = 4.0 * h_effective / diameter_m;
        
        Scalar term = std::pow(1.0/ratio, 2.0);
        
        if (term > 0.95) term = 0.95; 
        
        Scalar ge_boost = 1.0 / (1.0 - term);
        ground_factor = std::min(ge_boost, params.ground_effect_factor); 
    }

    Scalar h_force = 0.0;
    if (std::abs(axial_vel_mps) > 1.0) {
        h_force = (Ct * air_density * (n*n) * std::pow(diameter_m, 4)) * J_effective * 0.5;
    }

    Scalar thrust = Ct * ground_factor * air_density * (n*n) * std::pow(diameter_m, 4);
    Scalar torque = Cp * air_density * (n*n) * std::pow(diameter_m, 5);

    return {thrust, torque, h_force};
}