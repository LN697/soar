#pragma once

#include "types.h"

#include <tuple>

struct PropParams {
    Scalar diameter_inch;
    Scalar pitch_inch;
    
    Scalar thrust_coeff_static;
    Scalar power_coeff_static;
    
    Scalar ct_decay_linear;
    Scalar cp_decay_linear;
    Scalar ground_effect_factor;

    Scalar mach_limit_start; 
    Scalar mach_limit_slope;
    Scalar tau_flow_sec; 
};

class Propeller {
public:
    Propeller(const PropParams& params);

    void update(Scalar dt, Scalar rpm, Scalar axial_vel_mps);

    // Getters
    std::tuple<Scalar, Scalar, Scalar> getAerodynamics(Scalar rpm, Scalar axial_vel_mps, Scalar height_ground_m, Scalar air_density = 1.225) const;

private:
    const PropParams params;
    Scalar diameter_m;
    
    Scalar state_induced_velocity; 
};