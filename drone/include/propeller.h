#pragma once

#include "types.h"
#include <utility>

struct PropParams {
    Scalar diameter_inch;
    Scalar pitch_inch;
    // Aerodynamic Coefficients (depend on blade geometry)
    Scalar thrust_coeff; // Ct
    Scalar power_coeff;  // Cp
};

class Propeller {
public:
    Propeller(const PropParams& params);

    // Returns {Thrust (N), Torque_Drag (N*m)} based on RPM
    std::pair<Scalar, Scalar> getAerodynamics(Scalar rpm, Scalar air_density = 1.225) const;

private:
    const PropParams params;
    // Pre-calculated constants to avoid re-computing diameter math every step
    Scalar k_thrust;
    Scalar k_torque;
};
