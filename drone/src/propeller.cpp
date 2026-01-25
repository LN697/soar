#include "propeller.h"
#include <cmath>

Propeller::Propeller(const PropParams& p) : params(p) {
    // Convert inches to meters
    Scalar D = params.diameter_inch * 0.0254;
    
    // Simplified Propeller Constants (Static Thrust)
    // Thrust = Ct * rho * n^2 * D^4
    // Torque = Cp * rho * n^2 * D^5
    // Note: 'n' is usually revs/sec in these formulas.
    // We will bake unit conversions into k_thrust/k_torque for raw RPM usage if desired,
    // or keep it standard. Let's stick to standard physics: input is Rad/s.
    
    // We'll define k factors for Rad/s input to simplify the step function
    // F = k * w^2
    
    // Empirical approximation for standard quad props:
    // This is a rough conversion from geometric Ct/Cp to lumped constants
    k_thrust = params.thrust_coeff * std::pow(D, 4);
    k_torque = params.power_coeff  * std::pow(D, 5); 
}

std::pair<Scalar, Scalar> Propeller::getAerodynamics(Scalar rpm, Scalar air_density) const {
    // Convert RPM to revs/sec (n)
    Scalar n = rpm / 60.0;
    
    // Safety check for negative RPM
    if (n < 0) n = -n;

    // Standard Propeller Theory
    Scalar thrust = params.thrust_coeff * air_density * (n*n) * std::pow(params.diameter_inch * 0.0254, 4);
    Scalar torque = params.power_coeff  * air_density * (n*n) * std::pow(params.diameter_inch * 0.0254, 5);

    return {thrust, torque};
}
