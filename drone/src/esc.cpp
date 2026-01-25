#include "esc.h"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ESC::ESC(const EscParams& p) : params(p) {}

Scalar ESC::apply(Scalar throttle_voltage, Scalar battery_voltage, const Motor& motor) {
    // 1. Clamp request to physical battery availability
    Scalar v_request = std::min(throttle_voltage, battery_voltage);
    
    // 2. Predictive Current Limiting (Feed-Forward)
    // We need to peek into the motor's state (Back-EMF) to know the limit
    // Note: Ideally, Motor would expose "getBackEMF()" or "getResistance()"
    // For now, we calculate it using the Motor's public getters if possible,
    // or we assume we know the motor parameters (Simulating the ESC knowing its setting).
    
    // A proper architecture would have the ESC know the Motor's Resistance/Kv 
    // via a config or query. Let's calculate it here assuming standard 2300Kv/0.15R for now.
    // In a final polish, pass MotorParams to ESC constructor or query motor.
    
    // Hardcoded for this specific Motor (Refactor target: pass MotorParams)
    const Scalar Rm = 0.15;
    const Scalar Kv = 2300;
    
    Scalar rpm = motor.getRPM();
    Scalar Kv_SI = Kv * (2.0 * M_PI / 60.0);
    Scalar Kt = 1.0 / Kv_SI;
    Scalar back_emf = (rpm * (2.0 * M_PI / 60.0)) * Kt;

    // V_max = (I_limit * R) + BackEMF
    Scalar v_limit = (params.max_current * Rm) + back_emf;

    // 3. Final Decision
    return std::min(v_request, v_limit);
}