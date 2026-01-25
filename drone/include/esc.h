#pragma once

#include "types.h"
#include "motor.h" 

struct EscParams {
    Scalar max_current; // e.g. 40A
    // Future features: 
    // Scalar ramp_up_speed; 
    // bool bidirectional;
};

class ESC {
    public:
        ESC(const EscParams& params);

        // The Core Logic:
        // Takes a command (throttle 0-1 or voltage request) and the current state of the motor
        // Returns the Voltage that should actually be applied to the motor
        Scalar apply(Scalar throttle_voltage, Scalar battery_voltage, const Motor& motor);

    private:
        const EscParams params;
};
