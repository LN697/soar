#pragma once

#include "types.h"

struct BatteryParams {
    Scalar cell_count;     // e.g., 4 (4S LiPo)
    Scalar cell_voltage;   // e.g., 3.7V nominal, 4.2V max
    Scalar capacity_mAh;   // e.g., 1500 mAh
    Scalar c_rating;       // e.g., 100C (Max discharge)
    Scalar internal_R;     // Internal Resistance per cell (Ohms)
};

class Battery {
public:
    Battery(const BatteryParams& params);

    // Calculate available voltage under a specific current load
    // V_terminal = V_pack - (I_load * R_pack)
    Scalar getVoltage(Scalar current_load) const;

    // Drain the battery capacity (Coulomb Counting)
    void discharge(Scalar current, Scalar dt);

    Scalar getChargeLevel() const { return current_charge_mAh; }

private:
    const BatteryParams params;
    Scalar current_charge_mAh;
};
