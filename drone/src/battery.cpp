#include "battery.h"
#include <algorithm>

Battery::Battery(const BatteryParams& p) 
    : params(p), current_charge_mAh(p.capacity_mAh) {}

Scalar Battery::getVoltage(Scalar current_load) const {
    // Total Internal Resistance = Cell R * Series Count
    Scalar total_R = params.internal_R * params.cell_count;
    
    // Ideal Voltage (simplified linear drop based on charge)
    // Real LiPos have a sigmoid curve, but linear is fine for now.
    Scalar charge_pct = current_charge_mAh / params.capacity_mAh;
    Scalar cell_v_now = 3.2 + (1.0 * charge_pct); // 3.2V empty, 4.2V full
    Scalar pack_v_ideal = cell_v_now * params.cell_count;

    // Voltage Sag Calculation
    Scalar voltage_sag = current_load * total_R;
    Scalar terminal_voltage = pack_v_ideal - voltage_sag;

    return std::max(0.0, terminal_voltage);
}

void Battery::discharge(Scalar current, Scalar dt) {
    // Convert Amps * Seconds -> milliAmp * Hours
    // mAh = (Amps * 1000) * (dt_sec / 3600)
    Scalar drained_mAh = (current * 1000.0) * (dt / 3600.0);
    current_charge_mAh -= drained_mAh;
    
    if(current_charge_mAh < 0) current_charge_mAh = 0;
}
