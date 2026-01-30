#include "imu.h"

#include <cmath>
#include <algorithm>
#include <iostream>

const double G = 9.81;
const double DEG_TO_RAD = M_PI / 180.0;

IMU::IMU(const IMUConfig& config, uint32_t seed) : cfg(config), gen(seed), dist_std(0.0, 1.0) {
    init_accel_bias = Vec3(dist_std(gen), dist_std(gen), dist_std(gen)) * cfg.accel_init_bias_std;
    init_gyro_bias  = Vec3(dist_std(gen), dist_std(gen), dist_std(gen)) * cfg.gyro_init_bias_std;
    current_accel_bias_walk = Vec3::Zero();
    current_gyro_bias_walk = Vec3::Zero();
    last_reading = {Vec3::Zero(), Vec3::Zero(), 298.15};
}

void IMU::update(const Vec3& true_accel, const Vec3& true_gyro, double dt, double temp_K) {
    double sqrt_dt = std::sqrt(dt);
    current_accel_bias_walk += Vec3(dist_std(gen), dist_std(gen), dist_std(gen)) * cfg.accel_bias_instability * sqrt_dt;
    current_gyro_bias_walk  += Vec3(dist_std(gen), dist_std(gen), dist_std(gen)) * cfg.gyro_bias_instability * sqrt_dt;

    double delta_T = temp_K - 298.15;
    Vec3 total_accel_bias = init_accel_bias + current_accel_bias_walk + (cfg.accel_temp_coeff * delta_T);
    Vec3 total_gyro_bias  = init_gyro_bias  + current_gyro_bias_walk  + (cfg.gyro_temp_coeff  * delta_T);

    double vib_energy = std::max(0.0, true_accel.norm() - G); 
    total_accel_bias.z() += cfg.accel_vre_coeff * (vib_energy * vib_energy); 

    Vec3 g_coupling = true_accel * cfg.gyro_g_sensitivity;

    double bw_factor = std::sqrt(cfg.sample_rate_hz); 
    Vec3 accel_noise = Vec3(dist_std(gen), dist_std(gen), dist_std(gen)) * cfg.accel_noise_density * bw_factor;
    Vec3 gyro_noise  = Vec3(dist_std(gen), dist_std(gen), dist_std(gen)) * cfg.gyro_noise_density * bw_factor;

    Vec3 final_accel = true_accel + accel_noise;
    Vec3 final_gyro  = true_gyro + g_coupling + gyro_noise;

    double accel_max = cfg.accel_saturation_g * G;
    double accel_step = (2.0 * accel_max) / std::pow(2, cfg.resolution_bits);
    double gyro_max = cfg.gyro_saturation_dps * DEG_TO_RAD;
    double gyro_step = (2.0 * gyro_max) / std::pow(2, cfg.resolution_bits);

    last_reading.accel = applyImperfections(final_accel, total_accel_bias, accel_max, accel_step);
    last_reading.gyro  = applyImperfections(final_gyro, total_gyro_bias, gyro_max, gyro_step);
    last_reading.temp_K = temp_K;
}

Vec3 IMU::applyImperfections(const Vec3& signal, const Vec3& bias, double saturation, double quantization_step) {
    Vec3 misaligned = cfg.misalignment * signal;
    Vec3 biased = misaligned + bias;

    for (int i = 0; i < 3; i++) {
        double val_no_bias = biased[i] - bias[i];
        if (std::abs(val_no_bias) < (cfg.accel_deadband_g * G)) {
            biased[i] = bias[i];
        }

        biased[i] = std::clamp(biased[i], -saturation, saturation);
        
        biased[i] = std::round(biased[i] / quantization_step) * quantization_step;
    }
    return biased;
}

IMUData IMU::read() const {
    return last_reading;
}

IMUConfig IMU::ConfigMPU6000() {
    IMUConfig c; c.name = "MPU6000";
    c.accel_vre_coeff = 0.005; 
    c.accel_temp_coeff = Vec3::Constant(0.01); 
    return c;
}

IMUConfig IMU::ConfigBMI270() {
    IMUConfig c; c.name = "BMI270";
    c.accel_noise_density = 0.0015; 
    c.accel_vre_coeff = 0.001; 
    c.accel_temp_coeff = Vec3::Constant(0.002);
    return c;
}

IMUConfig IMU::ConfigTactical() {
    IMUConfig c; c.name = "Tactical";
    c.accel_noise_density = 1e-5; c.gyro_noise_density = 1e-6;
    c.accel_bias_instability = 1e-6; c.resolution_bits = 24;
    c.accel_temp_coeff = Vec3::Constant(1e-5); 
    return c;
}

IMUConfig IMU::ConfigBadMount() {
    IMUConfig c = ConfigMPU6000();
    c.name = "Badly Mounted Sensor";
    c.misalignment << 0.98, 0.05, 0.02,
                     -0.05, 0.98, 0.01,
                      0.02, -0.01, 1.05;
    
    c.accel_deadband_g = 0.05; // Significant stiction
    return c;
}