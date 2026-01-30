#pragma once

#include "types.h"

#include <random>
#include <vector>
#include <string>

struct IMUConfig {
    std::string name = "Generic";

    double accel_noise_density = 0.002; 
    double gyro_noise_density  = 0.0001; 
    double accel_bias_instability = 0.0005; 
    double gyro_bias_instability  = 0.00005; 
    
    double accel_init_bias_std = 0.2; 
    double gyro_init_bias_std  = 0.02; 
    Mat3 misalignment = Mat3::Identity(); 
    
    Vec3 accel_temp_coeff = Vec3(0.005, 0.005, 0.005); 
    Vec3 gyro_temp_coeff  = Vec3(0.0005, 0.0005, 0.0005); 
    double accel_vre_coeff = 0.002; 
    double gyro_g_sensitivity = 0.001; 
    
    double accel_deadband_g = 0.0; // Stiction threshold (e.g., 0.01g)
    
    double accel_saturation_g = 16.0; 
    double gyro_saturation_dps = 2000.0; 
    double sample_rate_hz = 1000.0; 
    int resolution_bits = 16;       
};

struct IMUData {
    Vec3 accel; 
    Vec3 gyro;  
    double temp_K;
};

class IMU {
    public:
        IMU(const IMUConfig& config, uint32_t seed = 42);
    
        void update(const Vec3& true_accel, const Vec3& true_gyro, double dt, double temp_K = 298.15);
        IMUData read() const;
    
        static IMUConfig ConfigMPU6000();  
        static IMUConfig ConfigBMI270();   
        static IMUConfig ConfigTactical();
        static IMUConfig ConfigBadMount();
    
    private:
        IMUConfig cfg;
        
        Vec3 current_accel_bias_walk; 
        Vec3 current_gyro_bias_walk;
        Vec3 init_accel_bias;         
        Vec3 init_gyro_bias;
        
        IMUData last_reading;
    
        std::mt19937 gen;
        std::normal_distribution<double> dist_std; 
    
        Vec3 applyImperfections(const Vec3& signal, const Vec3& total_bias, double saturation, double quantization_step);
};