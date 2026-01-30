/*
 * test_runner.cpp - Advanced IMU Physics Harness
 * Compile: g++ -std=c++17 -I. -I/usr/include/eigen3 test_runner.cpp imu.cpp -o imu_test
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include "imu.h"
#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argc, char* argv[]) {
    // Usage: ./imu_test [profile] [duration] [sensor_type]
    std::string profile = "static";
    double duration = 1.0;
    std::string sensor_type = "mpu6000";

    if (argc >= 2) profile = argv[1];
    if (argc >= 3) duration = std::stod(argv[2]);
    if (argc >= 4) sensor_type = argv[3];

    IMUConfig cfg;
    if (sensor_type == "bmi270") cfg = IMU::ConfigBMI270();
    else if (sensor_type == "tactical") cfg = IMU::ConfigTactical();
    else if (sensor_type == "bad_mount") cfg = IMU::ConfigBadMount();
    else cfg = IMU::ConfigMPU6000();

    IMU imu(cfg);
    double dt = 1.0 / cfg.sample_rate_hz;
    double t = 0.0;

    // Header including cross-axis fields
    std::cout << "time,true_ax,true_ay,true_az,meas_ax,meas_ay,meas_az,true_gx,meas_gx\n";

    while (t < duration) {
        Vec3 true_accel(0, 0, 9.81);
        Vec3 true_gyro(0, 0, 0);
        double temp_k = 298.15;

        // --- Profile Logic ---
        
        if (profile == "coupling") {
            // Apply pure X-axis acceleration surge
            // Ideally, Y and Z shouldn't change.
            // With bad mounting, Y/Z will react.
            true_accel.x() = 5.0 * std::sin(t * 2.0); // +/- 5 m/s^2 oscillation
        }

        if (profile == "g_sensitivity") {
            // High-G Turn simulation
            // 5G linear acceleration, but ZERO rotation.
            // Ideally Gyro = 0. With G-sensitivity, Gyro != 0.
            true_accel.y() = 50.0; // ~5G
        }

        // --- Update & Log ---
        imu.update(true_accel, true_gyro, dt, temp_k);
        IMUData meas = imu.read();

        std::cout << t << "," 
                  << true_accel.x() << "," << true_accel.y() << "," << true_accel.z() << ","
                  << meas.accel.x() << "," << meas.accel.y() << "," << meas.accel.z() << ","
                  << true_gyro.x() << "," << meas.gyro.x() << "\n";

        t += dt;
    }
    return 0;
}