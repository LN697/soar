#include <iostream>
#include <vector>
#include <iomanip>
#include "drone.h"

int main() {
    Drone drone;
    
    const double dt = 0.001;
    const double sim_duration = 1.0;
    double current_time = 0.0; 
    
    std::cout << "Time,Voltage,Current,RPM,PosZ,VelZ" << std::endl;
    std::cout << std::fixed << std::setprecision(5);

    while (current_time < sim_duration) {
        // --- Test Profile: Step Response ---
        // 0.0s to 0.1s: Silence
        // 0.1s to 0.6s: 100% Throttle (Step Up)
        // 0.6s to 1.0s: 0% Throttle (Step Down)
        
        double voltage = 0.0;
        if (current_time >= 0.1 && current_time < 0.6) {
            voltage = 12.0; // 3S LiPo Step Input
        }
        
        std::array<double, 4> inputs;
        inputs.fill(voltage);
        Eigen::Vector3d wind(0,0,0);

        // Physics Step
        drone.step(dt, inputs, wind);
        
        // Log Data (100Hz Logging for cleaner CSVs)
        static int log_div = 0;
        if (log_div++ % 10 == 0) {
            Telemetry t = drone.getTelemetry(current_time, voltage);
            
            std::cout << t.time << "," 
                      << t.voltage_in << ","
                      << t.current_total << ","
                      << t.rpm_motor_0 << ","
                      << t.pos_z << ","
                      << t.vel_z 
                      << std::endl;
        }

        current_time += dt;
    }

    return 0;
}