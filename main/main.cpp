#include <iostream>
#include <vector>
#include <iomanip>
#include "drone.h"

int main() {
    Drone drone;
    
    const double dt = 0.001; 
    const double sim_duration = 10.0; 
    double current_time = 0.0; 
    
    // CSV Header (Matches plot.py)
    std::cout << "Time,PosX,PosY,PosZ,Qw,Qx,Qy,Qz,"
              << "Fx_Thrust,Fy_Thrust,Fz_Thrust,"
              << "Fx_Wind,Fy_Wind,Fz_Wind" << std::endl;
    std::cout << std::fixed << std::setprecision(5);

    double target_altitude = 5.0; 

    while (current_time < sim_duration) {
        // 1. Flight Controller Step
        // The drone calculates its own throttle to reach target_altitude
        Scalar throttle_voltage = drone.updateAltitude(dt, target_altitude);

        // 2. Wind Gust Generation
        Vec3 wind(0,0,0);
        // if (current_time > 2.0 && current_time < 4.0) {
        //     wind = Vec3(2.0, 0.5, 0); 
        // }

        // 3. Physics Step
        std::array<double, 4> inputs;
        inputs.fill(throttle_voltage);
        
        drone.step(dt, inputs, wind);
        
        // 4. Log Data
        static int log_div = 0;
        if (log_div++ % 50 == 0) { // 20Hz logging
            Telemetry t = drone.getTelemetry(current_time);
            
            std::cout << t.time << "," 
                      << t.position.x() << "," << t.position.y() << "," << t.position.z() << ","
                      << t.orientation.w() << "," << t.orientation.x() << "," 
                      << t.orientation.y() << "," << t.orientation.z() << ","
                      << t.force_thrust_world.x() << "," << t.force_thrust_world.y() << "," << t.force_thrust_world.z() << ","
                      << t.force_wind.x() << "," << t.force_wind.y() << "," << t.force_wind.z()
                      << std::endl;
        }

        current_time += dt;
    }

    return 0;
}