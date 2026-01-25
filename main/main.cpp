#include <iostream>
#include <vector>
#include <iomanip>
#include "drone.h"

int main() {
    Drone drone;
    
    const double dt = 0.002; // 500Hz physics
    const double sim_duration = 15.0; 
    double current_time = 0.0; 
    
    std::cout << "Time,TX,TY,TZ,PX,PY,PZ,QX,QY,QZ,QW,EX,EY,EZ,EQX,EQY,EQZ,EQW,RPM0,RPM1,RPM2,RPM3,DragX,DragY,DragZ,GyroMX,GyroMY,GyroMZ" << std::endl;
    std::cout << std::fixed << std::setprecision(5);

    ControlInputs inputs;
    inputs.target_pos = Vec3(0, 0, 5); // Takeoff to 5m
    inputs.target_yaw = 0;

    while (current_time < sim_duration) {
        
        // Waypoint Logic
        if (current_time > 5.0 && current_time < 10.0) {
            inputs.target_pos = Vec3(2.0, 2.0, 5.0); // Move Diagonally
        } else if (current_time > 10.0) {
            inputs.target_pos = Vec3(2.0, 2.0, 1.0); // Land
        }

        drone.update(dt, inputs);
        
        static int log_div = 0;
        if (log_div++ % 25 == 0) { // 20Hz logging
            Telemetry t = drone.getTelemetry(current_time);
            
            std::cout << t.time << "," 
                      << t.target_pos.x() << "," << t.target_pos.y() << "," << t.target_pos.z() << ","
                      << t.pos_true.x() << "," << t.pos_true.y() << "," << t.pos_true.z() << ","
                      << t.ori_true.x() << "," << t.ori_true.y() << "," << t.ori_true.z() << "," << t.ori_true.w() << ","
                      << t.pos_est.x() << "," << t.pos_est.y() << "," << t.pos_est.z() << ","
                      << t.ori_est.x() << "," << t.ori_est.y() << "," << t.ori_est.z() << "," << t.ori_est.w() << ","
                      << t.motor_rpm[0] << "," << t.motor_rpm[1] << "," << t.motor_rpm[2] << "," << t.motor_rpm[3] << ","
                      << t.force_drag.x() << "," << t.force_drag.y() << "," << t.force_drag.z() << ","
                      << t.moment_gyro.x() << "," << t.moment_gyro.y() << "," << t.moment_gyro.z()
                      << std::endl;
        }

        current_time += dt;
    }

    return 0;
}