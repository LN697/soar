#include "drone.h"
#include <iostream>
#include <algorithm> 
#include <cmath>

Drone::Drone() 
    : battery({4, 3.7, 1500, 100, 0.002}),
      altitudePID({5.0, 1.0, 2.5, -10.0, 10.0, 50.0}), // PID Params: Kp, Ki, Kd, Min, Max, I_Max
      INERTIA_TENSOR((Mat3() << 0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.010).finished())
{
    // Motor: 2300Kv, 0.15 Ohm
    MotorParams mParams = {2300, 0.15, 0.5, 0.00005};
    // Prop: 5x4.5
    PropParams pParams = {5.0, 4.5, 0.12, 0.05};
    // ESC: 40A Limit
    EscParams eParams = {40.0}; 

    for(int i=0; i<4; ++i) {
        motors.emplace_back(mParams);
        propellers.emplace_back(pParams);
        escs.emplace_back(eParams);
    }
}

Scalar Drone::updateAltitude(Scalar dt, Scalar target_altitude) {
    // 1. Calculate Error
    Scalar error = target_altitude - s.pos.z();
    
    // 2. Run PID (Output is "Delta Voltage" needed on top of hover)
    Scalar pid_output = altitudePID.update(error, dt);
    
    // 3. Calculate Feed-Forward (Hover Voltage)
    // We need 9.81 N of thrust total -> 2.45 N per motor.
    // Inverting the Propeller/Motor math is hard analytically, 
    // so we stick to the empirical estimate for this specific drone config.
    // Based on previous sims, 2300Kv on 4S needs ~7.0V to hover 1kg.
    const Scalar FEED_FORWARD_VOLTAGE = 7.5; 
    
    // 4. Combine
    Scalar final_command = FEED_FORWARD_VOLTAGE + pid_output;
    
    // 5. Battery Safety Clamp (Anti-Windup assistance)
    // We check what the battery can actually give.
    // This isn't perfect anti-windup (PID internal state doesn't know),
    // but it prevents sending 100V requests.
    return std::max(0.0, final_command);
}

void Drone::step(Scalar dt, const std::array<Scalar, 4>& throttle_commands, const Vec3& wind_vector) {
    last_wind_vector = wind_vector;

    // 1. Battery Phase
    Scalar total_current = 0;
    for(const auto& m : motors) total_current += m.getCurrent();
    Scalar v_battery = battery.getVoltage(total_current);
    battery.discharge(total_current, dt);

    // 2. Control Loop
    Scalar total_thrust = 0;
    Vec3 generated_torque = Vec3::Zero();
    Scalar L = ARM_LENGTH / 1.41421356; 

    for(size_t i=0; i<4; ++i) {
        Scalar rpm = motors[i].getRPM();
        auto [thrust, drag_torque] = propellers[i].getAerodynamics(rpm);
        
        // Pass through ESC
        Scalar v_applied = escs[i].apply(throttle_commands[i], v_battery, motors[i]);
        
        // Update Motor Physics
        motors[i].update(dt, v_applied, drag_torque);

        total_thrust += thrust;
        
        // Torque Mixing (Standard Quad X)
        // FR(0), RL(1) are CCW -> Torque Up (+Z)
        // FL(2), RR(3) are CW  -> Torque Down (-Z)
        if (i==0) { 
             generated_torque.x() -= thrust * L; 
             generated_torque.y() += thrust * L; 
             generated_torque.z() += drag_torque;
        } else if (i==1) {
             generated_torque.x() += thrust * L;
             generated_torque.y() -= thrust * L;
             generated_torque.z() += drag_torque;
        } else if (i==2) {
             generated_torque.x() += thrust * L;
             generated_torque.y() += thrust * L;
             generated_torque.z() -= drag_torque;
        } else if (i==3) {
             generated_torque.x() -= thrust * L;
             generated_torque.y() -= thrust * L;
             generated_torque.z() -= drag_torque;
        }
    }

    // 3. Integration
    Vec3 force_body(0, 0, total_thrust);
    last_thrust_vector = s.ori * force_body;

    Vec3 gravity(0, 0, -9.81 * MASS);
    Vec3 force_world = last_thrust_vector + gravity;
    
    // Drag
    Vec3 v_air = s.vel - wind_vector;
    Vec3 force_drag = -0.5 * v_air; 
    force_world += force_drag;

    s.vel += (force_world / MASS) * dt;
    s.pos += s.vel * dt;
    
    Vec3 gyro = s.ang.cross(INERTIA_TENSOR * s.ang);
    Vec3 alpha = INERTIA_TENSOR.inverse() * (generated_torque - gyro);
    s.ang += alpha * dt;
    
    Quat q_dot; q_dot.w() = 0; q_dot.vec() = s.ang;
    Quat deriv = s.ori * q_dot;
    s.ori.w() += deriv.w() * 0.5 * dt;
    s.ori.vec() += deriv.vec() * 0.5 * dt;
    s.ori.normalize();
}

Telemetry Drone::getTelemetry(double t) const {
    Telemetry data;
    data.time = t;
    data.rpm_motor_0 = motors[0].getRPM();
    data.position = s.pos;
    data.velocity = s.vel;
    data.orientation = s.ori;
    data.force_thrust_world = last_thrust_vector;
    data.force_gravity = Vec3(0, 0, -9.81 * MASS);
    data.force_wind = last_wind_vector;
    data.current_total = 0;
    for(const auto& m : motors) data.current_total += m.getCurrent();
    data.voltage_battery = battery.getVoltage(data.current_total);
    return data;
}
