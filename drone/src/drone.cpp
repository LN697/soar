#include "drone.h"
#include <iostream>

Drone::Drone() 
    : INERTIA_TENSOR((Mat3() << 0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.010).finished())
{
    // Example: A typical 2300Kv racing motor
    MotorParams params;
    params.Kv = 2300; 
    params.Rm = 0.15;      // 150 milli-ohms
    params.I0 = 0.5;       // 0.5A no-load
    params.MoI = 0.00005;  // Small rotor inertia
    params.dragCoeff = 1.0e-7; // Propeller drag factor
    
    for(int i = 0; i < 4; ++i) {
        motors.emplace_back(params);
    }
}

void Drone::step(Scalar dt, const std::array<Scalar, 4>& voltages, const Vec3& wind_vector) {
    // 1. Motor Physics & Thrust Generation
    // We assume Quad-X config:
    // 0: Front-Right (CCW), 1: Rear-Left (CCW) -> Counter-Torque is Positive (Up)
    // 2: Front-Left (CW), 3: Rear-Right (CW)   -> Counter-Torque is Negative (Down)
    // Note: Standard betaflight mixing is often different, check your specific frame.
    
    Scalar total_thrust = 0.0;
    Vec3 generated_torque = Vec3::Zero();
    
    // Distance from center of mass to motor (diagonal)
    // For torque calc: L_x = L_y = ARM_LENGTH / sqrt(2)
    Scalar L_arm = ARM_LENGTH / 1.41421356; 

    for(size_t i = 0; i < 4; ++i) {
        auto [thrust, drag_torque] = motors[i].step(dt, voltages[i]);
        total_thrust += thrust;

        // Calculate Moments (Torques) in Body Frame
        // 0: FR (+x, -y) CW  -> PITCH DOWN, ROLL LEFT, YAW LEFT (Torque +)
        // 1: RL (-x, +y) CW  -> PITCH UP, ROLL RIGHT, YAW LEFT (Torque +)
        // 2: FL (+x, +y) CCW -> PITCH DOWN, ROLL RIGHT, YAW RIGHT (Torque -)
        // 3: RR (-x, -y) CCW -> PITCH UP, ROLL LEFT, YAW RIGHT (Torque -)
        
        // This mapping depends entirely on your motor indices/directions.
        // Let's use a standard X-Config map:
        // M0 (FR, CCW), M1 (RL, CCW), M2 (FL, CW), M3 (RR, CW)
        
        Scalar t_thrust = thrust;
        Scalar t_drag = drag_torque; // Direction depends on CW/CCW
        
        // Simple Quad-X Mixer Logic (Thrust -> Torque)
        if (i == 0) { // FR (CCW)
             generated_torque.x() -= t_thrust * L_arm; // Roll
             generated_torque.y() += t_thrust * L_arm; // Pitch
             generated_torque.z() += t_drag;           // Yaw (Reaction is CW)
        } else if (i == 1) { // RL (CCW)
             generated_torque.x() += t_thrust * L_arm;
             generated_torque.y() -= t_thrust * L_arm;
             generated_torque.z() += t_drag;
        } else if (i == 2) { // FL (CW)
             generated_torque.x() += t_thrust * L_arm;
             generated_torque.y() += t_thrust * L_arm;
             generated_torque.z() -= t_drag;
        } else if (i == 3) { // RR (CW)
             generated_torque.x() -= t_thrust * L_arm;
             generated_torque.y() -= t_thrust * L_arm;
             generated_torque.z() -= t_drag;
        }
    }

    // Force in Body Frame (Thrust is always up in Z-axis for the drone)
    Vec3 force_body(0, 0, total_thrust); 
    
    // --- Translational Dynamics (World Frame) ---
    
    // 2. Gravity (World Frame)
    Vec3 gravity(0, 0, -9.81 * MASS);
    
    // 3. Transform Thrust to World Frame
    // F_world = q * F_body * q_inv
    Vec3 force_thrust_world = s.ori * force_body;
    
    // 4. Aerodynamic Drag (Simple linear model)
    // Velocity relative to air
    Vec3 v_air = s.vel - wind_vector; 
    Vec3 force_aero = -0.1 * v_air; // Drag coeff 0.1
    
    // Total Force
    Vec3 force_total = gravity + force_thrust_world + force_aero;
    
    // Acceleration
    Vec3 acceleration = force_total / MASS;
    
    // Integration (Symplectic Euler for stability)
    s.vel += acceleration * dt;
    s.pos += s.vel * dt;
    
    // --- Rotational Dynamics (Body Frame) ---
    
    // Euler's Equation: I * alpha + w x (I * w) = Tau
    // alpha = I_inv * (Tau - w x (I * w))
    
    Vec3 gyro_moment = s.ang.cross(INERTIA_TENSOR * s.ang);
    Vec3 alpha = INERTIA_TENSOR.inverse() * (generated_torque - gyro_moment);
    
    s.ang += alpha * dt;
    
    // Update Orientation (Quaternion Integration)
    // q_new = q_old + 0.5 * q_old * (0, wx, wy, wz) * dt
    Quat q_dot;
    q_dot.w() = 0;
    q_dot.vec() = s.ang;
    
    // Standard quaternion derivative formula: q_dot = 1/2 * q * w
    Quat deriv = s.ori * q_dot;
    
    s.ori.w() += deriv.w() * 0.5 * dt;
    s.ori.vec() += deriv.vec() * 0.5 * dt;
    s.ori.normalize(); // Mandatory normalization to prevent drift
}

Telemetry Drone::getTelemetry(double t, double v_in) const {
    Telemetry data;
    data.time = t;
    data.voltage_in = v_in;
    data.pos_z = s.pos.z();
    data.vel_z = s.vel.z();
    
    // Aggregate Motor Data
    data.current_total = 0;
    data.thrust_total = 0;
    
    // Assuming Motor 0 is representative for RPM
    data.rpm_motor_0 = motors[0].getRPM(); 
    
    for(const auto& m : motors) {
        data.current_total += m.getCurrent();
        // We can't easily get 'thrust' back from motor unless we store it, 
        // but for now, let's just log current/rpm which is sufficient.
    }
    
    // Simulated Accelerometer (Gravity + Body Acceleration)
    // Ideally, this comes from forces, but for simple logging:
    // (We will skip accel_z logging here to save complexity, 
    //  deriving it in Python is often cleaner).
    data.accel_z_body = 0; 
    
    return data;
}
