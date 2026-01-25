#include "drone.h"
#include <iostream>
#include <algorithm> 
#include <cmath>

Drone::Drone() 
    : battery({4, 3.7, 1500, 100, 0.002}),
      imu({0.5, 0.01, 0.005}), 
      estimator(),
      controller(),
      INERTIA_TENSOR((Mat3() << 0.015, 0, 0, 0, 0.015, 0, 0, 0, 0.025).finished()) 
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
    
    estimator.initialize(ori, pos);
}

void Drone::update(Scalar dt, const ControlInputs& inputs) {
    last_inputs = inputs;

    // --- 1. SENSORS & ESTIMATION ---
    Vec3 gravity_world(0, 0, 9.81); // Gravity acts DOWN (-9.81). Reaction is UP (+9.81)
    Vec3 proper_accel_world = (vel - Vec3::Zero()) / dt + gravity_world; 
    Vec3 proper_accel_body = ori.inverse() * proper_accel_world;

    imu.update(proper_accel_body, ang_vel, dt);
    IMUData sensor_data = imu.read();

    estimator.update(dt, sensor_data);
    estimator.updatePositionExternal(pos, vel);

    // --- 2. CONTROLLER ---
    MotorMix cmds = controller.update(dt, inputs, 
                                      estimator.getPosition(), estimator.getVelocity(), 
                                      estimator.getAttitude(), estimator.getAngularVelocity());

    // --- 3. PHYSICS ---
    Scalar total_current = 0;
    for(const auto& m : motors) total_current += m.getCurrent();
    Scalar v_battery = battery.getVoltage(total_current);
    battery.discharge(total_current, dt);

    Scalar total_thrust = 0;
    Vec3 torque_aero = Vec3::Zero();
    Vec3 H_props = Vec3::Zero(); 

    Scalar L = ARM_LENGTH / 1.41421356; 
    
    // Physics Frame: ENU (X Front, Y Left, Z Up)
    // FR(0): (+X, -Y) | CCW
    // RL(1): (-X, +Y) | CCW
    // FL(2): (+X, +Y) | CW
    // RR(3): (-X, -Y) | CW

    for(size_t i=0; i<4; ++i) {
        Scalar v_req = cmds.throttle_01[i] * v_battery; 
        Scalar v_applied = escs[i].apply(v_req, v_battery, motors[i]);
        motors[i].update(dt, v_applied, 0.0); 
        Scalar rpm = motors[i].getRPM();

        auto [thrust, drag_torque] = propellers[i].getAerodynamics(rpm);
        total_thrust += thrust;

        // Gyroscopic Momentum
        // CCW (+Z rotation) props have +H. CW (-Z rotation) props have -H.
        double prop_sign = (i==0 || i==1) ? 1.0 : -1.0; // CCW=1, CW=-1
        double omega_prop = (rpm * 2.0 * M_PI / 60.0) * prop_sign;
        H_props.z() += 0.0001 * omega_prop; 

        // Torque Calculation (r x F) + Drag Torque
        // Thrust acts +Z. 
        // r = position of motor.
        // Torque_Thrust = (y*Fz) i + (-x*Fz) j
        // Drag Torque: Opposes rotation. CCW Prop -> CW Torque (-Z). CW Prop -> CCW Torque (+Z).
        
        Scalar Tx_thrust = 0;
        Scalar Ty_thrust = 0;
        Scalar Tz_drag = 0;

        if (i==0) { // FR (+X, -Y) CCW
            Tx_thrust = (-L) * thrust;  // Roll Left (-X) ? Wait. -y*Fz. -(-L)*T = +LT. 
                                        // r=(L, -L, 0). F=(0,0,T). r x F = (-LT, -LT, 0).
                                        // Wait. rxF. | i  j  k |
                                        //           | L -L  0 |
                                        //           | 0  0  T |
                                        // i: (-L*T - 0) = -LT. (Torque -X / Left Wing Down / Left Roll)
                                        // j: -(L*T - 0) = -LT. (Torque -Y / Nose Up / Pitch Up)
            Ty_thrust = (-L) * thrust; 
            Tz_drag   = -drag_torque;   // CCW prop -> CW torque (-Z)
        } else if (i==1) { // RL (-X, +Y) CCW
            // r=(-L, L, 0)
            // i: (L*T) = +LT. (Torque +X / Right Wing Down / Right Roll)
            // j: -(-L*T) = +LT. (Torque +Y / Nose Down / Pitch Down)
            Tx_thrust = (L) * thrust;
            Ty_thrust = (L) * thrust;
            Tz_drag   = -drag_torque;
        } else if (i==2) { // FL (+X, +Y) CW
            // r=(L, L, 0)
            // i: (L*T) = +LT. (Torque +X)
            // j: -(L*T) = -LT. (Torque -Y)
            Tx_thrust = (L) * thrust;
            Ty_thrust = (-L) * thrust;
            Tz_drag   = +drag_torque;   // CW prop -> CCW torque (+Z)
        } else if (i==3) { // RR (-X, -Y) CW
            // r=(-L, -L, 0)
            // i: (-L*T) = -LT. (Torque -X)
            // j: -(-L*T) = +LT. (Torque +Y)
            Tx_thrust = (-L) * thrust;
            Ty_thrust = (L) * thrust;
            Tz_drag   = +drag_torque;
        }
        
        torque_aero.x() += Tx_thrust;
        torque_aero.y() += Ty_thrust;
        torque_aero.z() += Tz_drag;
    }

    // Forces
    Vec3 force_body(0, 0, total_thrust);
    
    Vec3 vel_body = ori.inverse() * vel;
    Vec3 drag_body;
    drag_body.x() = -0.5 * 1.225 * 0.1 * vel_body.x() * std::abs(vel_body.x());
    drag_body.y() = -0.5 * 1.225 * 0.1 * vel_body.y() * std::abs(vel_body.y());
    drag_body.z() = -0.5 * 1.225 * 0.2 * vel_body.z() * std::abs(vel_body.z()); 
    last_drag = ori * drag_body;

    Vec3 force_world = (ori * (force_body + drag_body)) + Vec3(0, 0, -9.81 * MASS);

    // Moments
    Vec3 torque_gyro = -ang_vel.cross(H_props);
    last_gyro_moment = torque_gyro;

    Vec3 total_torque = torque_aero + torque_gyro;

    // Integration
    Vec3 acc = force_world / MASS;
    
    // Collision w/ Ground
    if (pos.z() < 0 && acc.z() < 0) {
        acc.z() = 0; vel.z() = 0; pos.z() = 0;
        vel.x() *= 0.5; vel.y() *= 0.5;
    }

    vel += acc * dt;
    pos += vel * dt;

    // Angular Acceleration
    Vec3 gyro_term = ang_vel.cross(INERTIA_TENSOR * ang_vel);
    Vec3 alpha = INERTIA_TENSOR.inverse() * (total_torque - gyro_term);
    ang_vel += alpha * dt;

    // Orientation
    Quat q_dot; q_dot.w() = 0; q_dot.vec() = ang_vel;
    Quat deriv = ori * q_dot;
    ori.w() += deriv.w() * 0.5 * dt;
    ori.vec() += deriv.vec() * 0.5 * dt;
    ori.normalize();
}

Telemetry Drone::getTelemetry(double t) const {
    Telemetry data;
    data.time = t;
    data.pos_true = pos;
    data.vel_true = vel;
    data.ori_true = ori;
    data.pos_est = estimator.getPosition();
    data.vel_est = estimator.getVelocity();
    data.ori_est = estimator.getAttitude();
    data.target_pos = last_inputs.target_pos;
    data.force_drag = last_drag;
    data.moment_gyro = last_gyro_moment;
    
    for(size_t i=0; i<4; ++i) {
        data.motor_rpm[i] = motors[i].getRPM();
        data.motor_current[i] = motors[i].getCurrent();
    }
    return data;
}