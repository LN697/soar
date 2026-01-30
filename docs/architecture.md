# Architecture - Soar 

This page describes the main subsystems and how they interact.

## Subsystems

- **Core / Physics** (`core/include/physics.h`)
  - `RigidBody` integrates position and orientation, collects forces and torques.
  - `ForceGenerator` and `Constraint` are extension points for environment or contact forces.

- **Battery** (`drone/include/battery.h`)
  - `LiPoBattery` models OCV curve, internal resistance, thermal state, hysteresis and noisy outputs.

- **Motor** (`drone/include/motor.h`)
  - `BLDCMotor` models electrical-to-mechanical conversion (Kv, resistance), torque, thermal dynamics, and measurement noise.

- **ESC** (`drone/include/esc.h`)
  - `ESC` simulates input filtering, current limiting, LVC, OCP, capacitor ESR, desync detection, and loss/efficiency calculations.

- **Propeller** (`drone/include/propeller.h`)
  - Produces thrust/power/torque using static coefficients and induced-velocity dynamics; includes ground-effect adjustment.

- **IMU** (`drone/include/imu.h`)
  - Sensor model with configurable noise, bias walk, alignment, temperature coefficients and quantization.

- **Controller & Estimator** (`drone/include/controller.h`, `drone/include/estimator.h`)
  - Cascaded PID controller (position → attitude → rate).
  - Simple complementary-ish estimator for attitude and pos/vel fusion.

## Data Flow

1. Controller computes motor throttle commands from target state & estimator outputs.
2. ESC converts throttle to voltage/current and applies hardware limits; ESC output drives motor.
3. Motor spins and applies torque to the propeller; propeller computes thrust and torque back onto the `RigidBody`.
4. Physics step integrates motion; IMU and other sensors sample the true state with imperfections.
5. Estimator ingests IMU (and optional external updates) to produce attitude/velocity for the controller.

## Extensibility

Add new models by providing a matching header/implementation and hooking it into the simulation runner. ForceGenerators and Constraints are available for adding environmental effects (wind, contacts).
