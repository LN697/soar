# Soar

**Soar** is a lightweight, modular drone simulation framework implemented in modern C++ (C++17).
It focuses on physically meaningful component models (battery, motor, ESC, propeller, IMU), a small-body physics integrator, and a simple control & estimation stack so you can prototype flight control algorithms, study propulsion/thermal behaviour, and visualize telemetry.

---

## Key Features

- **Modular components**: Separate, testable models for Battery, Motor (BLDC), ESC, Propeller, IMU, Estimator, Controller, and a small **RigidBody** physics system.
- **Physical modelling**:
  - LiPo battery model with Peukert-like behaviour, internal resistance, thermal dynamics, hysteresis, and noisy measurements.
  - BLDC motor model capturing Kv, resistance, torque, thermal state, and measurement noise.
  - ESC model with thermal modelling, input filtering, current limiting, LVC, OCP, desync handling, and efficiency/power-loss calculations.
  - Propeller aerodynamic model including induced velocity and simple ground-effect behaviour.
  - IMU model with configurable noise, bias walk, temperature coefficients, saturation & quantization.
- **Control & Estimation**:
  - Cascaded PID-based flight controller (position → attitude → rate loops).
  - Simple complementary-style estimator for attitude and velocity/position fusion.
- **Testing & Visualization**:
  - Per-component stress tests and Python plotting scripts to replay telemetry and analyze performance (using pandas + matplotlib).
- **Single-file binary build** via the included Makefile (g++ & header-only Eigen dependency).

---

## Project Structure

- `core/` — Core math/physics utilities and small multibody stepper (RigidBody, ForceGenerators).
- `drone/` — Drone subsystem models and flight components (headers & implementations).
- `pid/` — Simple PID library used by the flight controller.
- `main/` — Example `main.cpp` and top-level simulation runner (produces `telemetry.csv` for plotting).
- `test/` — Test harnesses and Python plotting utility (`test/plot.py`).
- `lib/` — External libraries (use `lib/Eigen` for the Eigen headers).

---

## Build & Run

Prerequisites:
- A C++17 toolchain (g++ or clang).
- Eigen headers (header-only) placed at `lib/Eigen` or added as a git submodule.
- Python 3 with `pandas` and `matplotlib` for visualization (if you want plots).

Recommended steps:

1. Add Eigen (example):

```bash
# Option A: Copy Eigen headers into lib/Eigen
mkdir -p lib
# download and extract Eigen into lib/Eigen, or
# git clone https://gitlab.com/libeigen/eigen.git lib/Eigen

# Option B: Add as a git submodule
git submodule add https://gitlab.com/libeigen/eigen.git lib/Eigen
```

2. Build the simulator:

```bash
make all
```

3. Run the simulation and generate telemetry (the included `test/plot.py` automates this):

```bash
python3 test/plot.py
```

If you want to run individual unit-style tests, see the commented `test` targets in the `Makefile` for example compile/run invocations.

---

## Implementation Details (short)

- **Battery (`drone/include/battery.h`)**: `LiPoBattery` exposes SoC, voltage, internal resistance, heating power and energy consumed. It keeps an OCV curve and models noise on sensor outputs.

- **Motor (`drone/include/motor.h`)**: `BLDCMotor` models motor electrical/mechanical coupling (Kv, dynamic resistance), torque generation, thermal state and noisy RPM/current measurements.

- **ESC (`drone/include/esc.h`)**: `ESC` simulates driver behaviour including input filtering/delay, switching noise, current limiting (soft & hard), over-current protection, low-voltage cutoff, capacitors ESR and thermal dynamics, desync detection and recovery.

- **Propeller (`drone/include/propeller.h`)**: `Propeller` produces thrust/power/torque using static coefficients and models induced velocity with a simple time constant and ground-effect scaling.

- **IMU (`drone/include/imu.h`)**: Configurable noise densities, bias instability, misalignment, temperature coefficients, and simple quantization/limits. Useful for testing estimator robustness.

- **Controller (`drone/include/controller.h`)**: Cascaded PID controllers for position → attitude → body rates. Uses the `PID` helper in `pid/`.

- **Estimator (`drone/include/estimator.h`)**: Lightweight complementary-style filter (alpha blending) for attitude and simple external position update hooks.

- **Physics (`core/include/physics.h`)**: `RigidBody` integrates linear/angular motion, adds forces/torques, and supports extensible `ForceGenerator`/`Constraint` objects for easy experiment setup.

---

## Tests & Visualization

- `test/plot.py` compiles & runs the simulation, captures `telemetry.csv`, and provides:
  - A 3D flight replay animation
  - Time-series diagnostics: position tracking, motor RPMs, drag, gyro moments, and estimation errors

- Several test harnesses under `test/` demonstrate per-component behaviours (battery, motor, esc, imu, controller). The tests are currently disabled in the `Makefile` but the sample compile commands are present as commented lines.

---

## Contributing & Roadmap

Contributions welcome — please open issues or PRs. Short-term and long-term plans include:

- Short-term (next steps)
  - Add a CI pipeline to run unit tests automatically.
  - Unblock and expand the per-component stress tests and include automated assertions.
  - Add more sample scenarios (wind gusts, battery ageing cases, payload variations).

- Medium-term (features)
  - Replace the explicit integrator with a proper constraint solver / contact model for more realistic ground interactions.
  - Add GPS/Barometer/Rangefinder sensor models and optional noise/latency.
  - Provide a ROS2 interface or wrapper for HIL testing and logging.

- Long-term (ambitious)
  - Hardware-in-the-loop support, parameter identification tools, and a plugin architecture for custom vehicle models.
  - Performance improvements (multi-threading, vectorized math) for large Monte Carlo studies.

---
