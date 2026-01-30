# Usage - Soar 

Quick steps to build and run the simulator locally and visualize results.

## Prerequisites
- C++ toolchain supporting C++17 (g++/clang).
- Eigen headers available at `lib/Eigen` (header-only).
- Python 3 with `pandas` and `matplotlib` for plotting.

## Build & Run

```bash
# Add Eigen (example)
# git submodule add https://gitlab.com/libeigen/eigen.git lib/Eigen

# Build
make all

# Run the visualizer (compiles & executes sim)
python3 test/plot.py
```

## Running component tests

The `Makefile` contains commented examples for compiling small component test binaries (battery, esc, propulsion, imu, and flight controller). Uncomment or adapt the lines under the `test` rule to compile and run them.

## Extending the simulator
- To add a new sensor or actuator, follow the patterns in `drone/include/` and implement a small test harness in `test/`.
- For scenario work, fork `main/main.cpp` into `scenarios/` and add scenario-specific logs or forced external inputs.

---
