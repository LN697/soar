import subprocess
import pandas as pd
import matplotlib.pyplot as plt
import io
import os
import sys

# Update if necessary
EXECUTABLE = "./propulsion_test"

class SystemTester:
    def __init__(self):
        if not os.path.exists(EXECUTABLE):
            print(f"Error: Executable {EXECUTABLE} not found.")
            print("Compile: g++ -std=c++17 test_runner.cpp motor.cpp propeller.cpp -o propulsion_test")
            sys.exit(1)

    def run(self, profile, duration):
        try:
            cmd = [EXECUTABLE, profile, str(duration)]
            res = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return pd.read_csv(io.StringIO(res.stdout))
        except Exception as e:
            print(f"Error running {profile}: {e}")
            return None

    def plot_window(self, df, title, plot_type):
        fig = plt.figure(figsize=(12, 8))
        fig.canvas.manager.set_window_title(title)
        plt.suptitle(title, fontsize=16, fontweight='bold')

        # Layout depends on test type
        if plot_type == "standard":
            # Grid 2x2: Electrical, Mechanical, Efficiency, Temp
            ax1 = plt.subplot(221)
            ax1.plot(df['time'], df['voltage'], 'b--', label='Volts In')
            ax1.plot(df['time'], df['current_truth'], 'r', label='Current')
            ax1.set_title("Electrical Input"); ax1.legend(); ax1.grid(True)

            ax2 = plt.subplot(222)
            ax2.plot(df['time'], df['rpm_truth'], 'g', label='RPM')
            ax2.plot(df['time'], df['thrust_N']*1000, 'm', label='Thrust (mN)') # Scaled for visibility
            ax2.set_title("Mechanical Output"); ax2.legend(); ax2.grid(True)

            ax3 = plt.subplot(223)
            ax3.plot(df['time'], df['efficiency']*100, 'k')
            ax3.set_ylabel("Efficiency %"); ax3.grid(True)

            ax4 = plt.subplot(224)
            ax4.plot(df['time'], df['temp_K'], 'r')
            ax4.set_ylabel("Motor Temp (K)"); ax4.grid(True)

        elif plot_type == "noise":
            # Zoom in on sensors
            plt.plot(df['time'], df['current_sens'], 'r-', alpha=0.4, label='Current (Sensor)')
            plt.plot(df['time'], df['current_truth'], 'k--', linewidth=1, label='Current (Truth)')
            plt.plot(df['time'], df['rpm_sens']/1000, 'b-', alpha=0.4, label='RPM/1k (Sensor)')
            plt.plot(df['time'], df['rpm_truth']/1000, 'g--', linewidth=1, label='RPM/1k (Truth)')
            plt.title("Sensor Noise Analysis"); plt.legend(); plt.grid(True)

        elif plot_type == "h_force":
            ax1 = plt.gca()
            ax1.plot(df['env_airspeed'], df['h_force_N'], 'purple', linewidth=2, label='H-Force Drag')
            ax1.set_xlabel("Airspeed (m/s)"); ax1.set_ylabel("Drag (N)", color='purple')
            ax2 = ax1.twinx()
            ax2.plot(df['env_airspeed'], df['thrust_N'], 'g--', label='Thrust')
            ax2.set_ylabel("Thrust (N)", color='g')
            plt.title("Forward Flight Aerodynamics"); plt.grid(True)

        elif plot_type == "vrs":
            ax1 = plt.gca()
            ax1.plot(df['time'], df['env_vel_z'], 'b', label='Descent Velocity')
            ax1.set_ylabel("Vel Z (m/s)", color='b')
            ax2 = ax1.twinx()
            ax2.plot(df['time'], df['thrust_N'], 'r', linewidth=2, label='Thrust')
            ax2.set_ylabel("Thrust (N)", color='r')
            plt.title("Vortex Ring State (Thrust Collapse)"); plt.grid(True)

        elif plot_type == "cogging":
            plt.plot(df['angle_rad'], df['current_truth'], 'orange')
            plt.xlabel("Rotor Angle (rad)"); plt.ylabel("Current (A)")
            plt.title("Cogging Torque Ripple (Low Speed)"); plt.grid(True)
            plt.xlim(0, 15)

        elif plot_type == "ground":
            ax1 = plt.gca()
            ax1.plot(df['env_height'], df['thrust_N'], 'g', linewidth=2)
            ax1.set_xlabel("Height AGL (m)"); ax1.set_ylabel("Thrust (N)")
            ax1.invert_xaxis() # 0 is on right usually, but standard plot is fine
            plt.title("Ground Effect Cushion"); plt.grid(True)

        plt.tight_layout()

if __name__ == "__main__":
    t = SystemTester()
    
    # 1. Step Response (Dynamics)
    t.plot_window(t.run("step", 0.2), "TEST 1: Step Response (Inductance & Inertia)", "standard")

    # 2. Ramp Up (Linearity Check)
    t.plot_window(t.run("ramp", 4.0), "TEST 2: Linear Ramp & Efficiency Curve", "standard")

    # 3. Mach Limit (Compressibility)
    t.plot_window(t.run("mach", 6.0), "TEST 3: Mach Tip Loss (Thrust Saturation)", "standard")

    # 4. Stall (Burnout)
    t.plot_window(t.run("stall", 5.0), "TEST 4: Locked Rotor Stall (Thermal Runaway)", "standard")

    # 5. Vortex Ring State
    t.plot_window(t.run("vrs", 5.0), "TEST 5: VRS Descent (Thrust Instability)", "vrs")

    # 6. Ground Effect
    t.plot_window(t.run("ground", 5.0), "TEST 6: Ground Effect (Thrust Boost)", "ground")

    # 7. H-Force (Drag)
    t.plot_window(t.run("h_force", 5.0), "TEST 7: Forward Flight H-Force", "h_force")

    # 8. Cogging Torque
    t.plot_window(t.run("cogging", 2.0), "TEST 8: Cogging Torque Ripple", "cogging")

    # 9. Thermal Fade (Endurance)
    t.plot_window(t.run("thermal", 60.0), "TEST 9: Thermal Fade (Resistance Rise)", "standard")

    # 10. Sensor Noise
    t.plot_window(t.run("noise", 1.0), "TEST 10: Sensor Noise Characterization", "noise")

    plt.show()