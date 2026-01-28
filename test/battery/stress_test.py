import subprocess
import pandas as pd
import matplotlib.pyplot as plt
import io
import os
import sys

# --- CONFIG ---
EXECUTABLE = "./battery_test"
K_STD = 298.15
K_COLD = 263.15
K_HOT = 313.15

class BatteryTester:
    def __init__(self, exe_path):
        self.exe = exe_path
        if not os.path.exists(self.exe):
            print(f"Error: Executable '{self.exe}' not found.")
            sys.exit(1)

    def run(self, profile, peak_amps, amb_temp, init_temp, duration):
        args = [profile, str(peak_amps), str(amb_temp), str(init_temp), str(duration)]
        cmd = [self.exe] + args
        print(f"Running: {profile.upper()}...")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return pd.read_csv(io.StringIO(result.stdout))
        except subprocess.CalledProcessError as e:
            print(f"Failed: {e}")
            return None

    def show_dashboard(self, df, title):
        """Standard Dashboard for Tests 1-6"""
        if df is None: return

        fig = plt.figure(figsize=(16, 10))
        fig.canvas.manager.set_window_title(title)
        fig.suptitle(title, fontsize=18, fontweight='bold')
        
        gs = fig.add_gridspec(2, 2)

        # PLOT 1: Truth vs Noise
        ax1 = fig.add_subplot(gs[0, 0])
        color = 'tab:blue'
        ax1.plot(df['time'], df['voltage_noisy'], color='lightblue', alpha=0.6, label='Sensor (Noisy)')
        ax1.plot(df['time'], df['voltage'], color='blue', linewidth=2, label='Physics (Truth)')
        ax1.set_ylabel('Voltage (V)', color=color, fontsize=12)
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.grid(True, linestyle='--', alpha=0.6)
        ax1.legend(loc='lower left')
        ax1.set_title("Sensor Simulation (Truth vs Noise)", fontsize=14)

        ax1b = ax1.twinx()
        color = 'tab:orange'
        ax1b.plot(df['time'], df['current'], color=color, linestyle='--', alpha=0.8, label='Current Load')
        ax1b.set_ylabel('Current (A)', color=color, fontsize=12)
        ax1b.tick_params(axis='y', labelcolor=color)

        # PLOT 2: Efficiency
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.fill_between(df['time'], df['power_W'], color='green', alpha=0.3, label='Output Power (W)')
        ax2.plot(df['time'], df['power_W'], color='green', linewidth=1)
        ax2.plot(df['time'], df['heat_W'], color='red', linewidth=2, label='Heat Loss (W)')
        ax2.set_ylabel("Power (Watts)", fontsize=12)
        ax2.legend(loc='upper right')
        ax2.grid(True)
        ax2.set_title("Efficiency & Heat Loss", fontsize=14)

        # PLOT 3: Thermal & Health
        ax3 = fig.add_subplot(gs[1, 0])
        color = 'tab:red'
        ax3.plot(df['time'], df['temperature_K'], color=color, linewidth=2, label='Temp (K)')
        ax3.axhline(y=333.15, color='k', linestyle=':', label='Max Safe (333K)')
        ax3.set_ylabel('Temperature (K)', color=color, fontsize=12)
        ax3.tick_params(axis='y', labelcolor=color)
        
        if df['health'].max() > 0:
            ax3b = ax3.twinx()
            color = 'purple'
            ax3b.plot(df['time'], df['health'], color=color, linestyle='-.', linewidth=2, label='Damage Factor')
            ax3b.set_ylabel('Permanent Damage', color=color, fontsize=12)
            ax3b.tick_params(axis='y', labelcolor=color)
            ax3.set_facecolor('#fff0f0')

        ax3.set_title("Thermal Runaway & Health", fontsize=14)
        ax3.grid(True)

        # PLOT 4: Impedance
        ax4 = fig.add_subplot(gs[1, 1])
        color = 'tab:gray'
        ax4.plot(df['time'], df['resistance'] * 1000, color=color, linewidth=2, label='Internal R (mΩ)')
        ax4.set_ylabel('Resistance (mΩ)', color=color, fontsize=12)
        ax4.tick_params(axis='y', labelcolor=color)
        
        ax4b = ax4.twinx()
        color = 'tab:cyan'
        ax4b.plot(df['time'], df['energy_J'] / 1000.0, color=color, linestyle='--', linewidth=2, label='Energy Used (kJ)')
        ax4b.set_ylabel('Energy Consumed (kJ)', color=color, fontsize=12)
        ax4b.tick_params(axis='y', labelcolor=color)
        
        ax4.set_title("Impedance & Energy Consumption", fontsize=14)
        ax4.grid(True)

        plt.tight_layout()

    def show_hysteresis_dashboard(self, df, title):
        """Specialized Dashboard for Test 7 (Hysteresis)"""
        if df is None: return

        fig = plt.figure(figsize=(16, 10))
        fig.canvas.manager.set_window_title(title)
        fig.suptitle(title, fontsize=18, fontweight='bold')
        
        gs = fig.add_gridspec(2, 2)

        # PLOT 1: Electrical State
        ax1 = fig.add_subplot(gs[0, 0])
        color = 'tab:blue'
        ax1.plot(df['time'], df['voltage'], color=color, linewidth=2, label='Voltage')
        ax1.set_ylabel('Voltage (V)', color=color)
        ax1b = ax1.twinx()
        color = 'tab:orange'
        ax1b.plot(df['time'], df['current'], color=color, linestyle='--', label='Current')
        ax1b.set_ylabel('Current (A)', color=color)
        ax1.set_title("Electrical State (Charge/Discharge Cycle)", fontsize=14)
        ax1.grid(True)

        # PLOT 2: Hysteresis State (The Offset)
        ax2 = fig.add_subplot(gs[0, 1])
        color = 'tab:purple'
        ax2.plot(df['time'], df['hyst_v'], color=color, linewidth=2)
        ax2.set_ylabel("Hysteresis Offset (V)", color=color)
        ax2.set_title("Internal Hysteresis State ('Voltage Drag')", fontsize=14)
        ax2.grid(True)
        ax2.axhline(y=0, color='k', linestyle=':')

        # PLOT 3: The Loop (Path Dependence)
        ax3 = fig.add_subplot(gs[1, 0])
        # Scatter plot colored by time to show direction of loop
        sc = ax3.scatter(df['soc']*100, df['voltage'], c=df['time'], cmap='viridis', s=15)
        plt.colorbar(sc, ax=ax3, label='Time (s)')
        ax3.invert_xaxis() # High SoC on left
        ax3.set_xlabel("SoC (%)")
        ax3.set_ylabel("Terminal Voltage (V)")
        ax3.set_title("Hysteresis Loop (OCV Path Dependence)", fontsize=14)
        ax3.grid(True)
        
        # PLOT 4: Energy
        ax4 = fig.add_subplot(gs[1, 1])
        ax4.plot(df['time'], df['energy_J'] / 1000.0, color='cyan', linewidth=2)
        ax4.set_ylabel('Energy Consumed (kJ)')
        ax4.set_title("Cumulative Energy", fontsize=14)
        ax4.grid(True)

        plt.tight_layout()

if __name__ == "__main__":
    tester = BatteryTester(EXECUTABLE)

    # --- TEST 1: Mission Profile ---
    df1 = tester.run("mission", 40.0, K_STD, K_STD, 300)
    tester.show_dashboard(df1, "TEST 1: Mission Profile (Noisy Sensor Check)")

    # --- TEST 2: Winter Cold Soak ---
    df2 = tester.run("constant", 20.0, K_COLD, K_COLD, 200)
    tester.show_dashboard(df2, "TEST 2: Winter Cold Soak (-10C)")

    # --- TEST 3: Thermal Abuse ---
    df3 = tester.run("abuse", 50.0, K_HOT, K_HOT, 400)
    tester.show_dashboard(df3, "TEST 3: Thermal Abuse (Puff Factor)")

    # --- TEST 4: Regen Braking ---
    df4 = tester.run("regen", 25.0, K_STD, K_STD, 150)
    tester.show_dashboard(df4, "TEST 4: Regen Braking (Efficiency Check)")

    # --- TEST 5: Turbulence ---
    df5 = tester.run("sine", 30.0, K_STD, K_STD, 150)
    tester.show_dashboard(df5, "TEST 5: Turbulence (Sine Wave)")

    # --- TEST 6: Deep Discharge ---
    df6 = tester.run("constant", 15.0, K_STD, K_STD, 400)
    tester.show_dashboard(df6, "TEST 6: Deep Discharge")

    # --- TEST 7: Hysteresis Loop ---
    # Low current (10A) to clearly see the OCV gap without massive sag masking it
    df7 = tester.run("hysteresis", 10.0, K_STD, K_STD, 400)
    tester.show_hysteresis_dashboard(df7, "TEST 7: Hysteresis Loop Verification")

    print("\nRendering all test windows... (Close windows to exit)")
    plt.show()