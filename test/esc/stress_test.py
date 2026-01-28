import subprocess
import pandas as pd
import matplotlib.pyplot as plt
import io
import os
import sys

EXECUTABLE = "./esc_test"

class SystemTester:
    def __init__(self):
        if not os.path.exists(EXECUTABLE):
            print(f"Error: {EXECUTABLE} not found.")
            print("Compile: g++ -std=c++17 test_runner.cpp esc.cpp motor.cpp propeller.cpp battery.cpp -o esc_test")
            sys.exit(1)

    def run(self, profile, duration):
        cmd = [EXECUTABLE, profile, str(duration)]
        try:
            print(f"Running Test: {profile}...")
            res = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return pd.read_csv(io.StringIO(res.stdout))
        except Exception as e:
            print(f"Failed: {e}")
            return None

    def plot(self, df, title, plot_type="standard"):
        fig = plt.figure(figsize=(14, 8))
        fig.canvas.manager.set_window_title(title)
        plt.suptitle(title, fontsize=16, fontweight='bold')

        if plot_type == "standard":
            # 2x2 Grid: Voltage, Current, RPM/Thrust, Temp
            ax1 = plt.subplot(221)
            ax1.plot(df['time'], df['throttle']*16, 'k:', label='Input (Scaled)')
            ax1.plot(df['time'], df['esc_out_v'], 'b-', label='ESC Out')
            ax1.set_ylabel("Voltage"); ax1.legend(); ax1.grid(True)
            
            ax2 = plt.subplot(222)
            ax2.plot(df['time'], df['current'], 'r-', label='Current (A)')
            ax2.set_ylabel("Amps"); ax2.legend(); ax2.grid(True)
            
            ax3 = plt.subplot(223)
            ax3.plot(df['time'], df['rpm'], 'g-', label='RPM')
            ax3.set_ylabel("RPM"); ax3.legend(); ax3.grid(True)
            
            ax4 = plt.subplot(224)
            ax4.plot(df['time'], df['esc_temp'], 'r', label='ESC Temp')
            ax4.plot(df['time'], df['cap_temp'], 'orange', label='Cap Temp')
            ax4.set_ylabel("Temp (K)"); ax4.legend(); ax4.grid(True)

        elif plot_type == "protection":
            # Focus on Limits and Flags
            ax1 = plt.subplot(211)
            ax1.plot(df['time'], df['current'], 'r-', label='Current')
            ax1.set_ylabel("Current (A)"); ax1.grid(True)
            
            ax2 = plt.subplot(212)
            ax2.plot(df['time'], df['is_limiting'], 'b-', label='Soft Limit')
            ax2.plot(df['time'], df['is_shutdown'], 'k-', label='Hard Shutdown')
            ax2.plot(df['time'], df['is_stalled'], 'r--', label='Desync')
            ax2.set_ylabel("Logic State (0/1)"); ax2.legend(); ax2.grid(True)

        elif plot_type == "voltage":
            # Focus on Input/Output Voltage and Sag
            plt.plot(df['time'], df['batt_v'], 'g--', label='Battery V')
            plt.plot(df['time'], df['esc_in_v'], 'r-', label='ESC Input (Cable)')
            plt.plot(df['time'], df['esc_out_v'], 'b-', label='ESC Output')
            plt.ylabel("Voltage"); plt.xlabel("Time"); plt.legend(); plt.grid(True)

        elif plot_type == "brownout":
            # Special Brownout Plot
            ax1 = plt.gca()
            ax1.plot(df['time'], df['batt_v'], 'g--', label='Supply Voltage')
            ax1.plot(df['time'], df['esc_out_v'], 'b-', label='ESC Output')
            ax1.axhline(3.0, color='r', linestyle=':', label='MCU Reset (3V)')
            ax2 = ax1.twinx()
            ax2.plot(df['time'], df['is_rebooting'], 'k-', alpha=0.3, label='Rebooting')
            ax2.set_ylabel("Reboot State"); ax1.legend(loc='upper left')
            plt.grid(True)

        elif plot_type == "gate":
            # Gate Drive Efficiency
            ax1 = plt.gca()
            ax1.plot(df['time'], df['batt_v'], 'b-', label='Input Voltage')
            ax2 = ax1.twinx()
            ax2.plot(df['time'], df['resistance'], 'r-', label='MOSFET R_on (mOhm)')
            ax2.set_ylabel("Internal R (mOhm)", color='r')
            ax1.grid(True)

        plt.tight_layout()

if __name__ == "__main__":
    t = SystemTester()

    # --- BASIC DYNAMICS ---
    t.plot(t.run("step", 0.5), "TEST 1: Step Response (Slew Rate)", "standard")
    t.plot(t.run("thermal", 60.0), "TEST 2: Thermal Endurance", "standard")
    
    # --- SIGNAL INTEGRITY ---
    t.plot(t.run("quantization", 1.0), "TEST 3: Quantization & Resolution", "voltage")
    t.plot(t.run("latency", 0.2), "TEST 4: Input Latency (Zoom in on step)", "standard")
    
    # --- PROTECTION & LIMITS ---
    t.plot(t.run("soft_limit", 1.0), "TEST 5: Soft Current Governor", "protection")
    t.plot(t.run("lvc_test", 10.0), "TEST 6: Low Voltage Cutoff (LVC)", "voltage")
    t.plot(t.run("braking", 1.0), "TEST 7: Regen Braking Voltage Clamp", "voltage")
    
    # --- FAILURE MODES ---
    t.plot(t.run("desync", 2.0), "TEST 8: High Load Desync (Death Roll)", "protection")
    t.plot(t.run("spike", 0.3), "TEST 9: Inductive Voltage Spikes", "voltage")
    t.plot(t.run("brownout", 2.0), "TEST 10: MCU Brown-out & Reboot", "brownout")
    t.plot(t.run("gate_sag", 3.0), "TEST 11: Gate Drive Voltage Sag", "gate")
    
    # --- COMPONENT STRESS ---
    t.plot(t.run("ripple", 30.0), "TEST 12: Capacitor Ripple Heating", "standard")

    plt.show()