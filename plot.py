import pandas as pd
import matplotlib.pyplot as plt
import subprocess
import sys
import os

def run_simulation():
    print("--- [Auto-Pilot] Starting Build & Run Sequence ---")

    if os.path.exists("telemetry.csv"):
        os.remove("telemetry.csv")

    # Step A: Compile (Optional, assumes Makefile exists)
    print("Building C++ binary...")
    subprocess.run(["make", "all"], check=True)

    # Step B: Run Simulation
    print("Running Simulation Kernel...")
    with open("telemetry.csv", "w") as outfile:
        # Calls the executable directly and pipes stdout to CSV
        result = subprocess.run(["./sim"], stdout=outfile, text=True)
        
    if result.returncode != 0:
        print("Error: Simulation crashed!")
        sys.exit(1)
        
    print("Simulation Complete. Data logged to telemetry.csv")

# --- 2. Visualization Section ---
def plot_analysis():
    # Load Data
    try:
        df = pd.read_csv('telemetry.csv')
    except Exception as e:
        print(f"Failed to read CSV: {e}")
        return

    # Derive Acceleration (dv/dt) for smoother analysis
    df['AccelZ_Calc'] = df['VelZ'].diff() / df['Time'].diff()
    # Smooth the acceleration slightly to remove numerical noise from differentiation
    df['AccelZ_Calc'] = df['AccelZ_Calc'].rolling(window=5).mean()

    # Setup Dashboard (3 Rows, 1 Column)
    plt.style.use('bmh') # Clean engineering style
    fig, (ax_elec, ax_mech, ax_kin) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    
    fig.suptitle('Drone System Step Response (0V -> 12V -> 0V)', fontsize=16)

    # --- ROW 1: Electrical System (Voltage & Current) ---
    # We use dual y-axes here
    color_v = 'tab:red'
    color_i = 'tab:orange'
    
    ax_elec.set_ylabel('Input Voltage (V)', color=color_v, fontweight='bold')
    l1 = ax_elec.plot(df['Time'], df['Voltage'], color=color_v, label='Voltage In', linestyle='--')
    ax_elec.tick_params(axis='y', labelcolor=color_v)
    ax_elec.grid(True, alpha=0.3)

    ax_elec2 = ax_elec.twinx()  # Create second y-axis
    ax_elec2.set_ylabel('Total Current (A)', color=color_i, fontweight='bold')
    l2 = ax_elec2.plot(df['Time'], df['Current'], color=color_i, label='Current Draw')
    ax_elec2.tick_params(axis='y', labelcolor=color_i)
    
    # Combine legends
    lns = l1 + l2
    labs = [l.get_label() for l in lns]
    ax_elec.legend(lns, labs, loc='upper left')
    ax_elec.set_title('Electrical Domain', fontsize=10)

    # --- ROW 2: Propulsion Domain (RPM & Inertia) ---
    color_rpm = 'tab:purple'
    ax_mech.plot(df['Time'], df['RPM'], color=color_rpm, linewidth=2)
    ax_mech.set_ylabel('Motor Speed (RPM)', color=color_rpm, fontweight='bold')
    ax_mech.fill_between(df['Time'], df['RPM'], alpha=0.1, color=color_rpm)
    ax_mech.set_title('Mechanical Domain (Inertia Lag)', fontsize=10)
    
    # Annotation for Lag
    peak_rpm_time = df.loc[df['RPM'].idxmax(), 'Time']
    # If the step down was at 0.6, calculate lag
    lag_time = peak_rpm_time - 0.6
    if lag_time > 0:
        ax_mech.annotate(f'Inertia Lag: {lag_time*1000:.0f}ms', 
                         xy=(peak_rpm_time, df['RPM'].max()), 
                         xytext=(peak_rpm_time+0.1, df['RPM'].max()),
                         arrowprops=dict(facecolor='black', shrink=0.05))

    # --- ROW 3: Kinematic Domain (Velocity & Accel) ---
    color_vel = 'tab:blue'
    color_acc = 'tab:green'
    
    ax_kin.plot(df['Time'], df['VelZ'], color=color_vel, label='Vertical Vel (m/s)', linewidth=2)
    ax_kin.set_ylabel('Velocity (m/s)', color=color_vel, fontweight='bold')
    ax_kin.axhline(0, color='black', linewidth=1)
    
    # Dual axis for Acceleration
    ax_kin2 = ax_kin.twinx()
    ax_kin2.plot(df['Time'], df['AccelZ_Calc'], color=color_acc, linestyle=':', label='G-Force (Calc)', alpha=0.6)
    ax_kin2.set_ylabel('Accel (m/s²)', color=color_acc)
    
    ax_kin.legend(loc='upper left')
    ax_kin.set_title('Kinematic Response', fontsize=10)
    ax_kin.set_xlabel('Time (s)', fontsize=12, fontweight='bold')

    plt.tight_layout()
    print("Dashboard generated: Displaying...")
    plt.show()

if __name__ == "__main__":
    run_simulation()
    plot_analysis()