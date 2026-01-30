import subprocess
import pandas as pd
import matplotlib.pyplot as plt
import io
import os
import sys

EXECUTABLE = "./build/imu_test"

class IMUTester:
    def __init__(self):
        if not os.path.exists("imu_test"):
            print("Starting IMU Simulation...")

    def run(self, profile, duration, sensor="mpu6000"):
        cmd = [EXECUTABLE, profile, str(duration), sensor]
        res = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return pd.read_csv(io.StringIO(res.stdout))

    def plot_coupling(self, df, title):
        fig, ax = plt.subplots(figsize=(10, 6))
        
        # Plot Input
        ax.plot(df['time'], df['true_ax'], 'k--', label='True Input (Pure X)', linewidth=2)
        
        # Plot Bleed-through
        # If perfect, Meas Y should be 0.
        ax.plot(df['time'], df['meas_ay'], 'r-', label='Measured Y (Cross-Talk)')
        ax.plot(df['time'], df['meas_az'] - 9.81, 'g-', label='Measured Z (Cross-Talk)')
        
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Accel (m/s^2)")
        ax.grid(True)
        ax.legend()
        plt.show()

    def plot_g_sense(self, df, title):
        fig, ax = plt.subplots(figsize=(10, 6))
        
        ax.plot(df['time'], df['true_ay'], 'b-', label='Linear Accel (5G)')
        
        # Plot Gyro Response (Should be 0, but G-sensitivity makes it non-zero)
        # Plot on secondary axis
        ax2 = ax.twinx()
        ax2.plot(df['time'], df['meas_gx'] * 57.3, 'r-', label='False Gyro Output (deg/s)')
        ax2.set_ylabel("Gyro (deg/s)", color='r')
        
        ax.set_title(title)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Accel (m/s^2)", color='b')
        ax.grid(True)
        plt.show()

if __name__ == "__main__":
    t = IMUTester()

    # TEST 7: Cross-Axis Coupling (Bad Mount)
    # We apply X accel. We check if Y/Z react.
    df7 = t.run("coupling", 5.0, "bad_mount")
    if df7 is not None:
        t.plot_coupling(df7, "TEST 7: Misalignment & Cross-Axis Coupling")

    # TEST 8: Gyro G-Sensitivity
    # We apply 5G linear force. We check if Gyro reports rotation.
    df8 = t.run("g_sensitivity", 2.0, "mpu6000")
    if df8 is not None:
        t.plot_g_sense(df8, "TEST 8: Gyro G-Sensitivity (Linear Accel Artifacts)")