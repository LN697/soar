import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import subprocess
import os
import sys

def run_simulation():
    print("--- [Soar] Compiling & Running ---")
    if os.path.exists("telemetry.csv"): os.remove("telemetry.csv")
    
    try:
        subprocess.run(["make", "all"], check=True)
        with open("telemetry.csv", "w") as outfile:
            subprocess.run(["./sim"], stdout=outfile, text=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[F] Failed: {e}")
        sys.exit(1)
    print("[P] Data Generated.")

class DroneVisualizer:
    def __init__(self, df):
        self.df = df
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.ax.set_title("6-DOF Flight Replay")
        self.ax.set_xlim(-2, 5); self.ax.set_ylim(-2, 5); self.ax.set_zlim(0, 6)
        
        self.line_true, = self.ax.plot([],[],[], 'b-', alpha=0.6, label='True Path')
        self.line_est, = self.ax.plot([],[],[], 'r--', alpha=0.6, label='Est Path')
        self.drone_frame, = self.ax.plot([],[],[], 'k-', linewidth=3)
        self.ax.legend()

    def get_rot(self, qx, qy, qz, qw):
        # Quaternion to Matrix
        return np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

    def update(self, num):
        row = self.df.iloc[num]
        
        # Paths
        h = self.df.iloc[:num]
        self.line_true.set_data(h['PX'], h['PY']); self.line_true.set_3d_properties(h['PZ'])
        self.line_est.set_data(h['EX'], h['EY']); self.line_est.set_3d_properties(h['EZ'])
        
        # Drone Body
        pos = np.array([row['PX'], row['PY'], row['PZ']])
        R = self.get_rot(row['QX'], row['QY'], row['QZ'], row['QW'])
        
        arms = np.array([[0.25,0,0], [-0.25,0,0], [0,0.25,0], [0,-0.25,0]]).T
        rot_arms = (R @ arms).T + pos
        
        lx = [rot_arms[0,0], rot_arms[1,0], np.nan, rot_arms[2,0], rot_arms[3,0]]
        ly = [rot_arms[0,1], rot_arms[1,1], np.nan, rot_arms[2,1], rot_arms[3,1]]
        lz = [rot_arms[0,2], rot_arms[1,2], np.nan, rot_arms[2,2], rot_arms[3,2]]
        
        self.drone_frame.set_data(lx, ly)
        self.drone_frame.set_3d_properties(lz)
        return self.drone_frame,

def plot_analysis(df):
    fig, axs = plt.subplots(3, 2, figsize=(15, 10))
    t = df['Time']
    
    # 1. Position Step Response
    axs[0,0].set_title("Position Tracking (X)")
    axs[0,0].plot(t, df['TX'], 'g--', label='Target')
    axs[0,0].plot(t, df['PX'], 'b', label='True')
    axs[0,0].plot(t, df['EX'], 'r:', label='Est')
    axs[0,0].legend()
    
    # 2. Z Step Response
    axs[0,1].set_title("Altitude Tracking (Z)")
    axs[0,1].plot(t, df['TZ'], 'g--', label='Target')
    axs[0,1].plot(t, df['PZ'], 'b', label='True')
    axs[0,1].legend()

    # 3. Orientation Error (Est vs True)
    # Simple approx error
    axs[1,0].set_title("Orientation Estimation Error (Quaternion Dist)")
    err = np.sqrt((df['QX']-df['EQX'])**2 + (df['QY']-df['EQY'])**2 + (df['QZ']-df['EQZ'])**2)
    axs[1,0].plot(t, err, 'k', label='Error Norm')
    axs[1,0].legend()

    # 4. Motor RPMs
    axs[1,1].set_title("Motor Usage")
    axs[1,1].plot(t, df['RPM0'], label='M1')
    axs[1,1].plot(t, df['RPM1'], label='M2')
    axs[1,1].plot(t, df['RPM2'], label='M3')
    axs[1,1].plot(t, df['RPM3'], label='M4')
    axs[1,1].legend()

    # 5. Drag Forces
    axs[2,0].set_title("Aerodynamic Drag Force (World)")
    axs[2,0].plot(t, df['DragX'], label='X')
    axs[2,0].plot(t, df['DragY'], label='Y')
    axs[2,0].plot(t, df['DragZ'], label='Z')
    axs[2,0].legend()
    
    # 6. Gyro Moments
    axs[2,1].set_title("Gyroscopic Moments (Body)")
    axs[2,1].plot(t, df['GyroMX'], label='X')
    axs[2,1].plot(t, df['GyroMY'], label='Y')
    axs[2,1].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    run_simulation()
    try:
        df = pd.read_csv('telemetry.csv')
        plot_analysis(df)
        vis = DroneVisualizer(df)
        ani = animation.FuncAnimation(vis.fig, vis.update, frames=len(df), interval=20)
        plt.show()
    except Exception as e:
        print(f"Error: {e}")