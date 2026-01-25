import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import subprocess
import os
import sys

# --- 1. Run Simulation ---
def run_simulation():
    print("--- [Auto-Pilot] Building & Running Simulation ---")
    
    # 1. Clean old data
    if os.path.exists("telemetry.csv"): os.remove("telemetry.csv")
    
    # 2. FORCE RECOMPILE (Crucial Fix)
    try:
        print("Compiling...")
        subprocess.run(["make", "all"], check=True)
    except subprocess.CalledProcessError:
        print("[F] Compilation Failed! Check your C++ code.")
        sys.exit(1)

    # 3. Run Binary
    print("Running ./sim...")
    with open("telemetry.csv", "w") as outfile:
        result = subprocess.run(["./sim"], stdout=outfile, text=True)
    
    if result.returncode != 0:
        print("[F] Simulation crashed!")
        sys.exit(1)
    print("[P] Simulation Complete.")

# --- 2. 3D Animation Class ---
class DroneVisualizer:
    def __init__(self, df):
        self.df = df
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("Drone Flight: PID Hover + Wind (Blue) + Thrust (Red)")
        
        # Plot Elements
        self.drone_frame, = self.ax.plot([], [], [], 'k-', linewidth=3, label='Drone Frame')
        self.traj_line, = self.ax.plot([], [], [], 'g:', alpha=0.5, label='Trajectory')
        
        # Vectors (Thrust & Wind)
        self.quivers = []

        # Set Limits
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.ax.set_zlim(-5, 6)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Altitude (m)')
        self.ax.legend()

    def get_rotation_matrix(self, w, x, y, z):
        Nq = w*w + x*x + y*y + z*z
        s = 2.0/Nq if Nq > 0 else 0
        X, Y, Z = x*s, y*s, z*s
        wX, wY, wZ = w*X, w*Y, w*Z
        xX, xY, xZ = x*X, x*Y, x*Z
        yY, yZ = y*Y, y*Z
        zZ = z*Z 
        
        # Corrected Matrix:
        # 1. Added zZ definition above
        # 2. Fixed wF -> wX
        return np.array([
            [1.0-(yY+zZ), xY-wZ, xZ+wY],
            [xY+wZ, 1.0-(xX+zZ), yZ-wX],
            [xZ-wY, yZ+wX, 1.0-(xX+yY)]
        ])

    def update(self, num):
        row = self.df.iloc[num]
        
        # 1. Update Trajectory
        history = self.df.iloc[:num]
        self.traj_line.set_data(history['PosX'], history['PosY'])
        self.traj_line.set_3d_properties(history['PosZ'])

        # 2. Draw Drone Body
        pos = np.array([row['PosX'], row['PosY'], row['PosZ']])
        arm_len = 0.25
        body_arms = np.array([
            [arm_len, 0, 0], [-arm_len, 0, 0], 
            [0, arm_len, 0], [0, -arm_len, 0]
        ]).T 
        
        R = self.get_rotation_matrix(row['Qw'], row['Qx'], row['Qy'], row['Qz'])
        world_arms = R @ body_arms + pos[:, np.newaxis]
        
        lx = [world_arms[0,0], world_arms[0,1]]
        ly = [world_arms[1,0], world_arms[1,1]]
        lz = [world_arms[2,0], world_arms[2,1]]
        
        lx2 = [world_arms[0,2], world_arms[0,3]]
        ly2 = [world_arms[1,2], world_arms[1,3]]
        lz2 = [world_arms[2,2], world_arms[2,3]]
        
        self.drone_frame.set_data(lx + [np.nan] + lx2, ly + [np.nan] + ly2)
        self.drone_frame.set_3d_properties(lz + [np.nan] + lz2)

        # 3. Draw Vectors
        vec_origins = [pos, pos]
        vec_dirs = [
            np.array([row['Fx_Thrust'], row['Fy_Thrust'], row['Fz_Thrust']]),
            np.array([row['Fx_Wind'], row['Fy_Wind'], row['Fz_Wind']])
        ]
        colors = ['r', 'b']
        
        for q in self.quivers: q.remove()
        self.quivers = []
        
        for o, d, c in zip(vec_origins, vec_dirs, colors):
            norm = np.linalg.norm(d)
            if norm > 0.1: 
                # Normalize length for visualization, but keep direction
                q = self.ax.quiver(o[0], o[1], o[2], 
                                   d[0], d[1], d[2], 
                                   length=norm*0.05, normalize=True, color=c)
                self.quivers.append(q)

        return self.drone_frame, self.traj_line

    def animate(self):
        ani = animation.FuncAnimation(self.fig, self.update, frames=len(self.df), interval=50, blit=False)
        plt.show()

if __name__ == "__main__":
    run_simulation()
    
    try:
        df = pd.read_csv('telemetry.csv')
        # Check for required columns
        required = ['PosX', 'PosY', 'PosZ', 'Qw', 'Qx', 'Qy', 'Qz']
        if not all(col in df.columns for col in required):
            print(f"[F] CSV Header Mismatch! Found: {list(df.columns)}")
            print("Make sure main.cpp is updating the headers correctly.")
            sys.exit(1)
            
        vis = DroneVisualizer(df)
        vis.animate()
    except Exception as e:
        print(f"[F] Error: {e}")