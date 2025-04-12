import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D plotting

# Load CSV
#df = pd.read_csv("odometry_pose.csv")
#df = pd.read_csv("odometry_pose_hover_frist.csv")
#df = pd.read_csv("odometry_pose_hover_second.csv")
df = pd.read_csv("odometry_pose_hover_third.csv")

# Normalize time (optional)
df['time_sec'] -= df['time_sec'].iloc[0]

# 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(df['x'], df['y'], df['z'], color='blue', linewidth=2)

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')

#ax.set_title('Simple Control with Velocity 100 and No Rotor Tilt - Drone Position Over Time')
#ax.set_title('hover_control with hover_altitude:=5.0, hover_gain:=12.0')
#ax.set_title('hover_control: allocation_method:="fixed_tilt", tilt_angle:="0.52')
ax.set_title('hover_control: hover_altitude:=5.0 hover_gain:=12.0 allocation_method:="fixed_tilt" tilt_angle:="0.52"')

ax.scatter(df['x'].iloc[0], df['y'].iloc[0], df['z'].iloc[0], c='green', label='Start')
ax.scatter(df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1], c='red', label='End')
ax.legend()

plt.tight_layout()
plt.show()



#plt.savefig("3d_trajectory.png", dpi=300)
