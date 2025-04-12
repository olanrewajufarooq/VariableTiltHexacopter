import pandas as pd
import matplotlib.pyplot as plt

# Load your CSV
## Simple 100
#df = pd.read_csv('odometry_pose.csv')

# ## Hover Frist
# df = pd.read_csv('odometry_pose_hover_frist.csv')

# ## Hover Second
# df = pd.read_csv('odometry_pose_hover_second.csv')

## Hover Third
df = pd.read_csv('odometry_pose_hover_third.csv')

# Normalize time to start from 0
df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]

# Plot
plt.figure()
plt.plot(df['time_sec'], df['x'], label='X')
plt.plot(df['time_sec'], df['y'], label='Y')
plt.plot(df['time_sec'], df['z'], label='Z')

# Labels and formatting
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
#plt.title('Simple Control with Velocity 100 and No Rotor Tilt - Drone Position Over Time')
#plt.title('hover_control with hover_altitude:=5.0, hover_gain:=12.0')
#plt.title('hover_control: allocation_method:="fixed_tilt", tilt_angle:="0.52"')
plt.title('hover_control: hover_altitude:=5.0 hover_gain:=12.0 allocation_method:="fixed_tilt" tilt_angle:="0.52"')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()