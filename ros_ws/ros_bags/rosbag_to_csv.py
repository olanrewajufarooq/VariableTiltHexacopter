import numpy as np
import matplotlib.pyplot as plt

from rosbags.highlevel import AnyReader
from pathlib import Path
import csv

## Simple
# bag_path = Path('rosbag2_2025_04_12-23_03_58_simple_100')
# output_csv = 'odometry_pose.csv'

## Hover Frist
# bag_path = Path('hover_frist')
# output_csv = 'odometry_pose_hover_frist.csv'

# ## Hover Second
# bag_path = Path('hover_second')
# output_csv = 'odometry_pose_hover_second.csv'

## Hover Third
bag_path = Path('hover_third')
output_csv = 'odometry_pose_hover_third.csv'

rows = []

with AnyReader([bag_path]) as reader:
    connections = [c for c in reader.connections if c.topic == '/model/variable_tilt_hexacopter/odometry']

    for conn, timestamp, rawdata in reader.messages(connections=connections):
        data = reader.deserialize(rawdata, conn.msgtype)
        t = timestamp * 1e-9
        pos = data.pose.pose.position
        rows.append([t, pos.x, pos.y, pos.z])


# Write to CSV
with open(output_csv, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['time_sec', 'x', 'y', 'z'])
    writer.writerows(rows)

print(f"✅ Saved pose data to {output_csv} with {len(rows)} entries.")

# # Lists for storing data
# timestamps = []
# x_vals, y_vals, z_vals = [], [], []

# with AnyReader([bag_path]) as reader:
#     reader.open()
#     connections = [c for c in reader.connections if c.topic == '/model/variable_tilt_hexacopter/pose']
    
#     for conn, msg in reader.messages(connections=connections):
#         # Use builtin type support for parsing
#         data = reader.deserialize(msg, conn.msgtype)
#         t = msg[3].nanoseconds * 1e-9  # timestamp
#         pos = data.pose.position
#         timestamps.append(t)
#         x_vals.append(pos.x)
#         y_vals.append(pos.y)
#         z_vals.append(pos.z)

# # Shift time to start from 0
# timestamps = np.array(timestamps)
# timestamps -= timestamps[0]

# # Plot
# plt.figure()
# plt.plot(timestamps, x_vals, label='X')
# plt.plot(timestamps, y_vals, label='Y')
# plt.plot(timestamps, z_vals, label='Z')
# plt.xlabel("Time [s]")
# plt.ylabel("Position [m]")
# plt.title("Drone Position Over Time")
# plt.legend()
# plt.grid()
# plt.tight_layout()
# plt.show()
