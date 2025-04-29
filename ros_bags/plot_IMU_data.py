import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.spatial.transform import Rotation as R
from rosbags.highlevel import AnyReader
from rosbags.serde import deserialize_cdr
import os

def quaternion_to_euler_zyx(x, y, z, w):
    """Convert quaternion to Euler angles (ZYX convention) in radians."""
    rot = R.from_quat([x, y, z, w])
    return rot.as_euler('zyx', degrees=False)

def main(start_shift=0.5, duration=10.0):
    bag_path = Path('imu_odom_bag')
    save_dir = Path('Plots_videos/IMU_data')
    save_dir.mkdir(parents=True, exist_ok=True)

    imu_data = {
        'time': [], 'roll': [], 'pitch': [], 'yaw': [],
        'ang_vel_x': [], 'ang_vel_y': [], 'ang_vel_z': [],
        'lin_acc_x': [], 'lin_acc_y': [], 'lin_acc_z': [],
    }
    odom_data = {
        'time': [], 'roll': [], 'pitch': [], 'yaw': [],
        'ang_vel_x': [], 'ang_vel_y': [], 'ang_vel_z': [],
        'lin_vel_x': [], 'lin_vel_y': [], 'lin_vel_z': [],
        'pos_x': [], 'pos_y': [], 'pos_z': [],
    }


    with AnyReader([bag_path]) as reader:
        for connection, timestamp, rawdata in reader.messages():
            topic = connection.topic
            time = timestamp / 1e9

            if topic == '/model/variable_tilt_hexacopter/imu':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                q = msg.orientation
                euler = quaternion_to_euler_zyx(q.x, q.y, q.z, q.w)

                imu_data['time'].append(time)
                imu_data['yaw'].append(euler[0])
                imu_data['pitch'].append(euler[1])
                imu_data['roll'].append(euler[2])
                imu_data['ang_vel_x'].append(msg.angular_velocity.x)
                imu_data['ang_vel_y'].append(msg.angular_velocity.y)
                imu_data['ang_vel_z'].append(msg.angular_velocity.z)
                imu_data['lin_acc_x'].append(msg.linear_acceleration.x)
                imu_data['lin_acc_y'].append(msg.linear_acceleration.y)
                imu_data['lin_acc_z'].append(msg.linear_acceleration.z)

            elif topic == '/model/variable_tilt_hexacopter/odometry':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                q = msg.pose.pose.orientation
                euler = quaternion_to_euler_zyx(q.x, q.y, q.z, q.w)

                odom_data['time'].append(time)
                odom_data['yaw'].append(euler[0])
                odom_data['pitch'].append(euler[1])
                odom_data['roll'].append(euler[2])

                tw = msg.twist.twist
                odom_data['ang_vel_x'].append(tw.angular.x)
                odom_data['ang_vel_y'].append(tw.angular.y)
                odom_data['ang_vel_z'].append(tw.angular.z)
                odom_data['lin_vel_x'].append(tw.linear.x)
                odom_data['lin_vel_y'].append(tw.linear.y)
                odom_data['lin_vel_z'].append(tw.linear.z)

                pos = msg.pose.pose.position
                odom_data['pos_x'].append(pos.x)
                odom_data['pos_y'].append(pos.y)
                odom_data['pos_z'].append(pos.z)


    # Shift and crop times
    t0 = min(imu_data['time'][0], odom_data['time'][0]) + start_shift
    t_end = t0 + duration

    def adjust(data_dict):
        adjusted = {}
        for k, v in data_dict.items():
            if k == 'time':
                adjusted[k] = [t - t0 for t in v if t0 <= t <= t_end]
            else:
                adjusted[k] = [val for t, val in zip(data_dict['time'], v) if t0 <= t <= t_end]
        return adjusted

    imu_data = adjust(imu_data)
    odom_data = adjust(odom_data)

    # ---------- Plot Orientation ----------
    plt.figure(figsize=(12, 8))
    plt.subplot(3, 1, 1)
    plt.plot(imu_data['time'], np.rad2deg(imu_data['roll']), label='IMU Roll')
    plt.plot(odom_data['time'], np.rad2deg(odom_data['roll']), label='Odom Roll')
    plt.ylabel('Roll (deg)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(imu_data['time'], np.rad2deg(imu_data['pitch']), label='IMU Pitch')
    plt.plot(odom_data['time'], np.rad2deg(odom_data['pitch']), label='Odom Pitch')
    plt.ylabel('Pitch (deg)')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(imu_data['time'], np.rad2deg(imu_data['yaw']), label='IMU Yaw')
    plt.plot(odom_data['time'], np.rad2deg(odom_data['yaw']), label='Odom Yaw')
    plt.ylabel('Yaw (deg)')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_dir / 'orientation.png')
    plt.close()

    # ---------- Plot Angular Velocity ----------
    plt.figure(figsize=(12, 6))
    plt.subplot(3, 1, 1)
    plt.plot(imu_data['time'], imu_data['ang_vel_x'], label='IMU ωₓ')
    plt.plot(odom_data['time'], odom_data['ang_vel_x'], label='Odom ωₓ')
    plt.ylabel('rad/s')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(imu_data['time'], imu_data['ang_vel_y'], label='IMU ωᵧ')
    plt.plot(odom_data['time'], odom_data['ang_vel_y'], label='Odom ωᵧ')
    plt.ylabel('rad/s')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(imu_data['time'], imu_data['ang_vel_z'], label='IMU ω_z')
    plt.plot(odom_data['time'], odom_data['ang_vel_z'], label='Odom ω_z')
    plt.ylabel('rad/s')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_dir / 'compare_angular_velocity.png')
    plt.close()

    # ---------- Plot Linear Velocity ----------
    plt.figure(figsize=(12, 6))
    plt.subplot(3, 1, 1)
    plt.plot(odom_data['time'], odom_data['lin_vel_x'], label='Linear Velocity X')
    plt.ylabel('m/s')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(odom_data['time'], odom_data['lin_vel_y'], label='Linear Velocity Y')
    plt.ylabel('m/s')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(odom_data['time'], odom_data['lin_vel_z'], label='Linear Velocity Z')
    plt.ylabel('m/s')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_dir / 'linear_velocity.png')
    plt.close()


    # ---------- Plot Linear Acceleration ----------
    plt.figure(figsize=(12, 6))
    plt.subplot(3, 1, 1)
    plt.plot(imu_data['time'], imu_data['lin_acc_x'], label='Linear Acceleration X')
    plt.ylabel('m/s²')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(imu_data['time'], imu_data['lin_acc_y'], label='Linear Acceleration Y')
    plt.ylabel('m/s²')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(imu_data['time'], imu_data['lin_acc_z'], label='Linear Acceleration Z')
    plt.ylabel('m/s²')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_dir / 'linear_acceleration.png')
    plt.close()

    # ---------- Plot Ground Truth Position ----------
    plt.figure(figsize=(12, 6))
    plt.subplot(3, 1, 1)
    plt.plot(odom_data['time'], odom_data['pos_x'], label='Position X')
    plt.ylabel('m')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(odom_data['time'], odom_data['pos_y'], label='Position Y')
    plt.ylabel('m')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(odom_data['time'], odom_data['pos_z'], label='Position Z')
    plt.ylabel('m')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_dir / 'position.png')
    plt.close()

    # ---------- Plot 3D Trajectory ----------
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the 3D trajectory without modifying the Z-values
    ax.plot(odom_data['pos_x'], odom_data['pos_y'], odom_data['pos_z'],
            linewidth=2, color='blue')

    # Set the axis labels and title
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectory')
    ax.grid(True)

    # Adjust the Z-axis limits to ensure it starts from zero (without shifting data)
    ax.set_zlim(bottom=0)

    # Set the same range for X and Y axes
    min_xy = min(min(odom_data['pos_x']), min(odom_data['pos_y']))
    max_xy = max(max(odom_data['pos_x']), max(odom_data['pos_y']))

    ax.set_xlim([min_xy, max_xy])
    ax.set_ylim([min_xy, max_xy])

    # Adjust the plot layout and save the figure
    plt.tight_layout()
    plt.savefig(save_dir / '3d_trajectory.png', dpi=300)
    plt.close()

if __name__ == '__main__':
    main(duration=10.0)
