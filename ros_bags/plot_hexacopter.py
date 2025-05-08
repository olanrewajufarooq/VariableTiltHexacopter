#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

def quat_to_euler(q):
    return R.from_quat([q.x, q.y, q.z, q.w]).as_euler('zyx', degrees=False)

def adjust_times(data, t0, t_end):
    mask = [(t0 <= t <= t_end) for t in data['t']]
    out = {'t': [t - t0 for t, m in zip(data['t'], mask) if m]}
    for k in data:
        if k != 't':
            out[k] = [v for v, m in zip(data[k], mask) if m]
    return out

def parse_rosbag(bagpath, start_shift, duration):
    bagdir = Path(bagpath)
    odom = {k: [] for k in ['t','px','py','pz','roll','pitch','yaw','vx','vy','vz','wx','wy','wz']}
    desired_pose = {k: [] for k in ['t','px','py','pz','roll','pitch','yaw']}
    desired_vel = {k: [] for k in ['t','vx','vy','vz','wx','wy','wz']}
    wrench = {k: [] for k in ['t','fx','fy','fz','tx','ty','tz']}
    motors = {'t':[], **{f'm{i}':[] for i in range(6)}}
    tilts = {'t':[], **{f'a{i}':[] for i in range(6)}}

    with AnyReader([Path(bagpath)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
        for conn, t_ns, raw in reader.messages():
            topic = conn.topic
            t = t_ns / 1e9
            msg = reader.deserialize(raw, conn.msgtype)

            if topic.endswith('/odometry'):
                e = quat_to_euler(msg.pose.pose.orientation)
                odom['t'].append(t)
                odom['roll'].append(e[2])
                odom['pitch'].append(e[1])
                odom['yaw'].append(e[0])
                p = msg.pose.pose.position
                odom['px'].append(p.x)
                odom['py'].append(p.y)
                odom['pz'].append(p.z)
                v = msg.twist.twist.linear
                odom['vx'].append(v.x)
                odom['vy'].append(v.y)
                odom['vz'].append(v.z)
                w = msg.twist.twist.angular
                odom['wx'].append(w.x)
                odom['wy'].append(w.y)
                odom['wz'].append(w.z)

            elif topic.endswith('/desired_pose'):
                e = quat_to_euler(msg.pose.orientation)
                desired_pose['t'].append(t)
                desired_pose['roll'].append(e[2])
                desired_pose['pitch'].append(e[1])
                desired_pose['yaw'].append(e[0])
                desired_pose['px'].append(msg.pose.position.x)
                desired_pose['py'].append(msg.pose.position.y)
                desired_pose['pz'].append(msg.pose.position.z)

            elif topic.endswith('/desired_velocity'):
                desired_vel['t'].append(t)
                desired_vel['vx'].append(msg.twist.linear.x)
                desired_vel['vy'].append(msg.twist.linear.y)
                desired_vel['vz'].append(msg.twist.linear.z)
                desired_vel['wx'].append(msg.twist.angular.x)
                desired_vel['wy'].append(msg.twist.angular.y)
                desired_vel['wz'].append(msg.twist.angular.z)

            elif topic.endswith('/desired_wrench'):
                wrench['t'].append(t)
                wrench['fx'].append(msg.force.x)
                wrench['fy'].append(msg.force.y)
                wrench['fz'].append(msg.force.z)
                wrench['tx'].append(msg.torque.x)
                wrench['ty'].append(msg.torque.y)
                wrench['tz'].append(msg.torque.z)

            elif topic.endswith('/plot/motor_speed'):
                motors['t'].append(t)
                for i in range(6):
                    motors[f'm{i}'].append(msg.data[i])

            elif topic.endswith('/plot/tilt_angle'):
                tilts['t'].append(t)
                for i in range(6):
                    tilts[f'a{i}'].append(msg.data[i])

    firsts = [d['t'][0] for d in (odom, desired_pose, desired_vel, wrench, motors, tilts) if d['t']]
    t0 = min(firsts) + start_shift
    t_end = t0 + duration

    odom = adjust_times(odom, t0, t_end)
    desired_pose = adjust_times(desired_pose, t0, t_end)
    desired_vel = adjust_times(desired_vel, t0, t_end)
    wrench = adjust_times(wrench, t0, t_end)
    motors = adjust_times(motors, t0, t_end)
    tilts = adjust_times(tilts, t0, t_end)

    t = {'odom': odom['t'], 'desired_pose': desired_pose['t'], 'desired_vel': desired_vel['t'], 
         'wrench': wrench['t'], 'motors': motors['t'], 'tilts': tilts['t']}
    actual_data = {
        'pos': [odom['px'], odom['py'], odom['pz']],
        'ori': [odom['roll'], odom['pitch'], odom['yaw']],
        'lin_vel': [odom['vx'], odom['vy'], odom['vz']],
        'ang_vel': [odom['wx'], odom['wy'], odom['wz']],
        'wrench_f': [wrench['fx'], wrench['fy'], wrench['fz']],
        'wrench_t': [wrench['tx'], wrench['ty'], wrench['tz']],
        'motors': [motors['m0'], motors['m1'], motors['m2']],
        'tilts': [np.rad2deg(tilts['a0']), np.rad2deg(tilts['a1']), np.rad2deg(tilts['a2'])],
    }
    desired_data = {
        'pos': [desired_pose['px'], desired_pose['py'], desired_pose['pz']],
        'ori': [desired_pose['roll'], desired_pose['pitch'], desired_pose['yaw']],
        'lin_vel': [desired_vel['vx'], desired_vel['vy'], desired_vel['vz']],
        'ang_vel': [desired_vel['wx'], desired_vel['wy'], desired_vel['wz']],
        'wrench_f': [wrench['fx'], wrench['fy'], wrench['fz']],
        'wrench_t': [wrench['tx'], wrench['ty'], wrench['tz']],
    }
    return t, actual_data, desired_data

def plot_all(t, data, desired_data, plotdir, duration):
    plotdir = Path(plotdir)
    plotdir.mkdir(parents=True, exist_ok=True)

    colors = ['r', 'g', 'b']
    lw = 1.5
    fs = 14

    def plot_xyz(ax, t1, actual, t2, desired, ylabel, label_names):
        for i, color in enumerate(colors):
            ax.plot(t1, actual[i], color=color, linewidth=lw, label=f'{label_names[i]} (actual)')
            ax.plot(t2, desired[i], color=color, linestyle='--', linewidth=lw, label=f'{label_names[i]} (desired)')
        ax.set_ylabel(ylabel, fontsize=fs)
        ax.set_xlabel("Time [s]", fontsize=fs)
        ax.set_xlim(0, duration)
        all_values = np.concatenate(actual + desired)
        y_min, y_max = np.min(all_values), np.max(all_values)
        margin = 0.1 * (y_max - y_min) if y_max != y_min else 0.1 * y_max
        ax.set_ylim(y_min - margin, y_max + margin)
        ax.legend(loc='best', fontsize=fs - 2)

    def plot_single(ax, t1, signal, ylabel, label_names):
        for i, color in enumerate(colors):
            ax.plot(t1, signal[i], color=color, linestyle='-', linewidth=lw, label=f'{label_names[i]} (commanded)')
        ax.set_ylabel(ylabel, fontsize=fs)
        ax.set_xlabel("Time [s]", fontsize=fs)
        ax.set_xlim(0, duration)
        all_values = np.concatenate(signal)
        y_min, y_max = np.min(all_values), np.max(all_values)
        margin = 0.1 * (y_max - y_min) if y_max != y_min else 0.1 * y_max
        ax.set_ylim(y_min - margin, y_max + margin)
        ax.legend(loc='best', fontsize=fs - 2)

    # 1. Position and Orientation
    fig, ax = plt.subplots(figsize=(10, 4))
    plot_xyz(ax, t['odom'], data['pos'], t['desired_pose'], desired_data['pos'], "Position [m]", ['x', 'y', 'z'])
    fig.tight_layout()
    fig.savefig(plotdir / 'pos_only.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    plot_xyz(ax, t['odom'], data['ori'], t['desired_pose'], desired_data['ori'], "Orientation [rad]", ['roll', 'pitch', 'yaw'])
    fig.tight_layout()
    fig.savefig(plotdir / 'ori_only.png')
    plt.close(fig)

    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    plot_xyz(axs[0], t['odom'], data['pos'], t['desired_pose'], desired_data['pos'], "Position [m]", ['x', 'y', 'z'])
    plot_xyz(axs[1], t['odom'], data['ori'], t['desired_pose'], desired_data['ori'], "Orientation [rad]", ['roll', 'pitch', 'yaw'])
    fig.tight_layout()
    fig.savefig(plotdir / 'pos_ori_subplots.png')
    plt.close(fig)

    # 2. Linear and Angular Velocities
    fig, ax = plt.subplots(figsize=(10, 4))
    plot_xyz(ax, t['odom'], data['lin_vel'], t['desired_vel'], desired_data['lin_vel'], "Linear Vel. [m/s]", [r'$v_x$', r'$v_y$', r'$v_z$'])
    fig.tight_layout()
    fig.savefig(plotdir / 'linear_vel_only.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    plot_xyz(ax, t['odom'], data['ang_vel'], t['desired_vel'], desired_data['ang_vel'], "Angular Vel. [rad/s]", [r'$\omega_x$', r'$\omega_y$', r'$\omega_z$'])
    fig.tight_layout()
    fig.savefig(plotdir / 'angular_vel_only.png')
    plt.close(fig)

    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    plot_xyz(axs[0], t['odom'], data['lin_vel'], t['desired_vel'], desired_data['lin_vel'], "Linear Vel. [m/s]", [r'$v_x$', r'$v_y$', r'$v_z$'])
    plot_xyz(axs[1], t['odom'], data['ang_vel'], t['desired_vel'], desired_data['ang_vel'], "Angular Vel. [rad/s]", [r'$\omega_x$', r'$\omega_y$', r'$\omega_z$'])
    fig.tight_layout()
    fig.savefig(plotdir / 'velocities_subplots.png')
    plt.close(fig)

    # 3. Forces and Torques — only commanded values
    fig, ax = plt.subplots(figsize=(10, 4))
    plot_single(ax, t['wrench'], desired_data['wrench_f'], "Force [N]", [r'$F_x$', r'$F_y$', r'$F_z$'])
    fig.tight_layout()
    fig.savefig(plotdir / 'forces_only.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    plot_single(ax, t['wrench'], desired_data['wrench_t'], "Torque [Nm]", [r'$\tau_x$', r'$\tau_y$', r'$\tau_z$'])
    fig.tight_layout()
    fig.savefig(plotdir / 'torques_only.png')
    plt.close(fig)

    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    plot_single(axs[0], t['wrench'], desired_data['wrench_f'], "Force [N]", [r'$F_x$', r'$F_y$', r'$F_z$'])
    plot_single(axs[1], t['wrench'], desired_data['wrench_t'], "Torque [Nm]", [r'$\tau_x$', r'$\tau_y$', r'$\tau_z$'])
    fig.tight_layout()
    fig.savefig(plotdir / 'wrenches_subplots.png')
    plt.close(fig)

    # 4. Motor Speeds and Tilt Angles
    fig, ax = plt.subplots(figsize=(10, 4))
    for i, color in enumerate(colors):
        ax.plot(t['motors'], data['motors'][i], color=color, linewidth=lw, label=f'Motor {i}')
    ax.set_ylabel("Motor Speed [rad/s]", fontsize=fs)
    ax.set_xlabel("Time [s]", fontsize=fs)
    ax.set_xlim(0, duration)
    all_vals = np.concatenate(data['motors'])
    y_min, y_max = np.min(all_vals), np.max(all_vals)
    margin = 0.1 * (y_max - y_min) if y_max != y_min else 0.1 * y_max
    ax.set_ylim(y_min - margin, y_max + margin)
    ax.legend(loc='best', fontsize=fs - 2)
    fig.tight_layout()
    fig.savefig(plotdir / 'motor_speeds_only.png')
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    for i, color in enumerate(colors):
        ax.plot(t['tilts'], data['tilts'][i], color=color, linewidth=lw, label=f'Tilt {i}')
    ax.set_ylabel("Tilt Angle [deg]", fontsize=fs)
    ax.set_xlabel("Time [s]", fontsize=fs)
    ax.set_xlim(0, duration)
    all_vals = np.concatenate(data['tilts'])
    y_min, y_max = np.min(all_vals), np.max(all_vals)
    margin = 0.1 * (y_max - y_min) if y_max != y_min else 0.1 * y_max
    ax.set_ylim(y_min - margin, y_max + margin)
    ax.legend(loc='best', fontsize=fs - 2)
    fig.tight_layout()
    fig.savefig(plotdir / 'tilt_angles_only.png')
    plt.close(fig)

    fig, ax1 = plt.subplots(figsize=(10, 4))
    ax2 = ax1.twinx()
    lines1, labels1, lines2, labels2 = [], [], [], []
    for i, color in enumerate(colors):
        l1, = ax1.plot(t['motors'], data['motors'][i], color=color, linewidth=lw, label=f'Motor {i}')
        l2, = ax2.plot(t['tilts'], data['tilts'][i], color=color, linestyle='--', linewidth=lw, label=f'Tilt {i}')
        lines1.append(l1); labels1.append(f'Motor {i}')
        lines2.append(l2); labels2.append(f'Tilt {i}')
    ax1.set_ylabel("Motor Speed [rad/s]", fontsize=fs)
    ax2.set_ylabel("Tilt Angle [deg]", fontsize=fs)
    ax1.set_xlabel("Time [s]", fontsize=fs)
    ax1.set_xlim(0, duration)
    ax2.set_xlim(0, duration)
    all_vals1 = np.concatenate(data['motors'])
    all_vals2 = np.concatenate(data['tilts'])
    margin1 = 0.1 * (np.max(all_vals1) - np.min(all_vals1))
    margin2 = 0.1 * (np.max(all_vals2) - np.min(all_vals2))
    ax1.set_ylim(np.min(all_vals1) - margin1, np.max(all_vals1) + margin1)
    ax2.set_ylim(np.min(all_vals2) - margin2, np.max(all_vals2) + margin2)
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='best', fontsize=fs - 2)
    fig.tight_layout()
    fig.savefig(plotdir / 'motor_and_tilt_combined.png')
    plt.close(fig)

    # 5. 2D Path (x-y trajectory)
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(data['pos'][0], data['pos'][1], color='blue', linewidth=lw, label='Actual Path')
    ax.plot(desired_data['pos'][0], desired_data['pos'][1], color='blue', linestyle='--', linewidth=lw, label='Desired Path')
    ax.set_xlabel("x [m]", fontsize=fs)
    ax.set_ylabel("y [m]", fontsize=fs)
    ax.set_aspect('equal', adjustable='box')
    ax.legend(loc='best', fontsize=fs - 2)
    fig.tight_layout()
    fig.savefig(plotdir / 'xy_path.png')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bagpath', '-b', required=True)
    parser.add_argument('--plotpath', '-o', default='plots')
    parser.add_argument('--start-shift', '-s', type=float, default=2.5)
    parser.add_argument('--duration', '-d', type=float, default=60.0)
    args = parser.parse_args()

    t, actual_data, desired_data = parse_rosbag(args.bagpath, args.start_shift, args.duration)
    plot_all(t, actual_data, desired_data, args.plotpath, args.duration)

if __name__ == '__main__':
    main()
