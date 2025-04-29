#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

def quat_to_euler(q):
    """Convert quaternion to roll, pitch, yaw (ZYX order)."""
    return R.from_quat([q.x, q.y, q.z, q.w]).as_euler('zyx', degrees=False)

def adjust_times(data, t0, t_end):
    """Crop data to [t0, t_end] and shift time so that t0 -> 0."""
    mask = [(t0 <= t <= t_end) for t in data['t']]
    out = {'t': [t - t0 for t, m in zip(data['t'], mask) if m]}
    for k in data:
        if k != 't':
            out[k] = [v for v, m in zip(data[k], mask) if m]
    return out

def main(bagpath, plotpath, start_shift, duration):
    bagdir = Path(bagpath)
    plotdir = Path(plotpath)
    plotdir.mkdir(parents=True, exist_ok=True)

    # Allocate buffers
    odom = {'t':[], 'px':[], 'py':[], 'pz':[], 'roll':[], 'pitch':[], 'yaw':[],
            'vx':[], 'vy':[], 'vz':[], 'wx':[], 'wy':[], 'wz':[]}
    wrench = {'t':[], 'fx':[], 'fy':[], 'fz':[], 'tx':[], 'ty':[], 'tz':[]}
    motors = {'t':[], 'm0':[], 'm1':[], 'm2':[], 'm3':[], 'm4':[], 'm5':[]}
    tilts = {'t':[], 'a0':[], 'a1':[], 'a2':[], 'a3':[], 'a4':[], 'a5':[]}

    with AnyReader([bagdir], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as reader:
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

    # Time adjustment
    firsts = [d['t'][0] for d in (odom, wrench, motors, tilts) if d['t']]
    t0 = min(firsts) + start_shift
    t_end = t0 + duration

    odom = adjust_times(odom, t0, t_end)
    wrench = adjust_times(wrench, t0, t_end)
    motors = adjust_times(motors, t0, t_end)
    tilts = adjust_times(tilts, t0, t_end)

    # =============================================
    # 1. Position and Orientation (3x2)
    fig, axs = plt.subplots(3,2, figsize=(10,10), sharex=True)
    axs = axs.flatten()
    axs[0].plot(odom['t'], odom['px']); axs[0].set_ylabel('px [m]')
    axs[1].plot(odom['t'], odom['py']); axs[1].set_ylabel('py [m]')
    axs[2].plot(odom['t'], odom['pz']); axs[2].set_ylabel('pz [m]')
    axs[3].plot(odom['t'], np.rad2deg(odom['roll'])); axs[3].set_ylabel('roll [deg]')
    axs[4].plot(odom['t'], np.rad2deg(odom['pitch'])); axs[4].set_ylabel('pitch [deg]')
    axs[5].plot(odom['t'], np.rad2deg(odom['yaw'])); axs[5].set_ylabel('yaw [deg]')
    for ax in axs:
        ax.grid()
    axs[-1].set_xlabel('Time [s]')
    fig.suptitle('Position and Orientation')
    fig.tight_layout()
    fig.savefig(plotdir / 'position_orientation.png')
    plt.close(fig)

    # =============================================
    # 2. Linear and Angular Velocities (3x2)
    fig, axs = plt.subplots(3,2, figsize=(10,10), sharex=True)
    axs = axs.flatten()
    axs[0].plot(odom['t'], odom['vx']); axs[0].set_ylabel('vx [m/s]')
    axs[1].plot(odom['t'], odom['vy']); axs[1].set_ylabel('vy [m/s]')
    axs[2].plot(odom['t'], odom['vz']); axs[2].set_ylabel('vz [m/s]')
    axs[3].plot(odom['t'], odom['wx']); axs[3].set_ylabel('wx [rad/s]')
    axs[4].plot(odom['t'], odom['wy']); axs[4].set_ylabel('wy [rad/s]')
    axs[5].plot(odom['t'], odom['wz']); axs[5].set_ylabel('wz [rad/s]')
    for ax in axs:
        ax.grid()
    axs[-1].set_xlabel('Time [s]')
    fig.suptitle('Linear and Angular Velocities')
    fig.tight_layout()
    fig.savefig(plotdir / 'velocities.png')
    plt.close(fig)

    # =============================================
    # 3. Wrench Forces and Torques (3x2)
    fig, axs = plt.subplots(3,2, figsize=(10,10), sharex=True)
    axs = axs.flatten()
    axs[0].plot(wrench['t'], wrench['fx']); axs[0].set_ylabel('Fx [N]')
    axs[1].plot(wrench['t'], wrench['fy']); axs[1].set_ylabel('Fy [N]')
    axs[2].plot(wrench['t'], wrench['fz']); axs[2].set_ylabel('Fz [N]')
    axs[3].plot(wrench['t'], wrench['tx']); axs[3].set_ylabel('τx [Nm]')
    axs[4].plot(wrench['t'], wrench['ty']); axs[4].set_ylabel('τy [Nm]')
    axs[5].plot(wrench['t'], wrench['tz']); axs[5].set_ylabel('τz [Nm]')
    for ax in axs:
        ax.grid()
    axs[-1].set_xlabel('Time [s]')
    fig.suptitle('Desired Wrench')
    fig.tight_layout()
    fig.savefig(plotdir / 'desired_wrench.png')
    plt.close(fig)

    # =============================================
    # 4. Motor Speeds and Tilt Angles (3x4)
    fig, axs = plt.subplots(3,4, figsize=(14,10), sharex=True)
    axs = axs.flatten()
    for i in range(6):
        axs[i].plot(motors['t'], motors[f'm{i}'])
        axs[i].set_ylabel(f'Motor {i} [rad/s]')
    for i in range(6):
        axs[i+6].plot(tilts['t'], np.rad2deg(tilts[f'a{i}']))
        axs[i+6].set_ylabel(f'Tilt {i} [deg]')
    for ax in axs:
        ax.grid()
    axs[-1].set_xlabel('Time [s]')
    fig.suptitle('Motor Speeds and Tilt Angles')
    fig.tight_layout()
    fig.savefig(plotdir / 'motor_tilt.png')
    plt.close(fig)

    # =============================================
    # 5. General Overview (5x6) — as explained earlier
    fig, axs = plt.subplots(5,6, figsize=(24,16), sharex=True)
    axs = axs.flatten()

    # Fill exactly like we discussed before...
    keys_labels = [
        ('px','px [m]'), ('py','py [m]'), ('pz','pz [m]'),
        ('roll','roll [deg]'), ('pitch','pitch [deg]'), ('yaw','yaw [deg]'),
        ('vx','vx [m/s]'), ('vy','vy [m/s]'), ('vz','vz [m/s]'),
        ('wx','wx [rad/s]'), ('wy','wy [rad/s]'), ('wz','wz [rad/s]'),
        ('fx','Fx [N]'), ('fy','Fy [N]'), ('fz','Fz [N]'),
        ('tx','τx [Nm]'), ('ty','τy [Nm]'), ('tz','τz [Nm]'),
        ('m0','Motor0'), ('m1','Motor1'), ('m2','Motor2'),
        ('m3','Motor3'), ('m4','Motor4'), ('m5','Motor5'),
        ('a0','Tilt0'), ('a1','Tilt1'), ('a2','Tilt2'),
        ('a3','Tilt3'), ('a4','Tilt4'), ('a5','Tilt5')
    ]

    for ax, (key, label) in zip(axs, keys_labels):
        if key in odom:
            ax.plot(odom['t'], np.rad2deg(odom[key]) if key in ['roll','pitch','yaw'] else odom[key])
        elif key in wrench:
            ax.plot(wrench['t'], wrench[key])
        elif key.startswith('m'):
            ax.plot(motors['t'], motors[key])
        elif key.startswith('a'):
            ax.plot(tilts['t'], np.rad2deg(tilts[key]))
        ax.set_ylabel(label)
        ax.grid()

    axs[-1].set_xlabel('Time [s]')
    fig.suptitle('Full State Overview', fontsize=18)
    fig.tight_layout()
    fig.subplots_adjust(top=0.95)
    fig.savefig(plotdir / 'full_state_overview.png')
    plt.close(fig)

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--bagpath','-b', default='geometric_control_bag/')
    p.add_argument('--plotpath','-o', default='default_plots')
    p.add_argument('--start-shift','-s', type=float, default=2.5)
    p.add_argument('--duration','-d', type=float, default=60.0)
    args = p.parse_args()
    main(args.bagpath, args.plotpath, args.start_shift, args.duration)
