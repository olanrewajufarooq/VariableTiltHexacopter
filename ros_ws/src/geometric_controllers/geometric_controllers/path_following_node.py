#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Wrench, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

from geometric_controllers.controller import Controller
from geometric_controllers.path_generator import PathGenerator
from geometric_controllers.utils import Ad_inv

class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')

        # ───── Parameters ─────
        self.declare_parameter('mass', 3.2)
        self.declare_parameter('gravity', 9.81)

        self.declare_parameter('Kp_pos', [1.0, 1.0, 1.0])
        self.declare_parameter('Kp_att', [1.0, 1.0, 1.0])
        self.declare_parameter('Kd', [1.0]*6)

        self.declare_parameter('controller_type', 'PD')  # PD, FeedLin, Adaptive
        self.declare_parameter('potential_type', 'liealgebra')  # liealgebra, separate
        self.declare_parameter('I', [0.1]*6)  # Inertia matrix if needed
        self.declare_parameter('CoG', [0.0, 0.0, 0.0])  # Center of Gravity

        self.declare_parameter('path', 'circle')
        self.declare_parameter('path_scale', 5.0)
        self.declare_parameter('path_period', 0.5)
        self.declare_parameter('path_altitude', 5.0)
        self.declare_parameter('path_start_with_hover', True)

        # ───── Load parameters ─────
        self.mass = self.get_parameter('mass').value
        self.gravity = self.get_parameter('gravity').value

        self.Kp_pos = self.get_parameter('Kp_pos').value
        self.Kp_att = self.get_parameter('Kp_att').value
        self.Kd = self.get_parameter('Kd').value

        self.controller_type = self.get_parameter('controller_type').value
        self.potential_type = self.get_parameter('potential_type').value
        self.I = self.get_parameter('I').value
        self.CoG = self.get_parameter('CoG').value

        self.path_name = self.get_parameter('path').value
        self.path_scale = self.get_parameter('path_scale').value
        self.path_period = self.get_parameter('path_period').value
        self.path_altitude = self.get_parameter('path_altitude').value
        self.path_start_with_hover = self.get_parameter('path_start_with_hover').value

        # ───── Initialize state ─────
        self.H = np.eye(4)
        self.V = np.zeros((6, 1))
        self.H_des = np.eye(4)
        self.V_des = np.zeros((6, 1))
        self.odometry_received = False

        # ───── Initialize Controller and Path Generator ─────
        self.get_logger().info(f"Controller: {self.controller_type} | Potential type: {self.potential_type}")

        self.previous_time = None

        self.controller = Controller(
            method=self.controller_type,
            Kp_att=self.Kp_att,
            Kp_pos=self.Kp_pos,
            Kd=self.Kd,
            m=self.mass,
            I=self.I,
            CoG=self.CoG,
            pot_type=self.potential_type,
        )

        self.path_generator = PathGenerator(
            path_name=self.path_name,
            scale=self.path_scale,
            period=self.path_period,
            altitude=self.path_altitude,
            start_with_hover=self.path_start_with_hover,
        )

        self.get_logger().info(
            f"PathGenerator: {self.path_name} | scale={self.path_scale} | period={self.path_period} | altitude={self.path_altitude}"
        )

        # ───── Subscribers and Publishers ─────
        cb_group = MutuallyExclusiveCallbackGroup()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/variable_tilt_hexacopter/odometry',
            self.odom_callback,
            10,
            callback_group=cb_group
        )

        self.wrench_pub = self.create_publisher(
            Wrench, '/model/variable_tilt_hexacopter/desired_wrench', 10)

        self.desired_pose_pub = self.create_publisher(
            PoseStamped, '/model/variable_tilt_hexacopter/desired_pose', 10)

        self.desired_velocity_pub = self.create_publisher(
            TwistStamped, '/model/variable_tilt_hexacopter/desired_velocity', 10)

        self.timer = self.create_timer(
            0.01, self.publish_hover_wrench, callback_group=cb_group)  # 100 Hz

    # ───── Odometry Callback ─────
    def odom_callback(self, msg):
        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            vel = msg.twist.twist.linear
            omega = msg.twist.twist.angular

            quat = np.array([ori.x, ori.y, ori.z, ori.w])
            R_mat = R.from_quat(quat).as_matrix()
            p_vec = np.array([pos.x, pos.y, pos.z]).reshape(3,1)

            omega_vec = np.array([omega.x, omega.y, omega.z]).reshape(3,1)
            v_vec = np.array([vel.x, vel.y, vel.z]).reshape(3,1)

            self.H = np.eye(4)
            self.H[:3,:3] = R_mat
            self.H[:3,3] = p_vec.flatten()

            self.V = np.vstack((omega_vec, v_vec))
            self.odometry_received = True

        except Exception as e:
            self.get_logger().warn(f"Odometry parsing error: {e}")

    # ───── Main Control Loop ─────
    def publish_hover_wrench(self):
        if not self.odometry_received:
            return

        try:
            now = self.get_clock().now()
            current_time = now.nanoseconds * 1e-9

            if self.previous_time is None:
                self.previous_time = current_time

            # Update desired path
            self.H_des, self.V_des, self.A_des = self.path_generator.generate(current_time)
            self.publish_desired_pose(now)
            self.publish_desired_velocity(now)

            # Compute wrench
            if self.controller_type == 'Adaptive':
                wrench = self.controller.compute_wrench(
                    H_des=self.H_des, H=self.H,
                    V_des=self.V_des, V=self.V,
                    A_des=self.A_des, dt = current_time - self.previous_time,
                ).flatten()

                # est_mass = self.controller.estimated_unknown_mass
                # self.get_logger().info(f"Estimated Mass: {est_mass}")

                self.previous_time = current_time
            else:
                wrench = self.controller.compute_wrench(
                    H_des=self.H_des, H=self.H,
                    V_des=self.V_des, V=self.V,
                    A_des=self.A_des,
                ).flatten()

            wrench_msg = Wrench()
            wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z = wrench[3:]
            wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z = wrench[0:3]

            self.wrench_pub.publish(wrench_msg)

        except Exception as e:
            self.get_logger().warn(f"Control computation error: {e}")

    # ───── Publish Desired Pose ─────
    def publish_desired_pose(self, now):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'world'

        pose_msg.pose.position.x = float(self.H_des[0,3])
        pose_msg.pose.position.y = float(self.H_des[1,3])
        pose_msg.pose.position.z = float(self.H_des[2,3])

        rot = R.from_matrix(self.H_des[:3,:3])
        quat = rot.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        self.desired_pose_pub.publish(pose_msg)

    # ───── Publish Desired Velocity ─────
    def publish_desired_velocity(self, now):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now.to_msg()
        twist_msg.header.frame_id = 'world'

        V_flat = self.V_des.flatten()
        twist_msg.twist.angular.x = float(V_flat[0])
        twist_msg.twist.angular.y = float(V_flat[1])
        twist_msg.twist.angular.z = float(V_flat[2])

        twist_msg.twist.linear.x = float(V_flat[3])
        twist_msg.twist.linear.y = float(V_flat[4])
        twist_msg.twist.linear.z = float(V_flat[5])

        self.desired_velocity_pub.publish(twist_msg)

# ───── Main ─────
def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
