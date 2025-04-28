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

class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')

        # Parameters
        self.declare_parameter('mass', 3.2)
        self.declare_parameter('gravity', 9.81)

        self.declare_parameter('Kxi', [1.0, 1.0, 1.0])
        self.declare_parameter('Gp', [1.0, 1.0, 1.0])
        self.declare_parameter('Kd', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        self.declare_parameter('path', 'circle')
        self.declare_parameter('path_scale', 5.0)
        self.declare_parameter('path_period', 0.5)
        self.declare_parameter('path_altitude', 5.0)

        # Load parameter values
        self.mass = self.get_parameter('mass').get_parameter_value().double_value
        self.gravity = self.get_parameter('gravity').get_parameter_value().double_value

        self.Kxi = np.array(self.get_parameter('Kxi').get_parameter_value().double_array_value)
        self.Gp = np.array(self.get_parameter('Gp').get_parameter_value().double_array_value)
        self.Kd = np.array(self.get_parameter('Kd').get_parameter_value().double_array_value)

        self.path_name = self.get_parameter('path').get_parameter_value().string_value
        self.path_scale = self.get_parameter('path_scale').get_parameter_value().double_value
        self.path_period = self.get_parameter('path_period').get_parameter_value().double_value
        self.path_altitude = self.get_parameter('path_altitude').get_parameter_value().double_value

        # Initialize state
        self.H = np.eye(4)
        self.V = np.zeros((6, 1))
        self.H_des = np.eye(4)
        self.V_des = np.zeros((6, 1))
        self.odometry_received = False

        # Controller and path generator
        self.controller = Controller(self.Gp, self.Kxi, self.Kd)

        self.path_generator = PathGenerator(
            path_name=self.path_name,
            scale=self.path_scale,
            period=self.path_period,
            altitude=self.path_altitude
        )
        self.get_logger().info(f"Path generator initialized with path: {self.path_name}, scale: {self.path_scale}, period: {self.path_period}, altitude: {self.path_altitude}")


        # Thread-safe callback groups
        cb_group = MutuallyExclusiveCallbackGroup()

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/variable_tilt_hexacopter/odometry',
            self.odom_callback,
            10,
            callback_group=cb_group
        )

        # Publisher
        self.wrench_pub = self.create_publisher(Wrench, '/model/variable_tilt_hexacopter/desired_wrench', 10)
        self.desired_pose_pub = self.create_publisher(PoseStamped, '/model/variable_tilt_hexacopter/desired_pose', 10)
        self.desired_velocity_pub = self.create_publisher(TwistStamped, '/model/variable_tilt_hexacopter/desired_velocity', 10)

        # Timer (100 Hz)
        self.timer = self.create_timer(0.001, self.publish_hover_wrench, callback_group=cb_group)

    def odom_callback(self, msg):
        try:
            # Extract and transform pose
            pos_data = msg.pose.pose.position
            att_data = msg.pose.pose.orientation
            vel_data = msg.twist.twist.linear
            omega_data = msg.twist.twist.angular

            quat = np.array([att_data.x, att_data.y, att_data.z, att_data.w])
            R_mat = R.from_quat(quat).as_matrix()
            pos = np.array([pos_data.x, pos_data.y, pos_data.z]).reshape(3, 1)
            vel = np.array([vel_data.x, vel_data.y, vel_data.z])
            omega = np.array([omega_data.x, omega_data.y, omega_data.z])

            self.H = np.eye(4)
            self.H[:3, :3] = R_mat
            self.H[:3, 3] = pos.flatten()
            self.V = np.concatenate((vel, omega)).reshape(6, 1)

            self.odometry_received = True
        except Exception as e:
            self.get_logger().warn(f"Odometry processing error: {e}")

    def publish_hover_wrench(self):
        if not self.odometry_received:
            return

        try:
            # Use high precision time
            now = self.get_clock().now()
            current_time = now.nanoseconds * 1e-9

            # Get desired state
            self.H_des, self.V_des = self.path_generator.generate(current_time)
            self.publish_desired_pose(now)
            self.publish_desired_velocity(now)

            # Compute wrench
            wrench = self.controller.compute_wrench(
                self.mass, self.gravity,
                self.H_des, self.H,
                self.V_des, self.V
            ).flatten()

            # Log debug
            self.get_logger().debug(f"Computed wrench: {wrench}")

            # Publish wrench
            wrench_msg = Wrench()
            wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z = wrench[3:6]
            wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z = wrench[0:3]
            self.wrench_pub.publish(wrench_msg)

        except Exception as e:
            self.get_logger().warn(f"Control error during wrench computation: {e}")

    def publish_desired_pose(self, now):
        """Publish desired pose as PoseStamped message"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'world'
        
        # Position
        pose_msg.pose.position.x = float(self.H_des[0, 3])
        pose_msg.pose.position.y = float(self.H_des[1, 3])
        pose_msg.pose.position.z = float(self.H_des[2, 3])
        
        # Orientation
        rot = R.from_matrix(self.H_des[:3, :3])
        quat = rot.as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        
        self.desired_pose_pub.publish(pose_msg)

    def publish_desired_velocity(self, now):
        """Publish desired velocity as TwistStamped message"""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now.to_msg()
        twist_msg.header.frame_id = 'world'
        
        # Flatten and convert velocity vector
        V_des_flat = self.V_des.flatten()
        
        # Angular velocity (first 3 elements)
        twist_msg.twist.angular.x = float(V_des_flat[0])
        twist_msg.twist.angular.y = float(V_des_flat[1])
        twist_msg.twist.angular.z = float(V_des_flat[2])
        
        # Linear velocity (last 3 elements)
        twist_msg.twist.linear.x = float(V_des_flat[3])
        twist_msg.twist.linear.y = float(V_des_flat[4])
        twist_msg.twist.linear.z = float(V_des_flat[5])
        
        self.desired_velocity_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
