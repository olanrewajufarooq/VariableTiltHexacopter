#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry

from geometric_controllers.controller import Controller

class HoverControlNode(Node):
    def __init__(self):
        super().__init__('hover_control_node')

        # Declare Parameters
        self.declare_parameter('mass', 3.2)
        self.declare_parameter('I', [0.1]*6)
        self.declare_parameter('CoG', [0, 0, 0])
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('hover_altitude', 5.0)

        self.declare_parameter('Kp_pos', [1.0, 1.0, 1.0])
        self.declare_parameter('Kp_att', [1.0, 1.0, 1.0])
        self.declare_parameter('Kd', [1.0]*6)

        self.declare_parameter('controller_type', 'PD')

        # Read parameters
        self.mass = self.get_parameter('mass').value
        self.I = self.get_parameter('I').value
        self.gravity = self.get_parameter('gravity').value
        self.hover_altitude = self.get_parameter('hover_altitude').value

        self.Kp_pos = self.get_parameter('Kp_pos').value
        self.Kp_att = self.get_parameter('Kp_att').value
        self.Kd = self.get_parameter('Kd').value
        self.controller_type = self.get_parameter('controller_type').value

        # Desired State
        self.H_des = np.eye(4)
        self.H_des[2, 3] = self.hover_altitude
        self.V_des = np.zeros((6,1))

        self.H = np.eye(4)
        self.V = np.zeros((6,1))

        # Controller
        self.get_logger().info(f"Controller: {self.controller_type}")
        self.controller = Controller(
            method=self.controller_type,
            Kp_att=self.Kp_att,
            Kp_pos=self.Kp_pos,
            Kd=self.Kd,
            CoG=[0, 0, 0],
            I=self.I
        )

        # Subscribers and Publishers
        self.odom_sub = self.create_subscription(
            Odometry, '/model/variable_tilt_hexacopter/odometry', self.odom_callback, 10)

        self.wrench_pub = self.create_publisher(
            Wrench, '/model/variable_tilt_hexacopter/desired_wrench', 10)

        self.timer = self.create_timer(0.001, self.publish_hover_wrench)  # 1000 Hz

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        omega = msg.twist.twist.angular

        quat = np.array([ori.x, ori.y, ori.z, ori.w])
        R_mat = R.from_quat(quat).as_matrix()

        self.H = np.eye(4)
        self.H[:3,:3] = R_mat
        self.H[:3,3] = np.array([pos.x, pos.y, pos.z])

        vel_vec = np.array([vel.x, vel.y, vel.z])
        omega_vec = np.array([omega.x, omega.y, omega.z])

        self.V = np.vstack((vel_vec.reshape(3,1), omega_vec.reshape(3,1)))

    def publish_hover_wrench(self):
        W = self.controller.compute_wrench(
            self.mass, self.gravity, self.H_des, self.H, self.V_des, self.V).flatten()

        wrench_msg = Wrench()
        wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z = W[0], W[1], W[2]
        wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z = W[3], W[4], W[5]

        self.wrench_pub.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HoverControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
