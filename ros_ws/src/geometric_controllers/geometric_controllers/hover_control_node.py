#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry

# Import the Controller class from the controller module
from geometric_controllers.controller import PDController, FeedbackLinearizedController, AdaptiveController

class HoverControlNode(Node):
    def __init__(self):
        super().__init__('hover_control_node')

        # Parameters
        self.declare_parameter('mass', 3.2)  # kg
        self.declare_parameter('I', [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Inertia matrix
        self.declare_parameter('gravity', 9.81)  # m/s^2
        self.declare_parameter('hover_altitude', 5.0) # m

        self.declare_parameter('Kp_pos', [1.0, 1.0, 1.0])
        self.declare_parameter('Kp_att', [1.0, 1.0, 1.0])
        self.declare_parameter('Kd', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        self.declare_parameter('controller_type', 'PD')  # Options: 'PD', 'FeedLin' or 'Adaptive'

        self.mass = self.get_parameter('mass').get_parameter_value().double_value
        self.I = self.get_parameter('I').get_parameter_value().double_array_value
        self.gravity = self.get_parameter('gravity').get_parameter_value().double_value
        self.hover_altitude = self.get_parameter('hover_altitude').get_parameter_value().double_value

        self.Kp_pos = self.get_parameter('Kp_pos').get_parameter_value().double_array_value
        self.Kp_att = self.get_parameter('Kp_att').get_parameter_value().double_array_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_array_value

        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value

        # Defining Desired Pose in SE(3) and Desired Twist in R6
        self.H_des = np.eye(4)
        self.H_des[2, 3] = self.hover_altitude  # Set desired altitude
        self.V_des = np.zeros((6, 1))  # Desired twist is zero

        # Initialize current pose and twist
        self.H = np.eye(4)
        self.V = np.zeros((6, 1))

        # Initialize Controller

        self.get_logger().info(f"Using {self.controller_type} controller")

        if self.controller_type == 'PD':
            self.controller = PDController(self.Kp_att, self.Kp_pos, self.Kd)
        elif self.controller_type == 'FeedLin':
            self.controller = FeedbackLinearizedController(self.Kp_att, self.Kp_pos, self.Kd, self.I)
        elif self.controller_type == 'Adaptive':
            self.controller = AdaptiveController(self.Kp_att, self.Kp_pos, self.Kd, self.I)
        else:
            self.get_logger().error(f"Unknown controller type: {self.controller_type}")
            self.get_logger().warn("Defaulting to PD controller")
            self.controller = PDController(self.Kp_att, self.Kp_pos, self.Kd)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/variable_tilt_hexacopter/odometry',
            self.odom_callback,
            10
        )

        # Publisher
        self.wrench_pub = self.create_publisher(
            Wrench, 
            '/model/variable_tilt_hexacopter/desired_wrench', 
            10
        )

        # Publish at 100 Hz
        self.timer = self.create_timer(0.001, self.publish_hover_wrench)

    def odom_callback(self, msg):
        # Store the current odometry information
        pos_data = msg.pose.pose.position
        att_data = msg.pose.pose.orientation
        vel_data = msg.twist.twist.linear
        omega_data = msg.twist.twist.angular

        # Convert Orientation to Rotation Matrix (Numpy)
        quat = np.array([att_data.x, att_data.y, att_data.z, att_data.w])
        att = R.from_quat(quat).as_matrix()

        # Convert Position to Numpy Array
        pos = np.array([pos_data.x, pos_data.y, pos_data.z]).reshape(3, 1)

        # Convert Twist to Numpy Array
        vel = np.array([vel_data.x, vel_data.y, vel_data.z])
        omega = np.array([omega_data.x, omega_data.y, omega_data.z])
        self.V = np.concatenate((vel, omega)).reshape(6, 1)

        # SE(3) Represetation of the Pose
        self.H = np.eye(4)
        self.H[:3, :3] = att
        self.H[:3, 3] = pos.reshape(3,)
    
    def publish_hover_wrench(self):
        # Compute the desired wrench
        W = self.controller.compute_wrench(
            self.mass,
            self.gravity,
            self.H_des,
            self.H,
            self.V_des,
            self.V
        ).flatten()

        self.get_logger().debug(f"Computed Wrench: {W}")

        # Create a Wrench message
        wrench_msg = Wrench()

        wrench_msg.torque.x = W[0]
        wrench_msg.torque.y = W[1]
        wrench_msg.torque.z = W[2]
        wrench_msg.force.x = W[3]
        wrench_msg.force.y = W[4]
        wrench_msg.force.z = W[5]
        
        # Publish the wrench message
        self.wrench_pub.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HoverControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
