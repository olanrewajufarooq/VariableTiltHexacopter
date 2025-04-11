#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry

class HoverControlNode(Node):
    def __init__(self):
        super().__init__('hover_control_node')

        # Parameters
        self.declare_parameter('mass', 3.2)  # kg
        self.declare_parameter('gravity', 9.81)  # m/s^2
        self.declare_parameter('hover_altitude', 10.0) # m
        self.declare_parameter('hover_gain', 1.0)  # Kp gain for hover error correction


        self.mass = self.get_parameter('mass').get_parameter_value().double_value
        self.gravity = self.get_parameter('gravity').get_parameter_value().double_value
        self.hover_altitude = self.get_parameter('hover_altitude').get_parameter_value().double_value
        self.hover_gain = self.get_parameter('hover_gain').get_parameter_value().double_value

        # Subscriber
        self.current_altitude_sub = self.create_subscription(
            Odometry,
            '/model/variable_tilt_hexacopter/odometry',
            self.altitude_callback,
            10
        )
        self.current_altitude = 0.0

        # Publisher
        self.wrench_pub = self.create_publisher(
            Wrench, 
            '/model/variable_tilt_hexacopter/desired_wrench', 
            10
        )

        # Publish at 10 Hz
        self.timer = self.create_timer(0.01, self.publish_hover_wrench)

    def altitude_callback(self, msg):
        self.current_altitude = msg.pose.pose.position.z
        self.get_logger().info(f"Current altitude: {self.current_altitude:.3f} m. Desired: {self.hover_altitude:.3f} m.")

    def publish_hover_wrench(self):
        wrench_msg = Wrench()

        hover_thrust = self.mass * self.gravity
        correction_thrust = self.hover_gain * (self.hover_altitude - self.current_altitude)
        
        thrust = hover_thrust + correction_thrust

        # Hover = force to counteract gravity
        wrench_msg.force.x = 0.0
        wrench_msg.force.y = 0.0
        wrench_msg.force.z = thrust

        # No torque for perfect hover
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = 0.0

        self.wrench_pub.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HoverControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
