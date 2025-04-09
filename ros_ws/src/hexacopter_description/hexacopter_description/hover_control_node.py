#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class HoverControlNode(Node):
    def __init__(self):
        super().__init__('hover_control_node')

        # Parameters
        self.declare_parameter('mass')  # kg
        self.declare_parameter('gravity')  # m/s^2

        self.mass = self.get_parameter('mass').get_parameter_value().double_value
        self.gravity = self.get_parameter('gravity').get_parameter_value().double_value

        # Publisher
        self.wrench_pub = self.create_publisher(Wrench, '/model/variable_tilt_hexacopter/desired_wrench', 10)

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_hover_wrench)

    def publish_hover_wrench(self):
        wrench_msg = Wrench()

        # Hover = force to counteract gravity
        thrust = self.mass * self.gravity
        wrench_msg.force.x = 0.0
        wrench_msg.force.y = 0.0
        wrench_msg.force.z = thrust

        # No torque for perfect hover
        wrench_msg.torque.x = 0.0
        wrench_msg.torque.y = 0.0
        wrench_msg.torque.z = 0.0

        self.wrench_pub.publish(wrench_msg)
        self.get_logger().info(f"Published hover wrench: Fz={thrust:.2f} N")

def main(args=None):
    rclpy.init(args=args)
    node = HoverControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
