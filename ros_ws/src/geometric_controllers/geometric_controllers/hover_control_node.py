#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class HoverControlNode(Node):
    def __init__(self):
        super().__init__('hover_control_node')

        # Parameters
        self.declare_parameter('mass', 3.2)  # kg
        self.declare_parameter('gravity', 9.81)  # m/s^2


        self.mass = self.get_parameter('mass').get_parameter_value().double_value
        self.gravity = self.get_parameter('gravity').get_parameter_value().double_value

        # Subscriber

        # Publisher
        self.wrench_pub = self.create_publisher(
            Wrench, 
            '/model/variable_tilt_hexacopter/desired_wrench', 
            10
        )

        # Publish at 10 Hz
        self.timer = self.create_timer(0.01, self.publish_hover_wrench)

    def publish_hover_wrench(self):
        wrench_msg = Wrench()
        wrench_msg.force.z = 0.0
        self.wrench_pub.publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HoverControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
