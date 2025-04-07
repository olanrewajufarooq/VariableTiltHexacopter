#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ros_gz_interfaces.msg import Float32Array

class SimpleControlNode(Node):
    def __init__(self):
        super().__init__('simple_control_node')

        # Declare and get parameters
        self.declare_parameter('angles', [0.0] * 6)
        self.declare_parameter('motor_speeds', [400.0] * 6)

        self.angles = self.get_parameter('angles').get_parameter_value().double_array_value
        self.motor_speeds = self.get_parameter('motor_speeds').get_parameter_value().double_array_value

        if len(self.angles) != 6 or len(self.motor_speeds) != 6:
            self.get_logger().error("Expected 6 elements for both 'angles' and 'motor_speeds'. Shutting down.")
            rclpy.shutdown()
            return

        # Setup publishers
        self.joint_names = [f"joint_arm_{i+1}" for i in range(6)]
        self.joint_publishers = [
            self.create_publisher(Float64, f"/model/variable_tilt_hexacopter/{j}/cmd_pos", 10)
            for j in self.joint_names
        ]

        self.motor_pub = self.create_publisher(Float32Array, "/model/variable_tilt_hexacopter/command/motor_speed", 10)

        # Timer to send at 10 Hz
        self.timer = self.create_timer(0.1, self.send_control_signals)

    def send_control_signals(self):
        # Publish joint angles
        for i, pub in enumerate(self.joint_publishers):
            msg = Float64()
            msg.data = self.angles[i]
            pub.publish(msg)

        # Publish motor speeds
        motor_msg = Float32Array()
        motor_msg.data = list(self.motor_speeds)
        self.motor_pub.publish(motor_msg)

        self.get_logger().info(f"Published angles: {list(self.angles)}, motor_speeds: {list(self.motor_speeds)}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
