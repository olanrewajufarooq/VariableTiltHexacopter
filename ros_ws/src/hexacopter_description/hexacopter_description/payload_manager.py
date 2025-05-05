#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Empty

class PayloadManager(Node):
    def __init__(self):
        super().__init__('payload_manager')

        # slider command pubs
        self.left_pub = self.create_publisher(
            Float64,
            '/model/variable_tilt_hexacopter/joint_slider_left/cmd_pos',
            10
        )
        self.right_pub = self.create_publisher(
            Float64,
            '/model/variable_tilt_hexacopter/joint_slider_right/cmd_pos',
            10
        )

        # attach/detach command pubs
        self.attach_pub = self.create_publisher(
            Empty,
            '/model/variable_tilt_hexacopter/payload/attach',
            10
        )
        self.detach_pub = self.create_publisher(
            Empty,
            '/model/variable_tilt_hexacopter/payload/detach',
            10
        )

        # subscriber flipping pick/drop
        self.create_subscription(
            Bool,
            '/model/variable_tilt_hexacopter/payload/pick',
            self.pick_callback,
            10
        )

        # simple open/close positions
        self.pick_position = 0.053
        self.drop_position = 0.0

        # start dropped
        self.move_sliders(self.drop_position)
        self.get_logger().info('PayloadManager ready.')

    def pick_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('PICK → closing sliders & attaching')
            self.move_sliders(self.pick_position)
            self.attach_pub.publish(Empty())
        else:
            self.get_logger().info('DROP → opening sliders & detaching')
            self.move_sliders(self.drop_position)
            self.detach_pub.publish(Empty())

    def move_sliders(self, position: float):
        m = Float64(data=position)
        self.left_pub .publish(m)
        self.right_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = PayloadManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
