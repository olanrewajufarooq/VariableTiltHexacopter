#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Wrench
from actuator_msgs.msg import Actuators
import numpy as np

from hexacopter_description.control_allocation import ControlAllocator

class ControlAllocationNode(Node):
    def __init__(self):
        super().__init__('control_allocation_node')

        # Declare parameters
        self.declare_parameter('allocation_method', 'fixed_tilt')
        self.declare_parameter('tilt_angle', 0.5235)
        self.declare_parameter('k_thrust', 8.54858e-06)
        self.declare_parameter('k_drag_to_thrust', 0.016)
        self.declare_parameter('arm_length', 0.229)
        self.declare_parameter('motor_directions', [1, -1] * 3)
        self.declare_parameter('max_motor_speed', 800.0)
        self.declare_parameter('min_motor_speed', 0.0)

        # Read parameters
        method = self.get_parameter('allocation_method').value
        tilt_angle = self.get_parameter('tilt_angle').value
        k_thrust = self.get_parameter('k_thrust').value
        k_drag_to_thrust = self.get_parameter('k_drag_to_thrust').value
        l = self.get_parameter('arm_length').value
        dirs = self.get_parameter('motor_directions').value
        max_motor = self.get_parameter('max_motor_speed').value
        min_motor = self.get_parameter('min_motor_speed').value

        # Allocator setup
        allocator_args = {
            'method': method,
            'k_thrust': k_thrust,
            'k_drag_to_thrust': k_drag_to_thrust,
            'arm_length': l,
            'motor_directions': dirs,
            'min_motor_speed': min_motor,
            'max_motor_speed': max_motor,
        }

        if method == 'fixed_tilt':
            allocator_args['tilt_angle'] = tilt_angle
        elif method == 'optimized':
            allocator_args['tilt_bounds'] = [-np.pi, np.pi]

        self.control_allocator = ControlAllocator(**allocator_args)
        self.get_logger().info(f"Control allocator initialized with method: {method}")

        # ROS interfaces
        self.wrench_sub = self.create_subscription(Wrench,
            '/model/variable_tilt_hexacopter/desired_wrench',
            self.wrench_callback, 10)

        self.motor_pub = self.create_publisher(Actuators,
            '/model/variable_tilt_hexacopter/command/motor_speed', 10)

        self.motor_plot_pub = self.create_publisher(Float64MultiArray,
            '/model/variable_tilt_hexacopter/plot/motor_speed', 10)

        self.tilt_plot_pub = self.create_publisher(Float64MultiArray,
            '/model/variable_tilt_hexacopter/plot/tilt_angle', 10)

        self.tilt_pubs = [
            self.create_publisher(Float64,
                f'/model/variable_tilt_hexacopter/joint_arm_{i+1}/cmd_pos', 10)
            for i in range(6)
        ]

    def wrench_callback(self, msg):
        wrench = np.array([
            msg.force.x, msg.force.y, msg.force.z,
            msg.torque.x, msg.torque.y, msg.torque.z
        ])

        try:
            # Allocate thrusts and tilt angles
            motor_speeds, tilt_angles = self.control_allocator.allocate(wrench)

            # ROS Publishing
            motor_msg = Actuators()
            motor_msg.velocity = motor_speeds.tolist()
            self.motor_pub.publish(motor_msg)

            for i, angle in enumerate(tilt_angles):
                tilt_msg = Float64()
                tilt_msg.data = angle
                self.tilt_pubs[i].publish(tilt_msg)

            motor_plot_msg = Float64MultiArray()
            motor_plot_msg.data = motor_speeds.tolist()
            self.motor_plot_pub.publish(motor_plot_msg)

            tilt_plot_msg = Float64MultiArray()
            tilt_plot_msg.data = tilt_angles
            self.tilt_plot_pub.publish(tilt_plot_msg)

        except Exception as e:
            self.get_logger().error(f"Allocation error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlAllocationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
