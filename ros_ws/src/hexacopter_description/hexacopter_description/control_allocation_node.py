#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from actuator_msgs.msg import Actuators
from std_msgs.msg import Float64
import numpy as np

class ControlAllocationNode(Node):
    def __init__(self):
        super().__init__('control_allocation_node')

        # Control Allocation Parameters
        self.declare_parameter('allocation_method', 'fixed_tilt') # 'fixed_tilt', 'variable_tilt'
        self.declare_parameter('tilt_angle', 0.5235)  # radians

        self.allocation_method = self.get_parameter('allocation_method').get_parameter_value().string_value
        self.tilt_angle = self.get_parameter('tilt_angle').get_parameter_value().double_value

        self.tilt_default_angle = 0.5235 # radians, defaults to this value if allocation matrix is rank deficient

        # Motor Parameters

        self.declare_parameter('k_thrust', 8.54858e-06)  # N/(rad/s)^2
        self.declare_parameter('k_drag_to_thrust', 0.016)  # N*m/(rad/s)^2
        self.declare_parameter('arm_length', 0.229)  # m
        self.declare_parameter('motor_directions', [1, -1] * 3) 

        self.declare_parameter('max_motor_speed', 800.0)  # rad/s
        self.declare_parameter('min_motor_speed', 0.00)  # rad/s
        self.declare_parameter('max_tilt_angle', 1.57)  # radians
        self.declare_parameter('min_tilt_angle', -1.57)  # radians

        self.k_thrust = self.get_parameter('k_thrust').get_parameter_value().double_value
        self.k_drag_to_thrust = self.get_parameter('k_drag_to_thrust').get_parameter_value().double_value
        self.arm_length = self.get_parameter('arm_length').get_parameter_value().double_value
        self.motor_directions = self.get_parameter('motor_directions').get_parameter_value().integer_array_value

        self.min_motor_speed = self.get_parameter('min_motor_speed').get_parameter_value().double_value
        self.max_motor_speed = self.get_parameter('max_motor_speed').get_parameter_value().double_value
        self.min_tilt_angle = self.get_parameter('min_tilt_angle').get_parameter_value().double_value
        self.max_tilt_angle = self.get_parameter('max_tilt_angle').get_parameter_value().double_value

        # Initialize fixed tilt allocation
        if self.allocation_method == 'fixed_tilt':
            self.init_fixed_tilt_allocation()
        elif self.allocation_method == 'variable_tilt':
            self.init_variable_tilt_allocation()
        else:
            self.get_logger().warn(f"Unknown method '{self.allocation_method}', falling back to fixed_tilt.")
            self.init_fixed_tilt_allocation()

        # Publisher and Subscriber
        self.wrench_sub = self.create_subscription(Wrench, '/model/variable_tilt_hexacopter/desired_wrench', self.wrench_callback, 10)
        self.motor_pub = self.create_publisher(Actuators, '/model/variable_tilt_hexacopter/command/motor_speed', 10)
        self.tilt_pubs = [
            self.create_publisher(Float64, f'/model/variable_tilt_hexacopter/joint_arm_{i+1}/cmd_pos', 10)
            for i in range(6)
        ]

    def wrench_callback(self, msg: Wrench):
        # Desired wrench: [Fz, Tx, Ty, Tz]
        desired_wrench = np.array([msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z])

        try:
            if self.allocation_method.lower() == 'fixed_tilt':
                tilt_angles = [self.tilt_angle, -self.tilt_angle] * 3
                thrusts = self.fixed_tilt_allocation(desired_wrench)
            elif self.allocation_method.lower() == 'variable_tilt':
                [thrusts, tilt_angles] = self.variable_tilt_allocation(desired_wrench)
            else:
                tilt_angles = [self.tilt_angle, -self.tilt_angle] * 3
                thrusts = self.fixed_tilt_allocation(desired_wrench)

            # Compute motor speeds
            motor_speeds = self.thrust_to_motor_speeds(thrusts)

            # Publish motor speeds
            motor_msg = Actuators()
            motor_msg.velocity = motor_speeds.tolist()
            self.motor_pub.publish(motor_msg)

            # Publish tilt angles
            for i, angle in enumerate(tilt_angles):
                tilt_msg = Float64()
                tilt_msg.data = angle
                self.tilt_pubs[i].publish(tilt_msg)
            # self.get_logger().info(f"Published tilt angles: {tilt_angles}")

        except Exception as e:
            self.get_logger().error(f"Control allocation error: {str(e)}")

    def compute_allocation_matrix(self, tilt_angles):
        l = self.arm_length
        gamma = self.k_drag_to_thrust
        sigmas = list(self.motor_directions)
        alpha = tilt_angles
        
        A = np.zeros((6, 6))
            
        for i in range(6):
            psi = i * (np.pi / 3)
            sigma_i = sigmas[i]
            alpha_i = alpha[i]
            
            # Orientation vector u_i
            u_i = np.array([
                np.sin(psi) * np.sin(alpha_i),
                -np.cos(psi) * np.sin(alpha_i),
                np.cos(alpha_i)
            ])
            
            # Displacement vector xi_i
            xi_i = np.array([l * np.cos(psi), l * np.sin(psi), 0])
            
            # Cross product ξ_i ∧ u_i
            xi_cross_u_i = np.cross(xi_i, u_i)
            
            # Column W = [γσ_i f + ξ_i ∧ f; f]
            T = gamma * sigma_i * u_i + xi_cross_u_i
            W = np.concatenate([u_i, T])
            
            A[:, i] = W
            
        return A
    
    def compute_static_allocation_matrix(self):
        l = self.arm_length
        gamma = self.k_drag_to_thrust
        sigmas = list(self.motor_directions)
        
        A_static = np.zeros((6, 12))
            
        for i in range(6):
            psi = i * (np.pi / 3)
            sigma_i = sigmas[i]
            
            # Vertical force column (cos component)
            A_static[2, 2*i] = 1  # Force in Z direction
            A_static[3, 2*i] = gamma * sigma_i  # Torque around X
            A_static[4, 2*i] = l * np.sin(psi)  # Torque around Y
            A_static[5, 2*i] = -l * np.cos(psi)  # Torque around Z

            # Lateral force column (sin component)
            A_static[0, 2*i + 1] = np.sin(psi)  # Force in X direction
            A_static[1, 2*i + 1] = -np.cos(psi)  # Force in Y direction
            # No torque around X from lateral thrust
            A_static[4, 2*i + 1] = l * np.cos(psi)  # Torque around Y
            A_static[5, 2*i + 1] = l * np.sin(psi)  # Torque around Z
            
        return A_static

    def thrust_to_motor_speeds(self, thrusts):
        # Convert thrusts to motor speeds
        motor_speeds = np.zeros(6)
        for i in range(6):
            if thrusts[i] < 0:
                # self.get_logger().error(f"Negative thrust for motor {i+1}: {thrusts[i]}. Setting to 0.")
                motor_speeds[i] = 0
            else:
                motor_speeds[i] = np.sqrt(thrusts[i] / self.k_thrust)
        
        # Clip motor speeds to the limits
        motor_speeds = np.clip(motor_speeds, self.min_motor_speed, self.max_motor_speed)

        return motor_speeds

    def init_fixed_tilt_allocation(self):
        """
        Initialize fixed tilt allocation method.
        This method computes the allocation matrix based on the fixed tilt angle ONCE.
        """

        # Initialize fixed tilt allocation
        tilt_angles = [self.tilt_angle, -self.tilt_angle] * 3
        A = self.compute_allocation_matrix(tilt_angles)

        # Check rank of the allocation matrix
        if np.abs(np.linalg.det(A)) < 1e-6:
            self.get_logger().error(f"Allocation matrix is rank deficient, Det= {np.linalg.det(A)}. Cannot compute motor speeds for tilt angles: {tilt_angles}. Defaulting to {self.tilt_default_angle} radians.")
            self.tilt_angle = self.tilt_default_angle
            
            tilt_angles = [self.tilt_angle, -self.tilt_angle] * 3
            A = self.compute_allocation_matrix(tilt_angles)

        self.inv_A = np.linalg.inv(A)

    def fixed_tilt_allocation(self, desired_wrench):
        thrusts = self.inv_A @ desired_wrench
        return thrusts

    def init_variable_tilt_allocation(self):
        A = self.compute_static_allocation_matrix()
        # assert np.abs(np.linalg.det(A.T @ A)) > 1e-8, f"Matrix is illconditioned. Det: {np.linalg.det(A.T @ A)}"

        self.A_inv = np.linalg.pinv(A)

        
    def variable_tilt_allocation(self, desired_wrench):
        thrust_comps = self.A_inv @ desired_wrench

        thrusts = np.zeros(6)
        tilt_angles = np.zeros(6)

        for i in range(6):
            Fv = thrust_comps[2*i]      # Vertical force component
            Fl = thrust_comps[2*i + 1]  # Lateral force component

            thrusts[i] = np.sqrt(Fv**2 + Fl**2)  # Rotor speed
            tilt_angles[i] = np.arctan2(Fl, Fv)  # Tilt angle
        
        return thrusts, tilt_angles

def main(args=None):
    rclpy.init(args=args)
    node = ControlAllocationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
