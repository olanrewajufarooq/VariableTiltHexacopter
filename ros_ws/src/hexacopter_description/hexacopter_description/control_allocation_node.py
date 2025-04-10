#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from actuator_msgs.msg import Actuators
from std_msgs.msg import Float64
import numpy as np
from scipy.optimize import lsq_linear

class ControlAllocationNode(Node):
    def __init__(self):
        super().__init__('control_allocation_node')

        # Control Allocation Parameters
        self.declare_parameter('allocation_method') # 'fixed_tilt', 'variable_tilt'
        self.declare_parameter('tilt_angle')  # radians

        self.allocation_method = self.get_parameter('allocation_method').get_parameter_value().string_value
        self.tilt_angle = self.get_parameter('tilt_angle').get_parameter_value().double_value

        self.tilt_default_angle = 0.0 # radians, defaults to this value if allocation matrix is rank deficient

        # Motor Parameters

        self.declare_parameter('k_thrust')  # N/(rad/s)^2
        self.declare_parameter('k_drag_to_thrust')  # N*m/(rad/s)^2
        self.declare_parameter('arm_length')  # m
        self.declare_parameter('motor_directions') 

        self.declare_parameter('max_speed')  # rad/s
        self.declare_parameter('min_speed')  # rad/s
        self.declare_parameter('max_tilt_angle')  # radians
        self.declare_parameter('min_tilt_angle')  # radians

        self.k_thrust = self.get_parameter('k_thrust').get_parameter_value().double_value
        self.k_drag_to_thrust = self.get_parameter('k_drag_to_thrust').get_parameter_value().double_value
        self.arm_length = self.get_parameter('arm_length').get_parameter_value().double_value
        self.motor_directions = self.get_parameter('motor_directions').get_parameter_value().double_array_value

        self.min_motor_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.max_motor_speed = self.get_parameter('max_speed').get_parameter_value().double_value
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

    def compute_allocation_matrix(self, tilt_angles):
        l = self.arm_length
        gamma = self.k_drag_to_thrust

        alpha = np.deg2rad(tilt_angles)
        
	M = np.zeros((6, 6))
	    
	for i in range(6):
		psi = i * (2 * np.pi / 6)
		sigma_i = (-1) ** (i + 1)
		alpha_i = alpha * (-1) ** (i + 1)
		
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
		
		# Column M_i = [γσ_i u_i + ξ_i ∧ u_i; u_i]
		M_i_top = gamma * sigma_i * u_i + xi_cross_u_i
		M_i = np.concatenate([M_i_top, u_i])
		
		M[:, i] = M_i
	    
	return M
        
     raise NotImplementedError("Allocation matrix computation is not implemented yet.")

    def wrench_callback(self, msg: Wrench):
        # Desired wrench: [Fz, Tx, Ty, Tz]
        desired_wrench = np.array([msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z])

        try:
            if self.allocation_method.lower() == 'fixed_tilt':
                tilt_angles = [self.tilt_angle] * 6
                thrusts = self.fixed_tilt_allocation(desired_wrench)
            elif self.allocation_method.lower() == 'variable_tilt':
                [thrusts, tilt_angles] = self.variable_tilt_allocation(desired_wrench)
            else:
                tilt_angles = [self.tilt_angle] * 6
                thrusts = self.fixed_tilt_allocation(desired_wrench)

            # Compute motor speeds
            motor_speeds = self.thrust_to_motor_speeds(thrusts)

            # Publish motor speeds
            motor_msg = Actuators()
            motor_msg.angular_velocities = motor_speeds.tolist()
            self.motor_pub.publish(motor_msg)
            self.get_logger().info(f"[{self.allocation_method.lower()}] Motor speeds: {np.round(motor_speeds, 1)}")

            # Publish tilt angles
            for i, angle in enumerate(tilt_angles):
                tilt_msg = Float64()
                tilt_msg.data = angle
                self.tilt_pubs[i].publish(tilt_msg)
            self.get_logger().info(f"Published tilt angles: {tilt_angles}")

        except Exception as e:
            self.get_logger().error(f"Control allocation error: {str(e)}")

    def thrust_to_motor_speeds(self, thrusts):
        # Convert thrusts to motor speeds
        motor_speeds = np.zeros(6)
        for i in range(6):
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
        tilt_angles = [self.tilt_angle] * 6
        A = self.compute_allocation_matrix(tilt_angles)

        # Check rank of the allocation matrix
        if np.linalg.matrix_rank(A) < 6:
            self.get_logger().error(f"Allocation matrix is rank deficient, cannot compute motor speeds. Defaulting to {self.tilt_default_angle} radians.")
            self.tilt_angle = self.tilt_default_angle
            
            tilt_angles = [self.tilt_angle] * 6
            A = self.compute_allocation_matrix(tilt_angles)

        self.inv_A = np.linalg.inv(A)

    def fixed_tilt_allocation(self, desired_wrench):
        thrusts = self.inv_A @ desired_wrench
        return thrusts

    def init_variable_tilt_allocation(self):
        raise NotImplementedError("Variable tilt allocation is not implemented yet.")
    
    def variable_tilt_allocation(self, desired_wrench):
        raise NotImplementedError("Variable tilt allocation is not implemented yet.")
        # return thrusts, tilt_angles

def main(args=None):
    rclpy.init(args=args)
    node = ControlAllocationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
