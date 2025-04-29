import numpy as np
from abc import ABC, abstractmethod

# ─────────────────────────────────────────────
# Base abstract class
# ─────────────────────────────────────────────
class BaseAllocator(ABC):
    def __init__(self, k_thrust, k_drag_to_thrust, arm_length, motor_directions,
                 min_motor_speed=0.0, max_motor_speed=800.0):
        self.k_thrust = k_thrust
        self.k_drag_to_thrust = k_drag_to_thrust
        self.arm_length = arm_length
        self.motor_directions = list(motor_directions)
        self.min_motor_speed = min_motor_speed
        self.max_motor_speed = max_motor_speed

    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def allocate(self, desired_wrench):
        pass

    def thrust_to_motor_speeds(self, thrusts):
        speeds = np.array([
            np.sqrt(max(thrust, 0) / self.k_thrust)
            for thrust in thrusts
        ])
        return np.clip(speeds, self.min_motor_speed, self.max_motor_speed)

# ─────────────────────────────────────────────
# Fixed Tilt Allocator
# ─────────────────────────────────────────────
class FixedTiltAllocator(BaseAllocator):
    def __init__(self, *args, tilt_angle=0.5235, tilt_default_angle=0.5235, **kwargs):
        super().__init__(*args, **kwargs)
        self.tilt_angle = tilt_angle
        self.tilt_default_angle = tilt_default_angle
        self.inv_A = None
        self.tilt_angles = None
        self.initialize()  # ✅ automatic inside subclass

    def initialize(self):
        self.tilt_angles = [self.tilt_angle, -self.tilt_angle] * 3
        A = self._compute_allocation_matrix(self.tilt_angles)

        if np.abs(np.linalg.det(A)) < 1e-6:
            self.tilt_angles = [self.tilt_default_angle, -self.tilt_default_angle] * 3
            A = self._compute_allocation_matrix(self.tilt_angles)

        self.inv_A = np.linalg.inv(A)

    def allocate(self, desired_wrench):
        thrusts = self.inv_A @ desired_wrench
        return thrusts, self.tilt_angles

    def _compute_allocation_matrix(self, tilt_angles):
        l = self.arm_length
        gamma = self.k_drag_to_thrust
        sigmas = self.motor_directions
        A = np.zeros((6, 6))

        for i in range(6):
            psi = i * (np.pi / 3)
            sigma = sigmas[i]
            alpha = tilt_angles[i]

            u_i = np.array([
                np.sin(psi) * np.sin(alpha),
                -np.cos(psi) * np.sin(alpha),
                np.cos(alpha)
            ])

            xi_i = np.array([l * np.cos(psi), l * np.sin(psi), 0])
            cross = np.cross(xi_i, u_i)
            T = gamma * sigma * u_i + cross
            A[:, i] = np.concatenate([u_i, T])

        return A

# ─────────────────────────────────────────────
# Variable Tilt Allocator
# ─────────────────────────────────────────────
class VariableTiltAllocator(BaseAllocator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.A_inv = None
        self.initialize()  # ✅ automatic inside subclass

    def initialize(self):
        A = self._compute_static_allocation_matrix()
        self.A_inv = np.linalg.pinv(A)

    def allocate(self, desired_wrench):
        thrust_comps = self.A_inv @ desired_wrench
        thrusts = np.zeros(6)
        tilt_angles = np.zeros(6)

        for i in range(6):
            Fv = thrust_comps[2*i]
            Fl = thrust_comps[2*i + 1]
            thrusts[i] = np.sqrt(Fv**2 + Fl**2)
            tilt_angles[i] = np.arctan2(Fl, Fv)

        return thrusts, tilt_angles

    def _compute_static_allocation_matrix(self):
        l = self.arm_length
        gamma = self.k_drag_to_thrust
        sigmas = self.motor_directions
        A = np.zeros((6, 12))

        for i in range(6):
            psi = i * (np.pi / 3)
            sigma = sigmas[i]

            A[2, 2*i] = 1
            A[3, 2*i] = gamma * sigma
            A[4, 2*i] = l * np.sin(psi)
            A[5, 2*i] = -l * np.cos(psi)

            A[0, 2*i + 1] = np.sin(psi)
            A[1, 2*i + 1] = -np.cos(psi)
            A[4, 2*i + 1] = l * np.cos(psi)
            A[5, 2*i + 1] = l * np.sin(psi)

        return A

# ─────────────────────────────────────────────
# Control Allocator (Wrapper/Factory)
# ─────────────────────────────────────────────
class ControlAllocator:
    def __init__(self, method, **kwargs):
        if method == 'fixed_tilt':
            self.allocator = FixedTiltAllocator(**kwargs)
        elif method == 'variable_tilt':
            self.allocator = VariableTiltAllocator(**kwargs)
        else:
            raise ValueError(f"Unsupported allocation method: {method}")

    def allocate(self, desired_wrench):
        return self.allocator.allocate(desired_wrench)

    def thrust_to_motor_speeds(self, thrusts):
        return self.allocator.thrust_to_motor_speeds(thrusts)
