#!/usr/bin/env python3
import numpy as np
import math
from abc import ABC, abstractmethod
from scipy.spatial.transform import Rotation as R

class BasePath(ABC):
    """Abstract base class for a 3D trajectory generator."""
    def __init__(self, scale: float, period: float, altitude: float, start_with_hover: bool = False):
        self.scale    = scale
        self.period   = period
        self.altitude = altitude
        self.start_with_hover  = start_with_hover

    @abstractmethod
    def generate(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute pose H (4×4), spatial velocity V (6×1) and acceleration A (6×1) at time t.
        :param t: elapsed time since start in seconds
        :return: (H, V, A)
        """
        pass

class HoverPath(BasePath):
    """Stationary hover at fixed altitude."""
    def generate(self, t: float):
        H = np.eye(4)
        H[2, 3] = self.altitude
        V = np.zeros((6, 1))
        A = np.zeros((6, 1))
        return H, V, A

class CirclePath(BasePath):
    """Circle in XY plane, constant altitude."""
    def __init__(self, scale, period, altitude, start_with_hover: bool = False):
        super().__init__(scale, period, altitude, start_with_hover)
        self.omega = 2*np.pi / period
        self.hover_transition_time = period * 0.1 if start_with_hover else 0.0

    def generate(self, t: float):
        if self.start_with_hover and t < self.hover_transition_time:
            alpha = t / self.hover_transition_time
            x, y = 0.0, 0.0
            z = self.altitude * alpha
            vx, vy, vz = 0.0, 0.0, 0.0
            ax, ay, az = 0.0, 0.0, 0.0
            yaw, omega, omega_dot = 0.0, 0.0, 0.0
        else:
            t -= self.hover_transition_time

            r = self.scale
            theta = (self.omega * t) % (2 * np.pi)
            
            x, y = r * (np.cos(theta) - 1), r * math.sin(theta)
            
            vx = -r * self.omega * np.sin(theta)
            vy = r * self.omega * np.cos(theta)

            ax = -r * self.omega**2 * np.cos(theta)
            ay = -r * self.omega**2 * np.sin(theta)
            
            z, vz, az = self.altitude, 0.0, 0.0

            yaw = theta
            omega = self.omega
            omega_dot = 0.0

        Rz = R.from_euler('z', yaw).as_matrix()
        H  = np.eye(4)
        H[:3,:3] = Rz
        H[:3,3]  = [x, y, z]

        V = np.array([0, 0, omega, vx, vy, vz]).reshape(6,1)
        A = np.array([0, 0, omega_dot, ax, ay, az]).reshape(6,1)
        return H, V, A

class SquarePath(BasePath):
    """Square loop in XY plane, constant altitude."""
    def __init__(self, scale, period, altitude, start_with_hover: bool = False):
        super().__init__(scale, period, altitude, start_with_hover)
        self.side_time = period / 4
        self.hover_transition_time = period * 0.1 if start_with_hover else 0.0

    def generate(self, t: float):
        if self.start_with_hover and t < self.hover_transition_time:

            alpha = t / self.hover_transition_time
            x, y = 0.0, 0.0
            z = self.altitude * alpha
            vx, vy, vz = 0.0, 0.0, 0.0
            ax, ay, az = 0.0, 0.0, 0.0
            yaw, omega, omega_dot = 0.0, 0.0, 0.0

        else:
            t -= self.hover_transition_time
            s = self.scale
            tm = t % (self.period)
            side = int(tm // self.side_time)
            dt = tm % self.side_time

            vx, vy = 0.0, 0.0

            if side == 0:
                x = s * (dt/self.side_time)
                y = 0
                vx = s / self.side_time
            elif side == 1:
                x = s
                y = s*(dt/self.side_time)
                vy = s / self.side_time
            elif side == 2: 
                x = s - s*(dt/self.side_time)
                y = s
                vx = -s / self.side_time
            else:
                x = 0
                y = s - s*(dt/self.side_time)
                vy = -s / self.side_time
            
            z = self.altitude
            vz = 0.0
            
            yaw, omega, omega_dot = 0.0, 0.0, 0.0

        Rz  = R.from_euler('z', yaw).as_matrix()

        H   = np.eye(4); 
        H[:3,:3] = Rz; 
        H[:3,3] = [x, y, z]

        V   = np.array([0, 0, omega, vx, vy, vz]).reshape(6,1)
        A   = np.array([0, 0, omega_dot, 0, 0, 0]).reshape(6,1)
        return H, V, A

class InfinityPath(BasePath):
    """Figure‐8 (lemniscate) in XY plane, constant altitude."""
    def __init__(self, scale, period, altitude, start_with_hover: bool = False):
        super().__init__(scale, period, altitude, start_with_hover)
        self.omega = 2*math.pi / period
        self.a = self.scale
        self.hover_transition_time = period * 0.1 if start_with_hover else 0.0


    def generate(self, t: float):
        if self.start_with_hover and t < self.hover_transition_time:
            alpha = t / self.hover_transition_time
            x, y = 0.0, 0.0
            z = self.altitude * alpha
            vx, vy, vz = 0.0, 0.0, 0.0
            ax, ay, az = 0.0, 0.0, 0.0
            yaw, omega, omega_dot = 0.0, 0.0, 0.0

        else:
            t -= self.hover_transition_time
            theta = self.omega * t
            a = self.a

            x = a * math.sin(theta)
            y = a * math.sin(2 * theta) / 2

            vx = a * self.omega * math.cos(theta)
            vy = a * self.omega * math.cos(2 * theta)

            ax = -a * self.omega**2 * np.sin(theta)
            ay = -2 * a * self.omega**2 * np.sin(2 * theta)

            z, vz, az = self.altitude, 0.0, 0.0
            yaw, omega, omega_dot = 0.0, 0.0, 0.0

        Rz = R.from_euler('z', yaw).as_matrix()
        H  = np.eye(4); 
        H[:3,:3] = Rz; 
        H[:3,3] = [x, y, z]

        V  = np.array([0, 0, omega, vx, vy, vz]).reshape(6,1)
        A  = np.array([0, 0, omega_dot, ax, ay, az]).reshape(6,1)
        return H, V, A

class TakeoffLandPath(BasePath):
    """Simple start_with_hover, straight move, and landing profile."""
    def __init__(self, scale, period, altitude, start_with_hover: bool = False):
        super().__init__(scale, period, altitude, start_with_hover)
        # split period: start_with_hover 25%, move 50%, land 25%
        self.t1 = period * 0.25
        self.t2 = period * 0.50
        self.t3 = period * 0.25

    def generate(self, t: float):
        z, vz = 0.0, 0.0
        x, y, vx, vy = 0.0, 0.0, 0.0, 0.0
        if t < self.t1:
            z  = self.altitude * (t/self.t1)
            vz = self.altitude / self.t1
        elif t < self.t1 + self.t2:
            z  = self.altitude
            x  = self.scale * ((t-self.t1)/self.t2)
            vx = self.scale / self.t2
        elif t < self.t1 + self.t2 + self.t3:
            z  = self.altitude * (1 - (t-self.t1-self.t2)/self.t3)
            vz = -self.altitude / self.t3
            x  = self.scale
        else:
            z, vz, x = 0.0, 0.0, self.scale

        H = np.eye(4)
        H[:3,3] = [x, y, z]
        V = np.array([0,0,0,vx,vy,vz]).reshape(6,1)
        A = np.array([0,0,0,0,0,0]).reshape(6,1)
        return H, V, A

class PathGenerator:
    """Selects among multiple BasePath strategies."""
    def __init__(self, path_name: str = "hover", scale: float     = 5.0,
                 period: float    = 20.0, altitude: float  = 5.0, 
                 start_with_hover: bool    = False):
        
        self.path_class = {
            "hover":         HoverPath,
            "circle":        CirclePath,
            "square":        SquarePath,
            "infinity_loop": InfinityPath,
            "takeoff_land":  TakeoffLandPath,
        }

        self.scale   = scale
        self.period  = period
        self.altitude= altitude
        self.start_with_hover = start_with_hover

        self.set_path(path_name)
        self._start = None

    def set_path(self, name: str):
        cls = self.path_class.get(name, HoverPath)
        self.path = cls(self.scale, self.period, self.altitude, self.start_with_hover)
        self._start = None

    def generate(self, current_time_sec: float):
        """Return (H, V) at now - start_time."""
        if self._start is None:
            self._start = current_time_sec

        t = current_time_sec - self._start
        return self.path.generate(t)
