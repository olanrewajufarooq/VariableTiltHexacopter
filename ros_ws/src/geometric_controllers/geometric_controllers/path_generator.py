#!/usr/bin/env python3
import numpy as np
import math
from abc import ABC, abstractmethod
from scipy.spatial.transform import Rotation as R

class BasePath(ABC):
    """Abstract base class for a 3D trajectory generator."""
    def __init__(self, scale: float, period: float, altitude: float):
        self.scale    = scale
        self.period   = period
        self.altitude = altitude

    @abstractmethod
    def generate(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute pose H (4×4) and spatial velocity V (6×1) at time t.
        :param t: elapsed time since start in seconds
        :return: (H, V)
        """
        pass

class HoverPath(BasePath):
    """Stationary hover at fixed altitude."""
    def generate(self, t: float):
        H = np.eye(4)
        H[2, 3] = self.altitude
        V = np.zeros((6, 1))
        return H, V

class CirclePath(BasePath):
    """Circle in XY plane, constant altitude."""
    def __init__(self, scale, period, altitude):
        super().__init__(scale, period, altitude)
        self.omega = 2*math.pi / period

    def generate(self, t: float):
        θ = self.omega * t
        r = self.scale
        x, y = r*math.cos(θ), r*math.sin(θ)
        vx = -r*self.omega*math.sin(θ)
        vy =  r*self.omega*math.cos(θ)
        yaw = θ + math.pi/2

        Rz = R.from_euler('z', yaw).as_matrix()
        H  = np.eye(4)
        H[:3,:3] = Rz
        H[:3,3]  = [x, y, self.altitude]

        V = np.array([0, 0, 0, vx, vy, 0]).reshape(6,1)
        return H, V

class SquarePath(BasePath):
    """Square loop in XY plane, constant altitude."""
    def __init__(self, scale, period, altitude):
        super().__init__(scale, period, altitude)
        self.st = period / 4
        self.v_lin     = 2*scale / self.side_time

    def generate(self, t: float):
        s  = self.scale
        vm = self.v_lin
        tm = t % (4* self.side_time)
        side = int(tm // self.side_time)
        dt   = tm % self.side_time

        if   side == 0:
            x, y, vx, vy =  s, -s + vm*dt,   0,  vm
        elif side == 1:
            x, y, vx, vy =  s - vm*dt, s,  -vm,   0
        elif side == 2:
            x, y, vx, vy = -s,  s - vm*dt,   0, -vm
        else:
            x, y, vx, vy = -s + vm*dt, -s,  vm,   0

        yaw = math.pi/2 + side*(math.pi/2)

        Rz  = R.from_euler('z', yaw).as_matrix()

        H   = np.eye(4); 
        H[:3,:3] = Rz; 
        H[:3,3] = [x, y, self.altitude]

        V   = np.array([0, 0, 0, vx, vy, 0]).reshape(6,1)
        return H, V

class InfinityPath(BasePath):
    """Figure‐8 (lemniscate) in XY plane, constant altitude."""
    def __init__(self, scale, period, altitude):
        super().__init__(scale, period, altitude)
        self.omega = 2*math.pi / period
        self.a = self.scale

    def generate(self, t: float):
        θ = self.omega * t
        x = self.a * math.sin(θ)
        y = self.a * math.sin(2*θ) / 2
        vx = self.a * self.omega * math.cos(θ)
        vy = self.a * self.omega * math.cos(2*θ)
        yaw = math.atan2(vy, vx)

        Rz = R.from_euler('z', yaw).as_matrix()
        H  = np.eye(4); 
        H[:3,:3] = Rz; 
        H[:3,3] = [x, y, self.altitude]

        V  = np.array([0, 0, 0, vx, vy, 0]).reshape(6,1)
        return H, V

class TakeoffLandPath(BasePath):
    """Simple takeoff, straight move, and landing profile."""
    def __init__(self, scale, period, altitude):
        super().__init__(scale, period, altitude)
        # split period: takeoff 25%, move 50%, land 25%
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
        return H, V

class PathGenerator:
    """Selects among multiple BasePath strategies."""
    def __init__(self, path_name: str = "hover", scale: float     = 5.0,
                 period: float    = 20.0, altitude: float  = 5.0):
        
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

        self.set_path(path_name)
        self._start = None

    def set_path(self, name: str):
        cls = self.path_class.get(name, HoverPath)
        self.path = cls(self.scale, self.period, self.altitude)
        self._start = None

    def generate(self, current_time_sec: float):
        """Return (H, V) at now - start_time."""
        if self._start is None:
            self._start = current_time_sec

        t = current_time_sec - self._start
        return self.path.generate(t)
