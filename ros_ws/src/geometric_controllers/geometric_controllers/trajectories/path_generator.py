#!/usr/bin/env python3

from __future__ import annotations

import math
import numpy as np
from abc import ABC, abstractmethod


def smooth_time_scaling(t: float, T: float):
    if T <= 0:
        return 0.0, 0.0, 0.0
    tau = np.clip(t / T, 0.0, 1.0)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    sd = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
    sdd = (60 * tau - 180 * tau**2 + 120 * tau**3) / (T**2)
    return s, sd, sdd


def yaw_from_velocity(v_world, a_world, last_yaw):
    vx, vy = float(v_world[0]), float(v_world[1])
    ax, ay = float(a_world[0]), float(a_world[1])
    denom = vx * vx + vy * vy
    if denom < 1e-12:
        return last_yaw, 0.0, 0.0, last_yaw
    yaw = math.atan2(vy, vx)
    wyaw = (vx * ay - vy * ax) / denom
    return yaw, wyaw, 0.0, yaw


def rpy_profile(s, sdot, sddot, amp, freq, phase):
    theta = 2 * math.pi * freq * s + phase
    theta_dot = 2 * math.pi * freq * sdot
    theta_ddot = 2 * math.pi * freq * sddot
    rpy = amp * np.sin(theta)
    rpy_dot = amp * np.cos(theta) * theta_dot
    rpy_ddot = amp * (-np.sin(theta) * (theta_dot**2) + np.cos(theta) * theta_ddot)
    return rpy, rpy_dot, rpy_ddot


def rpy_to_rotation(rpy):
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def rpy_rates_to_omega(rpy, rpy_dot, rpy_ddot):
    roll, pitch, _ = float(rpy[0]), float(rpy[1]), float(rpy[2])
    phi_dot, theta_dot, _ = float(rpy_dot[0]), float(rpy_dot[1]), float(rpy_dot[2])
    sphi, cphi = math.sin(roll), math.cos(roll)
    st, ct = math.sin(pitch), math.cos(pitch)

    T = np.array(
        [
            [1.0, 0.0, -st],
            [0.0, cphi, ct * sphi],
            [0.0, -sphi, ct * cphi],
        ]
    )

    T_dot = np.array(
        [
            [0.0, 0.0, -ct * theta_dot],
            [0.0, -sphi * phi_dot, (-st * theta_dot) * sphi + ct * cphi * phi_dot],
            [0.0, -cphi * phi_dot, (-st * theta_dot) * cphi + ct * (-sphi) * phi_dot],
        ]
    )

    omega = T @ rpy_dot
    omega_dot = T @ rpy_ddot + T_dot @ rpy_dot
    return omega, omega_dot


class BasePath(ABC):
    def __init__(
        self,
        scale: float,
        period: float,
        altitude: float,
        start_with_hover: bool = False,
    ):
        self.scale = scale
        self.period = period
        self.altitude = altitude
        self.start_with_hover = start_with_hover

    @abstractmethod
    def generate(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        raise NotImplementedError


class PreComputedPath(BasePath):
    def __init__(
        self,
        name: str,
        scale: float,
        period: float,
        altitude: float,
        start_with_hover: bool = False,
        **kwargs,
    ):
        super().__init__(scale, period, altitude, start_with_hover)
        self.name = name.lower()

        self.hover_frac = float(kwargs.get("hover_frac", 0.1))

        self.lissajous_amp = np.array(
            kwargs.get("lissajous_amp", [scale, scale, min(scale / 2, altitude / 2)]),
            dtype=float,
        ).reshape(3)
        self.lissajous_freq = np.array(
            kwargs.get("lissajous_freq", [1, 2, 3]), dtype=float
        ).reshape(3)
        self.lissajous_phase = np.array(
            kwargs.get("lissajous_phase", [0, math.pi / 2, math.pi / 4]), dtype=float
        ).reshape(3)

        self.helix_turns = float(kwargs.get("helix_turns", 1))
        self.helix_z_amp = float(
            kwargs.get("helix_z_amp", min(scale / 2, altitude / 2))
        )

        self.inf3d_mod_alpha = float(kwargs.get("inf3d_mod_alpha", 0.3))
        self.inf3d_mod_beta = float(kwargs.get("inf3d_mod_beta", 0.25))

        self.rpy_amp = np.array(
            kwargs.get("rpy_amp", np.deg2rad([10, 10, 20])), dtype=float
        ).reshape(3)
        self.rpy_freq = np.array(
            kwargs.get("rpy_freq", [1, 2, 3]), dtype=float
        ).reshape(3)
        self.rpy_phase = np.array(
            kwargs.get("rpy_phase", [0, 0, 0]), dtype=float
        ).reshape(3)

        self.use_yaw_from_velocity = self.name not in {
            "lissajous3d",
            "helix3d",
            "infinity3dmod",
            "poly3d",
        }
        if "use_yaw_from_velocity" in kwargs:
            self.use_yaw_from_velocity = bool(kwargs["use_yaw_from_velocity"])

        self.last_yaw = 0.0

        z_amp = min(scale / 2, altitude / 2)
        coeff = kwargs.get("poly_coeff")
        if coeff is None:
            self.poly_coeff = np.array(
                [
                    [0, 16, -32, 16, 0, 0],
                    [32, -80, 64, -16, 0, 0],
                    [0, 16, -32, 16, 0, altitude],
                ],
                dtype=float,
            )
            self.poly_coeff[0, :] *= scale
            self.poly_coeff[1, :] *= scale
            self.poly_coeff[2, 0:5] *= z_amp
        else:
            coeff = np.asarray(coeff, dtype=float)
            if coeff.shape[0] != 3 and coeff.shape[1] == 3:
                coeff = coeff.T
            if coeff.shape[0] != 3:
                raise ValueError("poly_coeff must be a 3xN matrix")
            self.poly_coeff = coeff

    def _segment_time(self, t):
        if self.start_with_hover:
            hover_time = self.hover_frac * self.period
            if t < hover_time:
                s, sd, sdd = smooth_time_scaling(t, hover_time)
                p = np.array([0.0, 0.0, self.altitude * s])
                v = np.array([0.0, 0.0, self.altitude * sd])
                a = np.array([0.0, 0.0, self.altitude * sdd])
                return p, v, a, np.zeros(3), np.zeros(3), np.zeros(3)
            t -= hover_time
        else:
            hover_time = 0.0

        tmod = t % self.period
        if self.start_with_hover:
            s, sd, sdd = smooth_time_scaling(tmod, self.period)
        else:
            s = tmod / self.period
            sd = 1.0 / self.period
            sdd = 0.0
        return None, None, None, s, sd, sdd

    def _build_output(self, p, v, a, rpy, rpy_dot, rpy_ddot):
        R = rpy_to_rotation(rpy)
        omega, omega_dot = rpy_rates_to_omega(rpy, rpy_dot, rpy_ddot)

        v_body = R.T @ v.reshape(3, 1)
        a_body = R.T @ a.reshape(3, 1) - np.cross(
            omega.reshape(3), v_body.reshape(3)
        ).reshape(3, 1)

        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = p.reshape(3)

        V = np.vstack((omega.reshape(3, 1), v_body))
        A = np.vstack((omega_dot.reshape(3, 1), a_body))
        return H, V, A

    def generate(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        p_pre, v_pre, a_pre, s, sdot, sddot = self._segment_time(t)
        if p_pre is not None:
            rpy = np.zeros(3)
            return self._build_output(
                p_pre, v_pre, a_pre, rpy, np.zeros(3), np.zeros(3)
            )

        theta = 2 * math.pi * s
        use_rpy_profile = self.name in {
            "lissajous3d",
            "helix3d",
            "infinity3dmod",
            "poly3d",
        }

        if self.name == "hover":
            p = np.array([0.0, 0.0, self.altitude])
            v = np.zeros(3)
            a = np.zeros(3)
        elif self.name == "circle":
            r = self.scale
            p = np.array(
                [r * (math.cos(theta) - 1.0), r * math.sin(theta), self.altitude]
            )
            dp = (
                2 * math.pi * np.array([-r * math.sin(theta), r * math.cos(theta), 0.0])
            )
            d2p = (2 * math.pi) ** 2 * np.array(
                [-r * math.cos(theta), -r * math.sin(theta), 0.0]
            )
            v = dp * sdot
            a = d2p * (sdot**2) + dp * sddot
        elif self.name in {"infinity", "infinity_loop"}:
            a0 = self.scale
            p = np.array(
                [a0 * math.sin(theta), a0 * math.sin(2 * theta) / 2.0, self.altitude]
            )
            dp = (
                2
                * math.pi
                * np.array([a0 * math.cos(theta), a0 * math.cos(2 * theta), 0.0])
            )
            d2p = (2 * math.pi) ** 2 * np.array(
                [-a0 * math.sin(theta), -2 * a0 * math.sin(2 * theta), 0.0]
            )
            v = dp * sdot
            a = d2p * (sdot**2) + dp * sddot
        elif self.name == "infinity3d":
            a0 = self.scale
            z_amp = min(self.scale / 2, self.altitude / 2)
            p = np.array(
                [
                    a0 * math.sin(theta),
                    a0 * math.sin(2 * theta) / 2.0,
                    self.altitude + z_amp * math.sin(theta),
                ]
            )
            dp = (
                2
                * math.pi
                * np.array(
                    [
                        a0 * math.cos(theta),
                        a0 * math.cos(2 * theta),
                        z_amp * math.cos(theta),
                    ]
                )
            )
            d2p = (2 * math.pi) ** 2 * np.array(
                [
                    -a0 * math.sin(theta),
                    -2 * a0 * math.sin(2 * theta),
                    -z_amp * math.sin(theta),
                ]
            )
            v = dp * sdot
            a = d2p * (sdot**2) + dp * sddot
        elif self.name == "lissajous3d":
            amp = self.lissajous_amp
            freq = self.lissajous_freq
            phase = self.lissajous_phase
            th = 2 * math.pi * freq * s + phase
            p = amp * np.sin(th)
            p0 = amp * np.sin(phase)
            p = p - p0 + np.array([0.0, 0.0, self.altitude])
            dp = 2 * math.pi * amp * freq * np.cos(th)
            d2p = -((2 * math.pi) ** 2) * amp * (freq**2) * np.sin(th)
            v = dp * sdot
            a = d2p * (sdot**2) + dp * sddot
            use_rpy_profile = True
        elif self.name == "helix3d":
            r = self.scale
            turns = self.helix_turns
            z_amp = self.helix_z_amp
            th = 2 * math.pi * turns * s
            p = np.array(
                [
                    r * math.cos(th),
                    r * math.sin(th),
                    self.altitude + z_amp * math.sin(th),
                ]
            )
            p0 = np.array([r, 0.0, self.altitude])
            p = p - p0 + np.array([0.0, 0.0, self.altitude])
            dp = (
                2
                * math.pi
                * turns
                * np.array([-r * math.sin(th), r * math.cos(th), z_amp * math.cos(th)])
            )
            d2p = (2 * math.pi * turns) ** 2 * np.array(
                [-r * math.cos(th), -r * math.sin(th), -z_amp * math.sin(th)]
            )
            v = dp * sdot
            a = d2p * (sdot**2) + dp * sddot
            use_rpy_profile = True
        elif self.name == "infinity3dmod":
            a0 = self.scale
            z_amp = min(self.scale / 2, self.altitude / 2)
            alpha = self.inf3d_mod_alpha
            beta = self.inf3d_mod_beta
            s1 = math.sin(theta)
            c1 = math.cos(theta)
            s2 = math.sin(2 * theta)
            c2 = math.cos(2 * theta)
            x = a0 * s1 * (1 + alpha * s2)
            y = (a0 / 2.0) * s2 * (1 + alpha * s1)
            z = self.altitude + z_amp * s1 * (1 + beta * s2)
            p = np.array([x, y, z])
            dx_dth = a0 * (c1 + alpha * (c1 * s2 + 2 * s1 * c2))
            dy_dth = (a0 / 2.0) * (2 * c2 + alpha * (c1 * s2 + 2 * s1 * c2))
            dz_dth = z_amp * (c1 + beta * (c1 * s2 + 2 * s1 * c2))
            d2x_dth2 = a0 * (-s1 + alpha * (-5 * s1 * s2 + 4 * c1 * c2))
            d2y_dth2 = (a0 / 2.0) * (-4 * s2 + alpha * (-5 * s1 * s2 + 4 * c1 * c2))
            d2z_dth2 = z_amp * (-s1 + beta * (-5 * s1 * s2 + 4 * c1 * c2))
            dp = 2 * math.pi * np.array([dx_dth, dy_dth, dz_dth])
            d2p = (2 * math.pi) ** 2 * np.array([d2x_dth2, d2y_dth2, d2z_dth2])
            v = dp * sdot
            a = d2p * (sdot**2) + dp * sddot
            use_rpy_profile = True
        elif self.name == "poly3d":
            coeff = self.poly_coeff
            p = np.array(
                [
                    np.polyval(coeff[0, :], s),
                    np.polyval(coeff[1, :], s),
                    np.polyval(coeff[2, :], s),
                ]
            )
            p0 = np.array(
                [
                    np.polyval(coeff[0, :], 0),
                    np.polyval(coeff[1, :], 0),
                    np.polyval(coeff[2, :], 0),
                ]
            )
            p = p - p0 + np.array([0.0, 0.0, self.altitude])
            dp = np.array(
                [
                    np.polyval(np.polyder(coeff[0, :]), s),
                    np.polyval(np.polyder(coeff[1, :]), s),
                    np.polyval(np.polyder(coeff[2, :]), s),
                ]
            )
            d2p = np.array(
                [
                    np.polyval(np.polyder(np.polyder(coeff[0, :])), s),
                    np.polyval(np.polyder(np.polyder(coeff[1, :])), s),
                    np.polyval(np.polyder(np.polyder(coeff[2, :])), s),
                ]
            )
            v = dp * sdot
            a = d2p * (sdot**2) + dp * sddot
            use_rpy_profile = True
        elif self.name in {"takeoffland", "takeoff_land"}:
            T = self.period
            t1 = 0.25 * T
            t2 = 0.75 * T
            tmod = t % T
            if tmod < t1:
                s1, sd1, sdd1 = smooth_time_scaling(tmod, t1)
                p = np.array([0.0, 0.0, self.altitude * s1])
                v = np.array([0.0, 0.0, self.altitude * sd1])
                a = np.array([0.0, 0.0, self.altitude * sdd1])
            elif tmod < t2:
                s2, sd2, sdd2 = smooth_time_scaling(tmod - t1, t2 - t1)
                p = np.array([self.scale * s2, 0.0, self.altitude])
                v = np.array([self.scale * sd2, 0.0, 0.0])
                a = np.array([self.scale * sdd2, 0.0, 0.0])
            else:
                s3, sd3, sdd3 = smooth_time_scaling(tmod - t2, T - t2)
                p = np.array([self.scale, 0.0, self.altitude * (1.0 - s3)])
                v = np.array([0.0, 0.0, -self.altitude * sd3])
                a = np.array([0.0, 0.0, -self.altitude * sdd3])
        else:
            p = np.array([0.0, 0.0, self.altitude])
            v = np.zeros(3)
            a = np.zeros(3)

        if use_rpy_profile:
            rpy, rpy_dot, rpy_ddot = rpy_profile(
                s, sdot, sddot, self.rpy_amp, self.rpy_freq, self.rpy_phase
            )
        else:
            rpy = np.zeros(3)
            rpy_dot = np.zeros(3)
            rpy_ddot = np.zeros(3)

        if self.use_yaw_from_velocity:
            yaw, wyaw, wyawdot, self.last_yaw = yaw_from_velocity(v, a, self.last_yaw)
            rpy[2] += yaw
            rpy_dot[2] += wyaw
            rpy_ddot[2] += wyawdot

        return self._build_output(p, v, a, rpy, rpy_dot, rpy_ddot)


class PathGenerator:
    def __init__(
        self,
        path_name: str = "hover",
        scale: float = 5.0,
        period: float = 20.0,
        altitude: float = 5.0,
        start_with_hover: bool = False,
        **kwargs,
    ):
        self.scale = scale
        self.period = period
        self.altitude = altitude
        self.start_with_hover = start_with_hover
        self.kwargs = kwargs
        self._start = None
        self.set_path(path_name)

    def set_path(self, name: str):
        name = name.lower()
        # Accept common launch-file spelling.
        if name == "takeoff_land":
            name = "takeoffland"
        self.path = PreComputedPath(
            name=name,
            scale=self.scale,
            period=self.period,
            altitude=self.altitude,
            start_with_hover=self.start_with_hover,
            **self.kwargs,
        )
        self._start = None

    def generate(
        self, current_time_sec: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if self._start is None:
            self._start = current_time_sec
        t = current_time_sec - self._start
        return self.path.generate(t)
