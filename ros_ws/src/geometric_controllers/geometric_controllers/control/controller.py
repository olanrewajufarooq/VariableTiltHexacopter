from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from geometric_controllers.control.adaptation import AdaptationFactory
from geometric_controllers.control.potentials import PotentialFactory
from geometric_controllers.utils import Ad, Ad_inv, ad, get_generalized_inertia


@dataclass
class ControllerConfig:
    controller_type: str = "PD"  # PD|FeedLin|FeedForward
    potential_type: str = "liealgebra"  # liealgebra|separate
    adaptation_type: str = "None"  # None|Euclidean|GeoAware


class WrenchController:
    """Core wrench controller (ROS-agnostic).

    Mirrors the MATLAB split:
    - potential: pose error -> 6D error vector
    - adaptation: parameter estimation (optional)
    and a controller type:
    - PD / FeedLin / FeedForward
    """

    def __init__(
        self,
        *,
        Kp_att: list[float],
        Kp_pos: list[float],
        Kd: list[float],
        mass: float,
        I: list[float],
        CoG: list[float],
        gravity: float,
        cfg: Optional[ControllerConfig] = None,
        gamma: Optional[np.ndarray] = None,
    ):
        if cfg is None:
            cfg = ControllerConfig()
        self.cfg = cfg

        self.Kp_att = np.diag([float(x) for x in Kp_att])
        self.Kp_pos = np.diag([float(x) for x in Kp_pos])
        self.Kd = np.diag([float(x) for x in Kd])
        self.Kp = np.diag([*self.Kp_att.diagonal(), *self.Kp_pos.diagonal()])

        self.gravity = float(gravity)
        self._nominal_mass = float(mass)
        self._nominal_I = np.asarray(I, dtype=float).reshape(6)
        self._nominal_cog = np.asarray(CoG, dtype=float).reshape(3)

        self.potential = PotentialFactory.create(cfg.potential_type)
        self.adaptation = AdaptationFactory.create(
            cfg.adaptation_type,
            mass=self._nominal_mass,
            cog=self._nominal_cog,
            I=self._nominal_I,
            gravity=self.gravity,
            gamma=gamma,
        )

        # Scratch buffers
        self._H_err: Optional[np.ndarray] = None

    def _gravity_wrench_body(self, *, H: np.ndarray, mass: float, cog: np.ndarray) -> np.ndarray:
        Hg = H.copy()
        Hg[:3, 3] = cog.reshape(3)
        g_world = np.vstack(
            (np.zeros((3, 1)), np.array([0.0, 0.0, -mass * self.gravity]).reshape(3, 1))
        )
        Wg = Ad(Hg).T @ g_world
        if np.iscomplexobj(Wg):
            Wg = Wg.astype(float)
        return Wg

    def compute_wrench(
        self,
        *,
        H_des: np.ndarray,
        H: np.ndarray,
        V_des: np.ndarray,
        V: np.ndarray,
        A_des: Optional[np.ndarray] = None,
        dt: Optional[float] = None,
    ) -> np.ndarray:
        if dt is None:
            dt = 1e-3
        if A_des is None:
            A_des = np.zeros((6, 1))

        H_err = np.linalg.inv(H_des) @ H
        self._H_err = H_err

        # Adaptation updates estimated parameters.
        self.adaptation.step(
            H_des=H_des,
            H=H,
            V_des=V_des,
            V=V,
            A_des=A_des,
            dt=float(dt),
        )
        mass, cog, I6 = self.adaptation.get_params()

        I_spatial = get_generalized_inertia(m=mass, I=I6, cog=cog)

        # Pose error term
        e = self.potential.error(H_des, H)
        Wp = -self.Kp @ e

        # Gravity wrench (body frame)
        Wg = self._gravity_wrench_body(H=H, mass=mass, cog=cog)

        # Velocity error
        V_err = V - Ad_inv(H_err) @ V_des
        Wd = -self.Kd @ V_err

        W = Wp - Wg + Wd

        ctype = str(self.cfg.controller_type)
        if ctype == "PD":
            return W

        coriolis = ad(V).T @ I_spatial @ V
        if ctype == "FeedLin":
            return coriolis + W

        if ctype == "FeedForward":
            V_e = V_err
            ff = I_spatial @ Ad_inv(H_err) @ (A_des + ad(V_des) @ (Ad(H_err) @ V_e))
            return coriolis + ff + W

        raise ValueError(f"Unknown controller_type: {ctype}")


class Controller:
    """Convenience wrapper matching existing node call sites."""

    def __init__(
        self,
        *,
        controller_type: str,
        adaptation_type: str,
        potential_type: str,
        Kp_att: list[float],
        Kp_pos: list[float],
        Kd: list[float],
        m: float,
        I: list[float],
        CoG: list[float],
        gravity: float,
        gamma: Optional[np.ndarray] = None,
    ):
        cfg = ControllerConfig(
            controller_type=str(controller_type),
            potential_type=str(potential_type),
            adaptation_type=str(adaptation_type),
        )
        self._impl = WrenchController(
            Kp_att=Kp_att,
            Kp_pos=Kp_pos,
            Kd=Kd,
            mass=m,
            I=I,
            CoG=CoG,
            gravity=gravity,
            cfg=cfg,
            gamma=gamma,
        )

    def compute_wrench(self, *, H_des, H, V_des, V, A_des=None, dt=None) -> np.ndarray:
        return self._impl.compute_wrench(
            H_des=H_des, H=H, V_des=V_des, V=V, A_des=A_des, dt=dt
        )
