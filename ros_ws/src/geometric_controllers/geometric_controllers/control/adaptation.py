from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from geometric_controllers.utils import Ad, Ad_inv, ad, get_generalized_inertia, hat


class Adaptation(ABC):
    """Parameter adaptation interface.

    This is ROS-agnostic core logic. It only consumes numeric arrays.
    """

    @abstractmethod
    def step(
        self,
        *,
        H_des: np.ndarray,
        H: np.ndarray,
        V_des: np.ndarray,
        V: np.ndarray,
        A_des: np.ndarray,
        dt: float,
    ) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_params(self) -> tuple[float, np.ndarray, np.ndarray]:
        """Return (mass, CoG(3,), I6_params(6,))."""


class NoAdaptation(Adaptation):
    def __init__(self, *, mass: float, cog: np.ndarray, I: np.ndarray):
        self._mass = float(mass)
        self._cog = np.asarray(cog, dtype=float).reshape(3)
        self._I = np.asarray(I, dtype=float).reshape(6)

    def step(
        self,
        *,
        H_des: np.ndarray,
        H: np.ndarray,
        V_des: np.ndarray,
        V: np.ndarray,
        A_des: np.ndarray,
        dt: float,
    ) -> None:
        return

    def get_params(self) -> tuple[float, np.ndarray, np.ndarray]:
        return self._mass, self._cog.copy(), self._I.copy()


class EuclideanAdaptation(Adaptation):
    r"""Euclidean adaptive estimation.

    Tracks:
        theta = [Ixx, Iyy, Izz, Ixy, Iyz, Ixz, m, m*Cx, m*Cy, m*Cz]^T
    """

    def __init__(
        self,
        *,
        mass: float,
        cog: np.ndarray,
        I: np.ndarray,
        gravity: float,
        gamma: Optional[np.ndarray] = None,
    ):
        Ixx, Iyy, Izz, Ixy, Ixz, Iyz = np.asarray(I, dtype=float).reshape(6)
        cog = np.asarray(cog, dtype=float).reshape(3)
        m = float(mass)

        self.gravity = float(gravity)
        self.theta_hat = np.array(
            [Ixx, Iyy, Izz, Ixy, Iyz, Ixz, m, m * cog[0], m * cog[1], m * cog[2]],
            dtype=float,
        ).reshape(10, 1)

        if gamma is None:
            self.gamma = 4e-3 * np.diag([20, 20, 30, 1, 1, 1, 90, 30, 30, 60])
        else:
            g = np.asarray(gamma, dtype=float)
            if g.shape == (10,):
                self.gamma = np.diag(g)
            elif g.shape == (10, 10):
                self.gamma = g
            else:
                raise ValueError("gamma must be shape (10,) or (10,10)")

        self._construct_bases()

    def _construct_bases(self) -> None:
        self.I_basis: list[np.ndarray] = []

        # i = 1..3 diagonal moments
        for k in range(3):
            E = np.zeros((3, 3))
            E[k, k] = 1.0
            Gi = np.zeros((6, 6))
            Gi[:3, :3] = E
            self.I_basis.append(Gi)

        # i = 4..6 off-diagonal symmetric products (xy, yz, xz)
        for i, j in [(0, 1), (1, 2), (0, 2)]:
            E = np.zeros((3, 3))
            E[i, j] = E[j, i] = 1.0
            Gi = np.zeros((6, 6))
            Gi[:3, :3] = E
            self.I_basis.append(Gi)

        # i = 7 mass translational block
        Gi = np.zeros((6, 6))
        Gi[3:, 3:] = np.eye(3)
        self.I_basis.append(Gi)

        # i = 8..10 m*CoG cross-terms
        for ax in range(3):
            e = np.eye(3)[ax]
            S = hat(e)
            Gi = np.zeros((6, 6))
            Gi[:3, 3:] = S
            Gi[3:, :3] = S
            self.I_basis.append(Gi)

        self.G_basis: list[np.ndarray] = []
        G7 = np.zeros((6, 6))
        G7[3:, 3:] = np.eye(3)
        self.G_basis.append(G7)
        for ax in range(3):
            e = np.eye(3)[ax]
            S = hat(e)
            Gi = np.zeros((6, 6))
            Gi[:3, :3] = S
            self.G_basis.append(Gi)

    def _unpack_theta(self) -> tuple[float, np.ndarray, np.ndarray]:
        Ixx, Iyy, Izz, Ixy, Iyz, Ixz, m, mCx, mCy, mCz = self.theta_hat.flatten()
        I6 = np.array([Ixx, Iyy, Izz, Ixy, Ixz, Iyz], dtype=float)
        cog = np.array([mCx, mCy, mCz], dtype=float) / max(float(m), 1e-9)
        return float(m), cog, I6

    def _regressor(
        self,
        *,
        H_err: np.ndarray,
        H: np.ndarray,
        V: np.ndarray,
        V_des: np.ndarray,
        A_des: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        Ad_inv_err = Ad_inv(H_err)
        V_e = V - Ad_inv_err @ V_des
        a_bar = A_des + ad(V_des) @ (Ad(H_err) @ V_e)

        R = H[:3, :3]
        g_body = R @ np.array([0, 0, -self.gravity], dtype=float).reshape(3, 1)
        gvec = np.vstack((g_body, g_body))

        cols = []
        for i in range(10):
            term = -ad(V).T @ self.I_basis[i] @ V - self.I_basis[i] @ Ad_inv_err @ a_bar
            if i >= 6:
                term += self.G_basis[i - 6] @ gvec
            cols.append(term)

        Y = np.hstack(cols)
        return Y, V_e

    def step(
        self,
        *,
        H_des: np.ndarray,
        H: np.ndarray,
        V_des: np.ndarray,
        V: np.ndarray,
        A_des: np.ndarray,
        dt: float,
    ) -> None:
        H_err = np.linalg.inv(H_des) @ H
        Y, V_e = self._regressor(H_err=H_err, H=H, V=V, V_des=V_des, A_des=A_des)
        self.theta_hat += self.gamma @ (Y.T @ V_e) * float(dt)

    def get_params(self) -> tuple[float, np.ndarray, np.ndarray]:
        m, cog, I6 = self._unpack_theta()
        return m, cog, I6

    def generalized_inertia(self) -> np.ndarray:
        m, cog, I6 = self._unpack_theta()
        return get_generalized_inertia(m=m, I=I6, cog=cog)


class GeoAwareAdaptation(Adaptation):
    """Geometry-aware adaptation placeholder.

    The MATLAB implementation separates adaptation strategies cleanly. This class
    preserves that structure in ROS, but is intentionally unimplemented for now.
    """

    def step(
        self,
        *,
        H_des: np.ndarray,
        H: np.ndarray,
        V_des: np.ndarray,
        V: np.ndarray,
        A_des: np.ndarray,
        dt: float,
    ) -> None:
        raise NotImplementedError("GeoAwareAdaptation is not implemented yet")

    def get_params(self) -> tuple[float, np.ndarray, np.ndarray]:
        raise NotImplementedError("GeoAwareAdaptation is not implemented yet")


class AdaptationFactory:
    @staticmethod
    def create(
        adaptation_type: str,
        *,
        mass: float,
        cog: np.ndarray,
        I: np.ndarray,
        gravity: float,
        gamma: Optional[np.ndarray] = None,
    ) -> Adaptation:
        t = str(adaptation_type).strip()
        if t.lower() in ("none", "no", "off", "false"):
            return NoAdaptation(mass=mass, cog=cog, I=I)
        if t.lower() in ("euclidean", "euclid"):
            return EuclideanAdaptation(
                mass=mass, cog=cog, I=I, gravity=gravity, gamma=gamma
            )
        if t.lower() in ("geoaware", "geo_aware", "geo-aware"):
            return GeoAwareAdaptation()
        raise ValueError(f"Unknown adaptation type: {adaptation_type}")
