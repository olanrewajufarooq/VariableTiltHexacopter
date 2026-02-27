from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Literal

import numpy as np

from geometric_controllers.utils import log_se3, vee


class Potential(ABC):
    """Pose error potential.

    Returns a 6x1 error vector `e = [e_R; e_p]`.
    The controller is responsible for applying gains and signs.
    """

    @abstractmethod
    def error(self, H_des: np.ndarray, H: np.ndarray) -> np.ndarray:
        raise NotImplementedError


class LieAlgebraPotential(Potential):
    def error(self, H_des: np.ndarray, H: np.ndarray) -> np.ndarray:
        H_err = np.linalg.inv(H_des) @ H
        return log_se3(H_err)


class SeparatePotential(Potential):
    def error(self, H_des: np.ndarray, H: np.ndarray) -> np.ndarray:
        R = H[:3, :3]
        p = H[:3, 3].reshape(3, 1)

        R_des = H_des[:3, :3]
        p_des = H_des[:3, 3].reshape(3, 1)

        e_p = p - p_des
        R_err = R_des.T @ R
        e_R = vee(0.5 * (R_err - R_err.T))
        return np.vstack((e_R, e_p)).reshape(6, 1)


PotentialType = Literal["liealgebra", "separate"]


class PotentialFactory:
    @staticmethod
    def create(potential_type: str) -> Potential:
        if potential_type == "liealgebra":
            return LieAlgebraPotential()
        if potential_type == "separate":
            return SeparatePotential()
        raise ValueError(f"Unknown potential type: {potential_type}")
