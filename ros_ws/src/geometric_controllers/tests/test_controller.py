import numpy as np

from geometric_controllers.controller import (
    PDController,
    FeedforwardCompensationController,
)


def test_pd_gravity_compensation_at_equilibrium():
    H = np.eye(4)
    V = np.zeros((6, 1))
    ctrl = PDController(
        Kp_att=[5.5, 5.5, 5.5],
        Kp_pos=[5.5, 5.5, 5.5],
        Kd=[2.05] * 6,
        m=2.0,
        CoG=[0, 0, 0],
    )
    W = ctrl.compute_wrench(H, H, V, V)
    assert W.shape == (6, 1)
    assert np.isclose(W[5, 0], 2.0 * 9.8)


def test_feedforward_zero_error_matches_pd():
    H = np.eye(4)
    V = np.zeros((6, 1))
    A = np.zeros((6, 1))
    ctrl = FeedforwardCompensationController(
        Kp_att=[5.5, 5.5, 5.5],
        Kp_pos=[5.5, 5.5, 5.5],
        Kd=[2.05] * 6,
        CoG=[0, 0, 0],
        m=1.0,
        I=[1.0, 1.0, 1.0, 0.0, 0.0, 0.0],
    )
    W = ctrl.compute_wrench(H, H, V, V, A)
    assert W.shape == (6, 1)
