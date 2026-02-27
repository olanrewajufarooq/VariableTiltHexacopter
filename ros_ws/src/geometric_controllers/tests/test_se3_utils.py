import numpy as np

from geometric_controllers.utils import (
    Ad,
    Ad_inv,
    ad,
    hat,
    vee,
    get_generalized_inertia,
)


def test_hat_vee_roundtrip():
    v = np.array([0.3, -1.2, 2.4])
    assert np.allclose(vee(hat(v)).reshape(-1), v)


def test_adjoint_inverse_identity():
    R = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    p = np.array([1.0, -2.0, 0.5])
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = p
    eye6 = np.eye(6)
    assert np.allclose(Ad_inv(H) @ Ad(H), eye6)


def test_ad_matrix_structure():
    V = np.array([0.1, -0.2, 0.3, 1.0, -0.5, 0.2])
    ad_V = ad(V)
    assert ad_V.shape == (6, 6)
    assert np.allclose(ad_V[:3, :3], hat(V[:3]))
    assert np.allclose(ad_V[3:6, :3], hat(V[3:6]))
    assert np.allclose(ad_V[3:6, 3:6], hat(V[:3]))


def test_generalized_inertia_blocks():
    m = 2.0
    I = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
    cog = np.array([0.4, -0.3, 0.2])
    G = get_generalized_inertia(m, I, cog)
    assert G.shape == (6, 6)
    assert np.allclose(G[3:6, 3:6], m * np.eye(3))
    assert np.allclose(G[:3, 3:6], m * hat(cog))
    assert np.allclose(G[3:6, :3], -m * hat(cog))
