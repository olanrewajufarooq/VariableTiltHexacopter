import numpy as np

from geometric_controllers.trajectories import PreComputedPath


def assert_rotation_matrix(R):
    eye = np.eye(3)
    assert np.allclose(R.T @ R, eye, atol=1e-6)
    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-6)


def test_circle_path_initial_point():
    path = PreComputedPath(name="circle", scale=2.0, period=10.0, altitude=5.0)
    H, V, A = path.generate(0.0)
    assert np.allclose(H[:3, 3], np.array([0.0, 0.0, 5.0]))
    assert V.shape == (6, 1)
    assert A.shape == (6, 1)
    assert_rotation_matrix(H[:3, :3])


def test_infinity3d_height_variation():
    path = PreComputedPath(name="infinity3d", scale=4.0, period=12.0, altitude=6.0)
    H0, _, _ = path.generate(0.0)
    Hq, _, _ = path.generate(3.0)
    assert not np.isclose(H0[2, 3], Hq[2, 3])


def test_lissajous3d_returns_se3():
    path = PreComputedPath(name="lissajous3d", scale=3.0, period=8.0, altitude=4.0)
    H, V, A = path.generate(1.0)
    assert H.shape == (4, 4)
    assert V.shape == (6, 1)
    assert A.shape == (6, 1)
    assert_rotation_matrix(H[:3, :3])


def test_takeoff_land_ramp():
    path = PreComputedPath(name="takeoffland", scale=2.0, period=8.0, altitude=3.0)
    H0, _, _ = path.generate(0.0)
    H1, _, _ = path.generate(1.0)
    assert H1[2, 3] >= H0[2, 3]
