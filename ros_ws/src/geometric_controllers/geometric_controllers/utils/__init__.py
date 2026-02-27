import numpy as np


def log_se3(H: np.ndarray) -> np.ndarray:
    """Compute the SE(3) logarithm as a 6x1 vector [omega; v].

    This is a real-valued implementation (avoids `scipy.linalg.logm`), based on the
    standard closed-form SO(3) log with special handling near 0 and pi.
    """
    if H.shape != (4, 4):
        raise ValueError("Input must be a 4x4 homogeneous transform.")

    R = H[:3, :3]
    p = H[:3, 3].reshape(3, 1)
    I3 = np.eye(3)

    # --- SO(3) log ---------------------------------------------------------------
    acosinput = (np.trace(R) - 1.0) / 2.0
    # Clamp for numerical safety.
    acosinput = float(np.clip(acosinput, -1.0, 1.0))

    if acosinput >= 1.0 - 1e-12:
        theta = 0.0
        omega = np.zeros((3, 1))
        omega_hat = np.zeros((3, 3))
    elif acosinput <= -1.0 + 1e-12:
        theta = np.pi
        # Choose a stable axis based on the largest diagonal entry.
        if R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
            v = np.array([R[0, 0] + 1.0, R[1, 0] + R[0, 1], R[2, 0] + R[0, 2]])
        elif R[1, 1] >= R[2, 2]:
            v = np.array([R[0, 1] + R[1, 0], R[1, 1] + 1.0, R[2, 1] + R[1, 2]])
        else:
            v = np.array([R[0, 2] + R[2, 0], R[1, 2] + R[2, 1], R[2, 2] + 1.0])

        n = np.linalg.norm(v)
        if n < 1e-12:
            omega = np.zeros((3, 1))
        else:
            axis = (v / n).reshape(3, 1)
            omega = theta * axis
        omega_hat = hat(omega)
    else:
        theta = float(np.arccos(acosinput))
        omega_hat = (theta / (2.0 * np.sin(theta))) * (R - R.T)
        omega = vee(omega_hat)

    # --- SE(3) log translation part ---------------------------------------------
    if abs(theta) < 1e-9:
        v = p
    else:
        half_theta = 0.5 * theta
        cot_half = np.cos(half_theta) / np.sin(half_theta)
        coeff = (1.0 / (theta * theta)) * (1.0 - half_theta * cot_half)
        V_inv = I3 - 0.5 * omega_hat + coeff * (omega_hat @ omega_hat)
        v = V_inv @ p

    zeta = np.vstack((omega.reshape(3, 1), v.reshape(3, 1))).reshape(6, 1)
    if np.iscomplexobj(zeta):
        zeta = zeta.astype(float)
    return zeta


def vee(X):
    if X.shape == (3, 3):
        out = np.array([X[2, 1], X[0, 2], X[1, 0]]).reshape(3, 1)
    elif X.shape == (4, 4):
        omega = vee(X[:3, :3])
        v = X[:3, 3].reshape(3, 1)
        out = np.vstack([omega, v])
    else:
        raise ValueError("Input must be 3x3 or 4x4.")

    if np.iscomplexobj(out):
        out = out.astype(float)
    return out


def hat(x):
    x = np.asarray(x).reshape(-1)
    if x.size != 3:
        raise ValueError("Input must be a 3-vector.")
    out = np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])
    if np.iscomplexobj(out):
        out = out.astype(float)
    return out


def ad(V):
    V = np.asarray(V).reshape(-1)
    if V.size != 6:
        raise ValueError("Input must be a 6-vector [omega; v].")
    omega = V[:3]
    v = V[3:6]
    ad_V = np.zeros((6, 6))
    ad_V[:3, :3] = hat(omega)
    ad_V[3:6, :3] = hat(v)
    ad_V[3:6, 3:6] = hat(omega)
    if np.iscomplexobj(ad_V):
        ad_V = ad_V.astype(float)
    return ad_V


def Ad(H):
    if H.shape != (4, 4):
        raise ValueError("Input must be a 4x4 homogeneous transform.")
    Ad_H = np.zeros((6, 6))
    R = H[:3, :3]
    p = H[:3, 3].reshape(3, 1)

    Ad_H[:3, :3] = R
    Ad_H[3:6, :3] = hat(p) @ R
    Ad_H[3:6, 3:6] = R

    if np.iscomplexobj(Ad_H):
        Ad_H = Ad_H.astype(float)
    return Ad_H


def Ad_inv(H):
    if H.shape != (4, 4):
        raise ValueError("Input must be a 4x4 homogeneous transform.")
    Ad_inv_H = np.zeros((6, 6))
    R = H[:3, :3]
    p = H[:3, 3].reshape(3, 1)

    Ad_inv_H[:3, :3] = R.T
    Ad_inv_H[3:6, :3] = -R.T @ hat(p)
    Ad_inv_H[3:6, 3:6] = R.T

    if np.iscomplexobj(Ad_inv_H):
        Ad_inv_H = Ad_inv_H.astype(float)
    return Ad_inv_H


def get_generalized_inertia(m, I, cog):
    Ixx, Iyy, Izz, Ixy, Ixz, Iyz = I  # unpack for clarity
    I_mat = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])

    G = np.zeros((6, 6))
    G[:3, :3] = I_mat
    G[3:6, 3:6] = m * np.eye(3)
    G[3:6, :3] = -m * hat(cog)
    G[:3, 3:6] = m * hat(cog)

    # NOTE: In this workspace, the inertia parameters `I` are defined about the body
    # frame origin (not about the center of mass). Do not apply an additional
    # parallel-axis shift here; the CoG coupling terms already capture the spatial
    # inertia structure used by the controller.

    if np.iscomplexobj(G):
        G = G.astype(float)
    return G
