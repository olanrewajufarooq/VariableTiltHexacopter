import numpy as np

def vee(X):
    if X.shape == (3, 3):
        return np.array([X[2,1], X[0,2], X[1,0]]).reshape(3,1)
    elif X.shape == (4, 4):
        omega = vee(X[:3, :3])
        v = X[:3, 3].reshape(3,1)
        return np.vstack([omega, v])
    else:
        raise ValueError("Input must be 3x3 or 4x4.")

def hat(x):
    x = x.flatten()
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])

def ad(V):
    omega = V[:3]
    v = V[3:6]
    ad_V = np.zeros((6,6))
    ad_V[:3, :3] = hat(omega)
    ad_V[3:6, :3] = hat(v)
    ad_V[3:6, 3:6] = hat(omega)
    return ad_V

def Ad(H):
    Ad_H = np.zeros((6, 6))
    R = H[:3, :3]
    p = H[:3, 3].reshape(3, 1)
    
    Ad_H[:3, :3] = R
    Ad_H[3:6, :3] = hat(p) @ R
    Ad_H[3:6, 3:6] = R

    return Ad_H

def Ad_inv(H):
    Ad_inv_H = np.zeros((6, 6))
    R = H[:3, :3]
    p = H[:3, 3].reshape(3, 1)
    
    Ad_inv_H[:3, :3] = R.T
    Ad_inv_H[3:6, :3] = -R.T @ hat(p)
    Ad_inv_H[3:6, 3:6] = R.T

    return Ad_inv_H

def get_generalized_inertia(m, I, cog):

    Ixx, Iyy, Izz, Ixy, Ixz, Iyz = I  # unpack for clarity
    I_mat = np.array([
        [Ixx, Ixy, Ixz],
        [Ixy, Iyy, Iyz],
        [Ixz, Iyz, Izz]
    ])

    G = np.zeros((6, 6))
    G[:3, :3] = I_mat
    G[3:6, 3:6] = m * np.eye(3)
    G[3:6, :3] = -m * hat(cog)
    G[:3, 3:6] = m * hat(cog)

    return G