import numpy as np
from abc import ABC, abstractmethod

# ─────────────────────────────────────────────
# Utility functions
# ─────────────────────────────────────────────
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

# ─────────────────────────────────────────────
# Base Abstract Controller
# ─────────────────────────────────────────────
class BaseController(ABC):
    def __init__(self, Kp_att, Kp_pos, Kd, m, CoG = [0, 0, 0], g=9.8):
        self.Kp_att = np.diag(Kp_att)
        self.Kp_pos = np.diag(Kp_pos)
        self.Kd = np.diag(Kd)

        self.mass = m
        self.gravity = g

        self.CoG = np.array(CoG).reshape(3, 1)

    def get_gravity_wrench(self, H):

        g_world = np.vstack((np.zeros((3,1)), np.array([0, 0, -self.mass*self.gravity]).reshape(3,1)))

        g = Ad(H).T @ g_world

        return g

    def get_Wp(self, H_des, H):
        R = H[:3,:3]
        p = H[:3,3].reshape(3,1)

        R_des = H_des[:3,:3]
        p_des = H_des[:3,3].reshape(3,1)

        e_p = p - p_des
        R_err = R_des.T @ R

        F = self.Kp_pos @ e_p
        T = vee(0.5 * ( self.Kp_att @ R_err - (self.Kp_att @ R_err).T ))

        H_grav = np.zeros((4, 4))
        H_grav[:3, :3] = R
        H_grav[:3, 3] = self.CoG.flatten()

        return -np.vstack((T, F)) - self.get_gravity_wrench(H_grav)

    def get_Wd(self, V_des, V, H):
        e_v = V - Ad(H) @ V_des
        return -self.Kd @ e_v

    @abstractmethod
    def compute_wrench(self, H_des, H, V_des, V):
        pass

# ─────────────────────────────────────────────
# PD Controller
# ─────────────────────────────────────────────
class PDController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, m, CoG, I=None):
        super().__init__(Kp_att, Kp_pos, Kd, m, CoG)
    
    def compute_wrench(self, H_des, H, V_des, V):
        return self.get_Wp(H_des, H) + self.get_Wd(V_des, V, H)

# ─────────────────────────────────────────────
# Feedback Linearized Controller
# ─────────────────────────────────────────────
class FeedbackLinearizedController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, CoG, m, I):
        super().__init__(Kp_att, Kp_pos, Kd, m, CoG)
        self.I = get_generalized_inertia(m=m, I=I, cog=self.CoG)

    def compute_wrench(self, H_des, H, V_des, V):
        Wp = self.get_Wp(H_des, H)
        Wd = self.get_Wd(V_des, V, H)
        return -ad(V).T @ self.I @ V + (Wp + Wd)

# ─────────────────────────────────────────────
# Adaptive Controller
# ─────────────────────────────────────────────
class AdaptiveController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, CoG, m, I):
        super().__init__(Kp_att, Kp_pos, Kd, m, CoG)
        self.I = get_generalized_inertia(m=m, I=I, cog=self.CoG)

    def adaptation_law(self, H_des, H, V_des, V):
        raise NotImplementedError()

    def compute_wrench(self, H_des, H, V_des, V):
        Wp = self.get_Wp(H_des, H)
        Wd = self.get_Wd(V_des, V, H)
        return -ad(V).T @ self.I @ V + (Wp + Wd)

# ─────────────────────────────────────────────
# Single Controller Interface (Factory)
# ─────────────────────────────────────────────
class Controller:
    def __init__(self, method, Kp_att, Kp_pos, Kd, m, CoG=[0, 0, 0], I=None):
        if method == 'PD':
            self.controller = PDController(Kp_att=Kp_att, Kp_pos=Kp_pos, Kd=Kd, m=m, CoG=CoG)
        elif method == 'FeedLin':
            self.controller = FeedbackLinearizedController(Kp_att=Kp_att, Kp_pos=Kp_pos, Kd=Kd, CoG=CoG, m=m, I=I)
        elif method == 'Adaptive':
            self.controller = AdaptiveController(Kp_att=Kp_att, Kp_pos=Kp_pos, Kd=Kd, CoG=CoG, m=m, I=I)
        else:
            raise ValueError(f"Unknown controller method: {method}")

    def compute_wrench(self, H_des, H, V_des, V):
        return self.controller.compute_wrench(H_des, H, V_des, V)
