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

# ─────────────────────────────────────────────
# Base Abstract Controller
# ─────────────────────────────────────────────
class BaseController(ABC):
    def __init__(self, Kp_att, Kp_pos, Kd):
        self.Kp_att = np.diag(Kp_att)
        self.Kp_pos = np.diag(Kp_pos)
        self.Kd = np.diag(Kd)

    def get_gravity_wrench(self, m, g, R):
        g_world = np.array([0, 0, -m*g]).reshape(3,1)
        g_body = R.T @ g_world
        return np.vstack((np.zeros((3,1)), g_body))

    def get_Wp(self, m, g, H_des, H):
        R = H[:3,:3]
        p = H[:3,3].reshape(3,1)

        R_des = H_des[:3,:3]
        p_des = H_des[:3,3].reshape(3,1)

        e_p = p - p_des
        R_err = R_des.T @ R

        F = self.Kp_pos @ e_p
        T = vee(0.5 * ( self.Kp_att @ R_err - (self.Kp_att @ R_err).T ))

        return -np.vstack((T, F)) - self.get_gravity_wrench(m, g, R)

    def get_Wd(self, V_des, V):
        e_v = V - V_des
        return -self.Kd @ e_v

    @abstractmethod
    def compute_wrench(self, m, g, H_des, H, V_des, V):
        pass

# ─────────────────────────────────────────────
# PD Controller
# ─────────────────────────────────────────────
class PDController(BaseController):
    def compute_wrench(self, m, g, H_des, H, V_des, V):
        return self.get_Wp(m, g, H_des, H) + self.get_Wd(V_des, V)

# ─────────────────────────────────────────────
# Feedback Linearized Controller
# ─────────────────────────────────────────────
class FeedbackLinearizedController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, I):
        super().__init__(Kp_att, Kp_pos, Kd)
        self.I = np.diag(I)

    def compute_wrench(self, m, g, H_des, H, V_des, V):
        Wp = self.get_Wp(m, g, H_des, H)
        Wd = self.get_Wd(V_des, V)
        return -ad(V).T @ self.I @ V + (Wp + Wd)

# ─────────────────────────────────────────────
# Adaptive Controller
# ─────────────────────────────────────────────
class AdaptiveController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, I):
        super().__init__(Kp_att, Kp_pos, Kd)
        self.I = np.diag(I)

    def adaptation_law(self, m, g, H_des, H, V_des, V):
        raise NotImplementedError()

    def compute_wrench(self, m, g, H_des, H, V_des, V):
        Wp = self.get_Wp(m, g, H_des, H)
        Wd = self.get_Wd(V_des, V)
        return -ad(V).T @ self.I @ V + (Wp + Wd)

# ─────────────────────────────────────────────
# Single Controller Interface (Factory)
# ─────────────────────────────────────────────
class Controller:
    def __init__(self, method, Kp_att, Kp_pos, Kd, I=None):
        if method == 'PD':
            self.controller = PDController(Kp_att, Kp_pos, Kd)
        elif method == 'FeedLin':
            self.controller = FeedbackLinearizedController(Kp_att, Kp_pos, Kd, I)
        elif method == 'Adaptive':
            self.controller = AdaptiveController(Kp_att, Kp_pos, Kd, I)
        else:
            raise ValueError(f"Unknown controller method: {method}")

    def compute_wrench(self, m, g, H_des, H, V_des, V):
        return self.controller.compute_wrench(m, g, H_des, H, V_des, V)
