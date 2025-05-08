import numpy as np
from abc import ABC, abstractmethod
from geometric_controllers.utils import vee, hat, ad, Ad, Ad_inv, get_generalized_inertia
from scipy.linalg import expm, logm

# ─────────────────────────────────────────────
# Base Abstract Controller
# ─────────────────────────────────────────────
class BaseController(ABC):
    def __init__(self, Kp_att, Kp_pos, Kd, m, CoG = [0, 0, 0], g=9.8, pot_type='liealgebra'):
        self.pot_type = pot_type # 'liealgebra' or 'separate'

        self.Kp_att = np.diag(Kp_att)
        self.Kp_pos = np.diag(Kp_pos)
        self.Kd = np.diag(Kd)
        self.Kp = np.diag([*Kp_att, *Kp_pos])

        self.mass = m
        self.gravity = g

        self.CoG = np.array(CoG).reshape(3, 1)
        self.H_err = None

    def get_gravity_wrench(self, H):

        g_world = np.vstack((np.zeros((3,1)), np.array([0, 0, -self.mass*self.gravity]).reshape(3,1)))
        g = Ad(H).T @ g_world

        return g

    def get_Wp(self, H_des, H):
        self.H_err = np.linalg.inv(H_des) @ H

        if self.pot_type == 'liealgebra':
            e_p = vee(logm(self.H_err))
            Wp = self.Kp @ e_p
            
        elif self.pot_type == 'separate':
            R = H[:3,:3]
            p = H[:3,3].reshape(3,1)

            R_des = H_des[:3,:3]
            p_des = H_des[:3,3].reshape(3,1)

            e_p = p - p_des
            R_err = R_des.T @ R

            F = self.Kp_pos @ e_p
            T = vee(0.5 * ( self.Kp_att @ R_err - (self.Kp_att @ R_err).T ))

            Wp = np.vstack((T, F)).reshape(6, 1)
        
        H_grav = H.copy()
        H_grav[:3, 3] = self.CoG.flatten()

        return -Wp - self.get_gravity_wrench(H_grav)

    def get_Wd(self, V_des, V):
        e_v = V - Ad_inv(self.H_err) @ V_des
        return -self.Kd @ e_v

    @abstractmethod
    def compute_wrench(self, H_des, H, V_des, V, dt=None):
        pass

# ─────────────────────────────────────────────
# PD Controller
# ─────────────────────────────────────────────
class PDController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, m, CoG, I=None, pot_type='liealgebra'):
        super().__init__(Kp_att, Kp_pos, Kd, m, CoG, pot_type=pot_type)
    
    def compute_wrench(self, H_des, H, V_des, V):
        return self.get_Wp(H_des, H) + self.get_Wd(V_des, V)

# ─────────────────────────────────────────────
# Feedback Linearized Controller
# ─────────────────────────────────────────────
class FeedbackLinearizedController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, CoG, m, I, pot_type='liealgebra'):
        super().__init__(Kp_att, Kp_pos, Kd, m, CoG, pot_type=pot_type)
        self.I = get_generalized_inertia(m=m, I=I, cog=self.CoG)

    def compute_wrench(self, H_des, H, V_des, V):
        Wp = self.get_Wp(H_des, H)
        Wd = self.get_Wd(V_des, V)
        return ad(V).T @ self.I @ V + (Wp + Wd)

# ────────────────────────────────────────────
# Feedforward Compensation Controller
# ─────────────────────────────────────────────

class FeedforwardCompensationController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, CoG, m, I, pot_type='liealgebra'):
        super().__init__(Kp_att, Kp_pos, Kd, m, CoG, pot_type=pot_type)
        self.I = get_generalized_inertia(m=m, I=I, cog=self.CoG)

    def compute_wrench(self, H_des, H, V_des, V, A_des = np.zeros((6, 1)) ):
        # Error terms
        self.H_err = np.linalg.inv(H_des) @ H
        Ad_inv_ = Ad_inv(self.H_err)
        V_e = V - Ad_inv_ @ V_des

        # Feedforward term (transformed desired acceleration)
        feedforward_term = self.I @ Ad_inv_ @ (A_des - ad(V_des) @ V_e)

        # Coriolis compensation
        coriolis_term = ad(V).T @ self.I @ V

        # Proportional and Derivative terms
        Wp = self.get_Wp(H_des, H)
        Wd = self.get_Wd(V_des, V)

        # Total control wrench
        return coriolis_term + feedforward_term + Wp + Wd
    

# ─────────────────────────────────────────────
# Adaptive Controller
# ─────────────────────────────────────────────
class AdaptiveController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, CoG, m, I, gamma=0.01, pot_type='liealgebra'):
        super().__init__(Kp_att, Kp_pos, Kd, m, CoG, pot_type=pot_type)
        self.I = get_generalized_inertia(m=m, I=I, cog=self.CoG)
        
        self.nominal_mass = m
        self.estimated_unknown_mass = 0.0
        self.gamma = gamma

    def adaptation_law(self, H, V, dt):
        H[:3, 3] = np.zeros((3,))

        g = np.vstack((np.zeros((3,1)), np.array([0, 0, -self.gravity]).reshape(3,1)))
        self.estimated_unknown_mass += np.float(self.gamma * V.T @ Ad(H).T @ g * dt)

        self.mass = self.nominal_mass + self.estimated_unknown_mass
        self.mass = max(self.mass, self.nominal_mass)  # Prevent negative mass

    def compute_wrench(self, H_des, H, V_des, V, dt):
        self.adaptation_law(H, V, dt)
        Wp = self.get_Wp(H_des, H)
        Wd = self.get_Wd(V_des, V)
        W = Wp + Wd
        return W

# ─────────────────────────────────────────────
# Single Controller Interface (Factory)
# ─────────────────────────────────────────────
class Controller:
    def __init__(self, method, Kp_att, Kp_pos, Kd, m, CoG=[0, 0, 0], I=None, pot_type='liealgebra'):
        self.method = method

        if method == 'PD':
            self.controller = PDController(Kp_att=Kp_att, Kp_pos=Kp_pos, Kd=Kd, m=m, CoG=CoG, pot_type=pot_type)
        elif method == 'FeedLin':
            self.controller = FeedbackLinearizedController(Kp_att=Kp_att, Kp_pos=Kp_pos, Kd=Kd, CoG=CoG, m=m, I=I, pot_type=pot_type)
        elif method == 'FeedForward':
            self.controller = FeedforwardCompensationController(Kp_att=Kp_att, Kp_pos=Kp_pos, Kd=Kd, CoG=CoG, m=m, I=I, pot_type=pot_type)
        elif method == 'Adaptive':
            self.controller = AdaptiveController(Kp_att=Kp_att, Kp_pos=Kp_pos, Kd=Kd, CoG=CoG, m=m, I=I, pot_type=pot_type)
        else:
            raise ValueError(f"Unknown controller method: {method}")

    def compute_wrench(self, H_des, H, V_des, V, A_des = None, dt=None):
        if self.method == 'PD':
            return self.controller.compute_wrench(H_des=H_des, H=H, V_des=V_des, V=V)
        elif self.method == 'FeedLin':
            return self.controller.compute_wrench(H_des=H_des, H=H, V_des=V_des, V=V)
        if self.method == 'FeedForward':
            if A_des is None:
                A_des = np.zeros((6, 1))
            return self.controller.compute_wrench(H_des=H_des, H=H, V_des=V_des, V=V, A_des=A_des)
        elif self.method == 'Adaptive':
            return self.controller.compute_wrench(H_des=H_des, H=H, V_des=V_des, V=V, dt=dt)
    
    def __getattr__(self, name):
        """Forward undefined attributes to the wrapped controller"""
        return getattr(self.controller, name)
