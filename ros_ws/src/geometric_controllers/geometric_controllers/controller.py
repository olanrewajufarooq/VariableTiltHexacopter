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
        self.V_err = None

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
        self.V_err = V - Ad_inv(self.H_err) @ V_des
        return -self.Kd @ self.V_err

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
# (Prev) Adaptive Controller
# ─────────────────────────────────────────────
class PrevAdaptiveController(BaseController):
    def __init__(self, Kp_att, Kp_pos, Kd, CoG, m, I, gamma=0.01, pot_type='liealgebra'):
        super().__init__(Kp_att, Kp_pos, Kd, m, CoG, pot_type=pot_type)
        self.I = get_generalized_inertia(m=m, I=I, cog=self.CoG)
        
        self.nominal_mass = m
        self.estimated_unknown_mass = 0.0
        self.gamma = gamma

    def adaptation_law(self, H, V, dt):
        H[:3, 3] = np.zeros((3,))

        g = np.vstack((np.zeros((3,1)), np.array([0, 0, -self.gravity]).reshape(3,1)))
        self.estimated_unknown_mass += float(self.gamma * V.T @ Ad(H).T @ g * dt)

        self.mass = self.nominal_mass + self.estimated_unknown_mass
        self.mass = max(self.mass, self.nominal_mass)  # Prevent negative mass

    def compute_wrench(self, H_des, H, V_des, V, dt):
        self.adaptation_law(H, V, dt)
        Wp = self.get_Wp(H_des, H)
        Wd = self.get_Wd(V_des, V)
        W = Wp + Wd
        return W
    
# ─────────────────────────────────────────────
# Adaptive Controller
# ─────────────────────────────────────────────
class AdaptiveController(BaseController):
    r"""
    Online estimation of
        θ = [Ixx, Iyy, Izz, Ixy, Iyz, Ixz, m, m·ξx, m·ξy, m·ξz]ᵀ.

    The basis matrices 𝓘_i and 𝓖_i reproduce (54)–(55) of the paper,
    and the update law ẋ̂ = Γ Yᵀ V_e dt enforces ḊV ≤0.
    """
    # ------------------------------------------------------------------
    def __init__(self, Kp_att, Kp_pos, Kd, CoG, m, I,
                 gamma=None, pot_type='liealgebra'):
        super().__init__(Kp_att, Kp_pos, Kd, m, CoG, pot_type=pot_type)

        # initial parameter estimates -------------------------------------------------
        Ixx, Iyy, Izz, Ixy, Ixz, Iyz = I
        print(f"Initial inertia: {Ixx}, {Iyy}, {Izz}, {Ixy}, {Ixz}, {Iyz}")
        self.theta_hat = np.array(
            [Ixx, Iyy, Izz, Ixy, Iyz, Ixz,
             m,
             m * CoG[0], m * CoG[1], m * CoG[2]],
            dtype=float).reshape(10, 1)

        if gamma is None:
            self.gamma = 1e-3 * np.diag([1, 1, 1, 1, 1, 1, 3, 0.1, 0.1, 0.5])
        elif isinstance(gamma, (int, float)):
            self.gamma = gamma * np.eye(10)
        elif isinstance(gamma, (list, np.ndarray)) and len(gamma) == 10:
            self.gamma = np.diag(gamma)
        else:
            raise ValueError("Gamma must be a scalar or a list/array of length 10.")
        self.gamma = self.gamma.astype(float)

        self._construct_bases()

    # ------------------------------------------------------------------
    #   Basis matrices  𝓘_i  (inertia)  and  𝓖_i  (gravity)
    # ------------------------------------------------------------------
    def _construct_bases(self):
        """Pre-compute 6×6 generators 𝓘_i (i=1…10) and 𝓖_i (i=7…10)."""
        def hat(v):
            return np.array([[0., -v[2],  v[1]],
                             [v[2],  0., -v[0]],
                             [-v[1], v[0], 0.]])

        # --- inertia generators ------------------------------------------------------
        self.I_basis = []

        # i = 1 … 3  → diagonal moments
        for k in range(3):
            E = np.zeros((3, 3));  E[k, k] = 1.
            Gi = np.zeros((6, 6)); Gi[:3, :3] = E
            self.I_basis.append(Gi)

        # i = 4 … 6  → off-diagonal products (xy, yz, xz)  — symmetric
        for (i, j) in [(0, 1), (1, 2), (0, 2)]:
            E = np.zeros((3, 3));  E[i, j] = E[j, i] = 1.
            Gi = np.zeros((6, 6)); Gi[:3, :3] = E
            self.I_basis.append(Gi)

        # i = 7  → pure mass in translational block
        Gi = np.zeros((6, 6)); Gi[3:, 3:] = np.eye(3)
        self.I_basis.append(Gi)

        # i = 8 … 10  → m ξ cross-terms:  tilde(e_i) in upper-right *and* lower-left
        for ax in range(3):
            e = np.eye(3)[ax]
            S = hat(e)
            Gi = np.zeros((6, 6))
            Gi[:3, 3:] = S
            Gi[3:, :3] = S
            self.I_basis.append(Gi)

        # --- gravity generators 𝓖_i (only 7…10) ------------------------------------
        self.G_basis = []
        # i = 7
        G7 = np.zeros((6, 6)); G7[3:, 3:] = np.eye(3)
        self.G_basis.append(G7)
        # i = 8…10
        for ax in range(3):
            e = np.eye(3)[ax]
            S = hat(e)
            Gi = np.zeros((6, 6)); Gi[:3, :3] = S
            self.G_basis.append(Gi)

    # ------------------------------------------------------------------
    #   Helper: current estimated inertia, mass and CoG
    # ------------------------------------------------------------------
    def _unpack_theta(self):
        Ixx, Iyy, Izz, Ixy, Iyz, Ixz, m, mCx, mCy, mCz = self.theta_hat.flatten()
        # I_est = np.array([[Ixx, Ixy, Ixz],
        #                   [Ixy, Iyy, Iyz],
        #                   [Ixz, Iyz, Izz]])
        I_est = Ixx, Iyy, Izz, Ixy, Iyz, Ixz
        CoG_est = np.array([mCx, mCy, mCz]) / max(m, 1e-9)
        return m, I_est, CoG_est

    def _generalized_inertia(self):
        m, I6, cog = self._unpack_theta()
        return get_generalized_inertia(m=m, I=I6, cog=cog)

    def _gravity_wrench_hat(self, H):
        """Estimated gravity wrench in body frame, using θ̂."""
        m, _, cog = self._unpack_theta()
        Hg = H.copy()
        Hg[:3, 3] = cog
        g_world = np.vstack((np.zeros((3, 1)),
                             np.array([0, 0, -m * self.gravity]).reshape(3, 1)))
        return Ad(Hg).T @ g_world

    # ------------------------------------------------------------------
    #   Regression matrix  Y(H,V,V_d,a_d)   (Eq. (54))
    # ------------------------------------------------------------------
    def _regressor(self, H_err, H, V, V_d, a_d):
        """Return 6×10 matrix Y."""
        Ad_inv_err = Ad_inv(H_err)
        V_e = V - Ad_inv_err @ V_d
        a_bar = a_d - ad(V_d) @ V_e

        # common gravity vector  [R g; R g]
        R = H[:3, :3]
        g_body = R @ np.array([0, 0, -self.gravity]).reshape(3, 1)
        gvec = np.vstack((g_body, g_body))        # (6×1)

        cols = []
        for i in range(10):
            term = -ad(V).T @ self.I_basis[i] @ V \
                   - self.I_basis[i] @ Ad_inv_err @ a_bar
            if i >= 6:       # i = 7…10 ⇒ add gravity contribution
                Gi = self.G_basis[i - 6]
                term += Gi @ gvec
            cols.append(term)

        Y = np.hstack(cols)  # 6×10 matrix
        assert Y.shape == (6, 10), "Y must be a 6x10 matrix"

        return Y

    # ------------------------------------------------------------------
    #   Adaptation law   θ̂ ← θ̂ + Γ Yᵀ V_e dt
    # ------------------------------------------------------------------
    def _adapt(self, Y, V_e, dt):
        self.theta_hat += self.gamma @ (Y.T @ V_e) * dt

    # ------------------------------------------------------------------
    #   Public API
    # ------------------------------------------------------------------
    def compute_wrench(self, H_des, H, V_des, V,
                       A_des=None, dt=1e-3):
        if A_des is None:
            A_des = np.zeros((6, 1))

        # pose and twist errors -------------------------------------------------------
        H_err = np.linalg.inv(H_des) @ H
        Ad_inv_err = Ad_inv(H_err)
        V_e = V - Ad_inv_err @ V_des

        # regression matrix and parameter update -------------------------------------
        Y = self._regressor(H_err, H, V, V_des, A_des)
        self._adapt(Y, V_e, dt)

        # estimated dynamic quantities -----------------------------------------------
        m, I6, cog = self._unpack_theta()
        I_hat = self._generalized_inertia()
        Wg_hat = self._gravity_wrench_hat(H)

        # control terms ---------------------------------------------------------------
        self.CoG = cog.reshape(3, 1)  # update CoG in BaseController
        self.mass = m  # update mass in BaseController

        Wp = self.get_Wp(H_des, H)          # potential + gravity compensation
        Wd = self.get_Wd(V_des, V)          # damping
        coriolis   = ad(V).T @ I_hat @ V
        feed_forwd = I_hat @ Ad_inv_err @ (A_des - ad(V_des) @ V_e)

        return coriolis + feed_forwd + Wp + Wd
        # return coriolis + feed_forwd + Wp + Wd - Wg_hat

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
