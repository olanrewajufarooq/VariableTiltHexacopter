import numpy as np

class Controller:
    def __init__(self, Kp_att, Kp_pos, Kd):
        self.Kp_att = np.diag(Kp_att)       # Gain matrix for orientation (R3x3)
        self.Kp_pos = np.diag(Kp_pos)     # Gain matrix for position (R3x3)
        self.Kd = np.diag(Kd)       # Gain matrix for velocity (R6x6)

    def get_gravity_wrench(self, m, g, R):
        g_world = np.array([0, 0, -m * g]).reshape(3, 1)        # World Frame
        g_body = R.T @ g_world                                  # Body Frame

        return np.concatenate(( np.zeros((3, 1)), g_body ), axis=0)
    
    def get_Wp(self, m, g, H_des, H):
        # Extract rotation matrix and position vector from H
        R = H[:3, :3]
        p = H[:3, 3].reshape(3, 1)

        # Extract desired rotation matrix and position vector from H_des
        R_des = H_des[:3, :3]
        p_des = H_des[:3, 3].reshape(3, 1)

        # Calculate the error in position
        e_p = p - p_des

        # Calculate the error in orientation
        R_err = R_des.T @ R

        # Compute force
        F = self.Kp_pos @ e_p

        # Compute torque
        T = self.vee( 0.5 * self.Kp_att @ (R_err - R_err.T) )

        # Calculate the desired wrench
        return -np.concatenate((T, F), axis=0) - self.get_gravity_wrench(m, g, R)
    
    def get_Wd(self, V_des, V):
        e_v = V - V_des     # Error in velocity

        return -self.Kd @ e_v
    
    def compute_wrench(self, m, g, H_des, H, V_des, V):
        raise NotImplementedError("This method should be implemented in subclasses.")
    
    # Utility functions
    def vee(self, X):
        if X.shape == (3, 3):  # SO(3)
            if not np.allclose(X + X.T, 0, atol=1e-8):
                raise ValueError("Input matrix must be skew-symmetric for SO(3) or 3x3 Matrices.")
            return np.array([X[2, 1], X[0, 2], X[1, 0]]).reshape(3, 1)

        elif X.shape == (4, 4):  # SE(3)
            if not np.allclose(X[:3, :3] + X[:3, :3].T, 0, atol=1e-8):
                raise ValueError("Top-left 3x3 block must be skew-symmetric for SE(3).")
            if not np.allclose(X[3, :], 0, atol=1e-8):
                raise ValueError("Bottom row must be zeros for SE(3).")
            
            omega = self.vee(X[:3, :3])  # Extract angular part (SO(3) vee)
            v = X[:3, 3].reshape(3, 1)   # Extract linear part
            return np.vstack([omega, v])  # Combine into [ω; v]

        else:
            raise ValueError("Input must be either 3x3 (for SO(3) or 3x3 Matrices) or 4x4 (for SE(3)).")
    
    def hat(self, X):
        if X.shape == (3, 1): # SO(3)
            X = np.asarray(X).flatten()
            return np.array([[0, -X[2], X[1]],
                             [X[2], 0, -X[0]],
                             [-X[1], X[0], 0]])
        
        elif X.shape == (6, 1): # SE(3)
            X_hat = np.zeros((4, 4))
            X_hat[:3, :3] = self.hat(X[:3])
            X_hat[:3, 3] = X[3:6].reshape(3,)
            
            return X_hat
            
    def Ad(self, H):
        if H.shape != (4, 4):
            raise ValueError("Input must be a 4x4 matrix.")
        R = H[:3, :3]
        p = H[:3, 3].reshape(3, 1)
        Ad_H = np.zeros((6, 6))
        Ad_H[:3, :3] = R
        Ad_H[3:6, 3:6] = R
        Ad_H[3:6, :3] = self.hat(p) @ R

        return Ad_H
    
    def ad(self, V):
        if V.shape != (6, 1):
            raise ValueError("Input must be a 6x1 vector.")
        
        omega = V[:3]
        v = V[3:6]

        ad_V = np.zeros((6, 6))
        ad_V[:3, :3] = self.hat(omega)
        ad_V[3:6, 3:6] = self.hat(omega)
        ad_V[3:6, :3] = self.hat(v)

        return ad_V
        
class PDController(Controller):
    def __init__(self, Gp, Kxi, Kd):
        super().__init__(Gp, Kxi, Kd)

    def compute_wrench(self, m, g, H_des, H, V_des, V):

        assert H.shape == (4, 4), "H must be a 4x4 transformation matrix"
        assert H_des.shape == (4, 4), "H_des must be a 4x4 transformation matrix"
        assert V.shape == (6, 1), "V must be a 6x1 twist"

        Wp = self.get_Wp(m, g, H_des, H)
        Wd = self.get_Wd(V_des, V)
        
        return Wp + Wd

class FeedbackLinearizedController(Controller):
    def __init__(self, Gp, Kxi, Kd, I):
        super().__init__(Gp, Kxi, Kd)
        self.I = np.diag(I)       # Inertia matrix (R6x6)
        self.I_inv = np.linalg.inv(self.I)
    
    def compute_wrench(self, m, g, H_des, H, V_des, V):

        assert H.shape == (4, 4), "H must be a 4x4 transformation matrix"
        assert H_des.shape == (4, 4), "H_des must be a 4x4 transformation matrix"
        assert V.shape == (6, 1), "V must be a 6x1 twist"

        Wp = self.get_Wp(m, g, H_des, H)
        Wd = self.get_Wd(V_des, V)

        Wpd = Wp + Wd

        return  -self.ad(V).T @ self.I @ V + Wpd
    
class AdaptiveController(Controller):
    def __init__(self, Gp, Kxi, Kd, I):
        super().__init__(Gp, Kxi, Kd)
        self.I = np.diag(I)       # Inertia matrix (R6x6)
        self.I_inv = np.linalg.inv(self.I)

    def adaptation_law(self, m, g, H_des, H, V_des, V):
        raise NotImplementedError("Adaptation law not implemented yet.")
    
    def compute_wrench(self, m, g, H_des, H, V_des, V):

        assert H.shape == (4, 4), "H must be a 4x4 transformation matrix"
        assert H_des.shape == (4, 4), "H_des must be a 4x4 transformation matrix"
        assert V.shape == (6, 1), "V must be a 6x1 twist"

        Wp = self.get_Wp(m, g, H_des, H)
        Wd = self.get_Wd(V_des, V)

        Wpd = Wp + Wd

        return -self.ad(V).T @ self.I @ V + Wpd