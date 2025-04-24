import numpy as np

class Controller:
    def __init__(self, Gp, Kxi, Kd):
        self.Gp = np.diag(Gp)       # Gain matrix for orientation (R3x3)
        self.Kxi = np.diag(Kxi)     # Gain matrix for position (R3x3)
        self.Kd = np.diag(Kd)       # Gain matrix for velocity (R6x6)

    def get_gravity_wrench(self, m, g, R):
        g_world = np.array([0, 0, -m * g]).reshape(3, 1)        # World Frame
        g_body = R.T @ g_world                                  # Body Frame

        return np.concatenate(( np.zeros((3, 1)), g_body ), axis=0)
    
    def get_Wp(self, m, g, H_des, H, V_des, V):
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
        F = self.Kxi @ e_p

        # Compute torque
        T = self.vee( 0.5 * self.Gp @ (R_err - R_err.T) )

        # Calculate the desired wrench
        return -np.concatenate((T, F), axis=0) - self.get_gravity_wrench(m, g, R)
    
    def get_Wd(self, V_des, V):
        e_v = V_des - V     # Error in velocity

        return -self.Kd @ e_v
    
    def compute_wrench(self, m, g, H_des, H, V_des, V):

        assert H.shape == (4, 4), "H must be a 4x4 transformation matrix"
        assert H_des.shape == (4, 4), "H_des must be a 4x4 transformation matrix"
        assert V.shape == (6, 1), "V must be a 6x1 twist"

        Wp = self.get_Wp(m, g, H_des, H, V_des, V)
        Wd = self.get_Wd(V_des, V)
        
        return Wp + Wd
    
    # Utility functions
    def vee(self, R):
        if R.shape != (3, 3):
            raise ValueError("Input must be a 3x3 matrix.")
        if not np.allclose(R + R.T, 0, atol=1e-8):
            raise ValueError("Input matrix must be a skew-symmetric matrix.")
        return np.array([R[2, 1], R[0, 2], R[1, 0]]).reshape(3, 1)