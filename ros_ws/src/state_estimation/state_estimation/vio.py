import numpy as np
import gtsam
from gtsam.symbol_shorthand import X, V, B
from state_estimation.vo import VisualOdometry
import open3d as o3d

class VisualInertiaOdometry:
    """
    Tightly-coupled VIO: Open3D RGB-D front-end + GTSAM CombinedImuFactor.
    """

    def __init__(self,
                 intrinsic: o3d.camera.PinholeCameraIntrinsic,
                 gravity: float = 9.81,
                 accel_noise: float = 0.021,
                 gyro_noise: float = 0.009,
                 integ_noise: float = 0.0001,
                 bias_rw_noise_acc: float = 0.000375,
                 bias_rw_noise_gyro: float = 0.00002,
                 pose_prior_sigmas: np.ndarray = None,
                 vel_prior_sigma: float = 0.1,
                 bias_prior_sigma: float = 0.005):
        
        # Visual front-end
        self.vo = VisualOdometry(intrinsic)

        # IMU preintegration params
        params = gtsam.PreintegrationCombinedParams.MakeSharedU(gravity)
        params.setAccelerometerCovariance(np.eye(3)*(accel_noise**2))
        params.setGyroscopeCovariance(np.eye(3)*(gyro_noise**2))
        params.setIntegrationCovariance(np.eye(3)*(integ_noise**2))
        params.setBiasAccCovariance(np.eye(3)*(bias_rw_noise_acc**2))
        params.setBiasOmegaCovariance(np.eye(3)*(bias_rw_noise_gyro**2))

        self.preint = gtsam.PreintegratedCombinedMeasurements(
            params, gtsam.imuBias.ConstantBias()
        )

        # Graph & values
        self.graph  = gtsam.NonlinearFactorGraph()
        self.values = gtsam.Values()

        # Noise models
        if pose_prior_sigmas is None:
            pose_prior_sigmas = np.array([0.1]*6)
        self.pose_prior = gtsam.noiseModel.Diagonal.Sigmas(pose_prior_sigmas)
        self.vel_prior  = gtsam.noiseModel.Isotropic.Sigma(3, vel_prior_sigma)
        # self.bias_prior = gtsam.noiseModel.Isotropic.Sigma(6, bias_prior_sigma)
        self.bias_prior = gtsam.noiseModel.Diagonal.Sigmas(
            [0.000375]*3 + [0.00002]*3
        )

        # iSAM2
        self.isam = gtsam.ISAM2()
        self._initialize()
        self.idx = 0

    def _initialize(self):
        p0 = gtsam.Pose3()
        v0 = np.zeros(3)
        b0 = gtsam.imuBias.ConstantBias()

        self.graph.add(gtsam.PriorFactorPose3(X(0), p0, self.pose_prior))
        self.graph.add(gtsam.PriorFactorVector(V(0), v0, self.vel_prior))
        self.graph.add(gtsam.PriorFactorConstantBias(B(0), b0, self.bias_prior))

        self.values.insert(X(0), p0)
        self.values.insert(V(0), v0)
        self.values.insert(B(0), b0)

        self.isam.update(self.graph, self.values)
        self.graph.resize(0)
        self.values.clear()

    def integrate_imu(self, accel: np.ndarray, gyro: np.ndarray, dt: float):
        """Accumulate IMU readings until next RGB-D frame."""
        self.preint.integrateMeasurement(accel, gyro, dt)

    def process_frame(self, rgbd: o3d.geometry.RGBDImage):
        """
        Call at each RGB-D arrival:
          1. Compute ΔT via VisualOdometry.
          2. Add CombinedImuFactor.
          3. If VO success, add BetweenFactorPose3.
          4. Insert initial guesses and optimize.
        """
        success, delta_T = self.vo.process_rgbd(rgbd)
        prev, cur = self.idx, self.idx + 1

        # (a) IMU factor
        self.graph.add(
            gtsam.CombinedImuFactor(
                X(prev), V(prev),
                X(cur),  V(cur),
                B(prev), B(cur),
                self.preint
            )
        )

        # Bias random-walk factor to keep bias well-posed :contentReference[oaicite:4]{index=4}
        self.graph.add(
            gtsam.BetweenFactorConstantBias(
                B(prev), B(cur),
                gtsam.imuBias.ConstantBias(),
                self.bias_prior
            )
        )

        # (b) Visual factor
        if success:
            meas = gtsam.Pose3(
                gtsam.Rot3(delta_T[:3,:3]),
                gtsam.Point3(delta_T[:3,3])
            )

            vision_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1]*6))

            self.graph.add(
                gtsam.BetweenFactorPose3(X(prev), X(cur), meas, vision_noise)
            )
        else:
            eps_noise = gtsam.noiseModel.Diagonal.Sigmas([1e-3]*6)
            self.graph.add(
                gtsam.BetweenFactorPose3(X(prev), X(cur), gtsam.Pose3(), eps_noise
            ))

        # (c) Initial guesses
        est   = self.isam.calculateEstimate()
        p_pre = est.atPose3(X(prev))
        v_pre = est.atVector(V(prev))
        b_pre = est.atConstantBias(B(prev))

        navstate_prev = gtsam.NavState(p_pre, v_pre)

        pred_state = self.preint.predict(navstate_prev, b_pre)
        p_init = pred_state.pose()
        v_init = pred_state.velocity()

        self.values.insert(X(cur), p_init)
        self.values.insert(V(cur), v_init)
        self.values.insert(B(cur), b_pre)

        # (d) Optimize & reset
        self.isam.update(self.graph, self.values)
        self.graph.resize(0)
        self.values.clear()
        self.preint.resetIntegration()
        self.idx = cur

    def latest_pose(self) -> gtsam.Pose3:
        return self.isam.calculateEstimate().atPose3(X(self.idx))

    def latest_twist(self) -> np.ndarray:
        return self.isam.calculateEstimate().atVector(V(self.idx))
