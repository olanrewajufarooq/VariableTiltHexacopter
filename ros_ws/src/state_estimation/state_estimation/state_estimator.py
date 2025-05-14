import numpy as np
import gtsam
from gtsam.symbol_shorthand import X, V, B
from abc import ABC, abstractmethod


class AbstractStateEstimator(ABC):
    @abstractmethod
    def integrate_imu(self, acc: np.ndarray, gyro: np.ndarray, dt: float): 
        pass

    @abstractmethod
    def update_with_visual(self, relative_pose: np.ndarray): 
        pass

    @abstractmethod
    def propagate_if_needed(self): 
        pass

    @abstractmethod
    def latest_pose(self) -> gtsam.Pose3: 
        pass

    @abstractmethod
    def latest_twist(self) -> np.ndarray: 
        pass


class _BaseEstimator(AbstractStateEstimator):
    def __init__(self, gravity, accel_noise, gyro_noise, integration_cov,
                 bias_noise, pose_prior_sigmas, vel_sigma, bias_sigma):
        
        self.k = 0
        self.bias = gtsam.imuBias.ConstantBias()
        
        # Creating value object and factor graph

        self.initial = gtsam.Values()
        self.graph = gtsam.NonlinearFactorGraph()
        
        # Initialize ISAM2
        isam_params = gtsam.ISAM2Params()
        isam_params.setFactorization('QR')
        isam_params.setRelinearizeThreshold(0.1)
        isam_params.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(isam_params)

        # Initialize noise models
        self.prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(pose_prior_sigmas))
        self.prior_vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, vel_sigma)
        self.prior_bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, bias_sigma)
        self.bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, bias_noise)

        # Initialize IMU parameters
        self.imu_params = gtsam.PreintegrationParams.MakeSharedU(gravity)
        self.imu_params.setAccelerometerCovariance(np.eye(3) * accel_noise)
        self.imu_params.setGyroscopeCovariance(np.eye(3) * gyro_noise)
        self.imu_params.setIntegrationCovariance(np.eye(3) * integration_cov)

        self._reset_preintegrator()
        self._initialize()

    def _reset_preintegrator(self):
        self.preintegrated = gtsam.PreintegratedImuMeasurements(self.imu_params, self.bias)

    def _initialize(self):
        pose0 = gtsam.Pose3()
        vel0 = np.zeros(3)
        self.graph.add(gtsam.PriorFactorPose3(X(0), pose0, self.prior_pose_noise))
        self.graph.add(gtsam.PriorFactorVector(V(0), vel0, self.prior_vel_noise))
        self.graph.add(gtsam.PriorFactorConstantBias(B(0), self.bias, self.prior_bias_noise))
        self.initial.insert(X(0), pose0)
        self.initial.insert(V(0), vel0)
        self.initial.insert(B(0), self.bias)
        self.k = 1

        self._optimize()

    def integrate_imu(self, acc, gyro, dt):
        self.preintegrated.integrateMeasurement(acc, gyro, dt)

    def _add_imu_factor(self):
        self.graph.add(gtsam.ImuFactor(X(self.k - 1), V(self.k - 1), X(self.k), V(self.k), B(self.k - 1), self.preintegrated))
        self.graph.add(gtsam.BetweenFactorConstantBias(B(self.k - 1), B(self.k), gtsam.imuBias.ConstantBias(), self.bias_noise))

    def _insert_state(self, pose, velocity=None):
        if velocity is None:
            velocity = np.zeros(3)
        self.initial.insert(X(self.k), pose)
        self.initial.insert(V(self.k), velocity)
        self.initial.insert(B(self.k), self.bias)
        self.k += 1

    def _optimize(self):
        self.isam.update(self.graph, self.initial)
        self.graph.resize(0)
        self.initial.clear()
        
        estimate = self.isam.calculateEstimate()
        self.bias = estimate.atConstantBias(B(self.k - 1))  # Update bias
        self._reset_preintegrator()

    def latest_pose(self):
        try:
            return self.isam.calculateEstimate().atPose3(X(self.k - 1))
        except RuntimeError as e:
            return None
    
    def latest_twist(self):
        try:
            return self.isam.calculateEstimate().atVector(V(self.k - 1))
        except RuntimeError as e:
            return None


class _RobustEstimator(_BaseEstimator):
    def update_with_visual(self, relative_pose: np.ndarray):
        self._add_imu_factor()
        prev_pose = self.latest_pose()

        if prev_pose is None:
            self._insert_state(gtsam.Pose3(relative_pose))
            self._optimize()
            return True

        # Estimate pose from composition
        new_pose = prev_pose.compose(gtsam.Pose3(relative_pose))

        # Add relative constraint
        noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1]*6))
        self.graph.add(gtsam.BetweenFactorPose3(X(self.k - 1), X(self.k), gtsam.Pose3(relative_pose), noise))

        self._insert_state(new_pose)
        self._optimize()
        return True

    def propagate_if_needed(self):
        prev_estimate = self.isam.calculateEstimate()
        prev_pose = prev_estimate.atPose3(X(self.k - 1))
        prev_velocity = prev_estimate.atVector(V(self.k - 1))

        # Predict current pose from preintegrated IMU
        pred_state = self.preintegrated.predict(prev_pose, prev_velocity, self.bias)

        self._add_imu_factor()
        self._insert_state(pred_state.pose(), pred_state.v())
        self._optimize()
        return True


class _FullEstimator(_BaseEstimator):
    def update_with_visual(self, relative_pose: np.ndarray):
        self._add_imu_factor()
        prev_pose = self.latest_pose()
        pose = prev_pose.compose(gtsam.Pose3(relative_pose))
        self._insert_state(pose)
        self._optimize()


    def propagate_if_needed(self):
        return True  # No need to propagate in full mode


class StateEstimator(AbstractStateEstimator):
    def __init__(self, mode="robust", gravity=9.81,
                 accel_noise=0.1, gyro_noise=0.1, integration_cov=1e-4,
                 bias_noise=1e-3, pose_prior_sigmas=None,
                 vel_sigma=0.1, bias_sigma=1e-3):
        
        if pose_prior_sigmas is None:
            pose_prior_sigmas = [0.1] * 6

        assert mode in ["robust", "full"], "mode must be 'robust' or 'full'"

        self.mode = mode
        self.initialized = False

        estimator_cls = _RobustEstimator if mode == "robust" else _FullEstimator

        self._impl = estimator_cls(
            gravity=gravity,
            accel_noise=accel_noise,
            gyro_noise=gyro_noise,
            integration_cov=integration_cov,
            bias_noise=bias_noise,
            pose_prior_sigmas=pose_prior_sigmas,
            vel_sigma=vel_sigma,
            bias_sigma=bias_sigma
        )

    def integrate_imu(self, acc, gyro, dt):
        self._impl.integrate_imu(acc, gyro, dt)

    def update_with_visual(self, relative_pose):
        success = self._impl.update_with_visual(relative_pose)
        if not self.initialized and success:
            self.initialized = True
        return success if self.mode == "robust" else True

    def propagate_if_needed(self):
        success = self._impl.propagate_if_needed()
        return success if self.mode == "robust" else True

    def latest_pose(self):
        return self._impl.latest_pose()
    
    def latest_twist(self):
        return self._impl.latest_twist()
    
    def is_initialized(self):
        return self.initialized
