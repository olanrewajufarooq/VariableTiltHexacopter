import open3d as o3d
import numpy as np

class RelativePoseEstimator:
    def __init__(self, intrinsics=None):
        self.intrinsics = intrinsics  # open3d.camera.PinholeCameraIntrinsic
        self.prev_rgbd = None

    def estimate_from_rgbd(self, rgbd):
        if self.intrinsics is None:
            raise ValueError("Camera intrinsics not set in RelativePoseEstimator.")

        if self.prev_rgbd is None or rgbd is None:
            self.prev_rgbd = rgbd
            return False, np.eye(4)

        success, trans, _ = o3d.pipelines.odometry.compute_rgbd_odometry(
            rgbd,
            self.prev_rgbd,
            self.intrinsics,
            np.eye(4),
            o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm()
        )
        self.prev_rgbd = rgbd

        if not success:
            return False, None
        
        return success, trans
