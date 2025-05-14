import open3d as o3d
import numpy as np

class VisualOdometry:
    """
    Dense RGB-D Visual Odometry using Open3D’s odometry pipelines.
    """

    def __init__(self,
                 intrinsic: o3d.camera.PinholeCameraIntrinsic,
                 method=o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),
                 option=o3d.pipelines.odometry.OdometryOption()):
        """
        :param intrinsic: Open3D PinholeCameraIntrinsic calibration.
        :param method: Odometry Jacobian (hybrid color+geometry).
        :param option: OdometryOption (multi-scale & RANSAC).
        """
        if intrinsic is None:
            raise ValueError("Camera intrinsics must be provided.")
        self.intrinsic   = intrinsic
        self.method      = method
        self.option      = option
        self.prev_rgbd   = None
        self.global_pose = np.eye(4)

    def process_rgbd(self, rgbd: o3d.geometry.RGBDImage):
        """
        Estimate relative transform ΔT between the previous and current RGB-D frames,
        update the cumulative pose, and return (success, ΔT).
        """
        if self.prev_rgbd is None:
            self.prev_rgbd = rgbd
            return False, np.eye(4)

        success, trans, _ = o3d.pipelines.odometry.compute_rgbd_odometry(
            rgbd, self.prev_rgbd, self.intrinsic,
            np.eye(4), self.method, self.option
        )
        self.prev_rgbd = rgbd
        if success:
            self.global_pose = self.global_pose @ trans
            return True, trans
        else:
            return False, None

    def reset(self):
        """Reset odometry to initial state."""
        self.prev_rgbd   = None
        self.global_pose = np.eye(4)
