import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped

import numpy as np
from open3d_ros2_helper import open3d_ros2_helper as o3d_helper

from state_estimation.state_estimator import StateEstimator
from state_estimation.relative_pose_estimator import RelativePoseEstimator


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__('state_estimator_node')
        self.last_imu_time = None
        self.color_msg = None
        self.depth_msg = None
        self.camera_info = None
        self.intrinsic_loaded = False
        self.warn_cache = {}

        # Declare parameters
        mode = self.declare_parameter("mode", "robust").value.lower()
        g = self.declare_parameter("estimator.gravity", 9.81).value
        accel = self.declare_parameter("estimator.imu.accel_noise", 0.1).value
        gyro = self.declare_parameter("estimator.imu.gyro_noise", 0.1).value
        int_cov = self.declare_parameter("estimator.imu.integration_cov", 1e-4).value
        b_noise = self.declare_parameter("estimator.imu.bias_noise", 1e-3).value
        pose_sigmas = self.declare_parameter("estimator.prior_noise.pose", [0.1]*6).value
        vel_sigma = self.declare_parameter("estimator.prior_noise.velocity", 0.1).value
        bias_sigma = self.declare_parameter("estimator.prior_noise.bias", 1e-3).value

        self.estimator = StateEstimator(
            mode=mode, gravity=g, accel_noise=accel, gyro_noise=gyro,
            integration_cov=int_cov, bias_noise=b_noise,
            pose_prior_sigmas=pose_sigmas, vel_sigma=vel_sigma,
            bias_sigma=bias_sigma
        )
        
        self.odom = RelativePoseEstimator(None)

        # Subscriptions
        self.sub_color = self.create_subscription(Image, '/model/variable_tilt_hexacopter/camera/image', self.color_cb, 10)
        self.sub_depth = self.create_subscription(PointCloud2, '/model/variable_tilt_hexacopter/camera/points', self.depth_cb, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/model/variable_tilt_hexacopter/camera_info', self.info_cb, 10)
        self.sub_imu = self.create_subscription(Imu, '/model/variable_tilt_hexacopter/imu', self.imu_cb, 100)

        # Publishers
        self.pub_pose = self.create_publisher(PoseStamped, '/model/variable_tilt_hexacopter/estimator/pose', 10)
        self.pub_twist = self.create_publisher(TwistStamped, '/model/variable_tilt_hexacopter/estimator/twist', 10)

    # ───── Callback Methods ─────────────────────────────
    def color_cb(self, msg):
        self.color_msg = msg
        self.try_process()

    def depth_cb(self, msg):
        self.depth_msg = msg
        self.try_process()

    def info_cb(self, msg):
        if not self.intrinsic_loaded:
            try:
                self.odom.intrinsics = o3d_helper.convert_intrinsics(msg)
                self.camera_info = msg
                self.intrinsic_loaded = True
                self.get_logger().info(f"Camera intrinsics loaded. Width: {msg.width}, Height: {msg.height}")
                self.clear_warn("camera_info_failed", "Camera intrinsics successfully loaded.")
            except Exception as e:
                self.warn_once("camera_info_failed", f"Failed to load camera intrinsics: {e}")

    def imu_cb(self, msg):
        t_curr = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = t_curr
            return

        if not self.estimator.is_initialized():
            self.warn_once("estimator_uninit_imu", "Estimator not initialized. Skipping IMU update.")
            self.last_imu_time = t_curr
            return
        else:
            self.clear_warn("estimator_uninit_imu", "Estimator initialized. IMU updates resumed.")

        dt = t_curr - self.last_imu_time
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.estimator.integrate_imu(acc, gyro, dt)

        success = self.estimator.propagate_if_needed()
        if success:
            self.publish_state()
            self.clear_warn("imu_propagation_failed", "IMU propagation resumed successfully.")
        else:
            self.warn_once("imu_propagation_failed", "IMU Propagation failed. Skipping publish.")

        self.last_imu_time = t_curr

    # ───── Visual Odometry Processing ───────────────────────
    def try_process(self):
        if not (self.color_msg and self.depth_msg and self.intrinsic_loaded):
            return

        if self.estimator.latest_pose() is None:
            self.warn_once("estimator_uninit_vo", "Skipping VO update: estimator not initialized.")
            return
        else:
            self.clear_warn("estimator_uninit_vo", "Estimator initialized. VO updates resumed.")

        try:
            rgbd = o3d_helper.convert_rgbd_from_pointcloud(
                self.color_msg, self.depth_msg,
                self.camera_info.width, self.camera_info.height,
            )
            if rgbd.color is None or rgbd.depth is None:
                self.warn_once("rgbd_invalid", "RGB-D conversion returned invalid frame.")
                return
            else:
                self.clear_warn("rgbd_invalid", "RGB-D conversion valid again.")
            

            success, rel_pose = self.odom.estimate_from_rgbd(rgbd)
            if not (success and rel_pose is not None):
                self.warn_once("vo_failed", "Visual odometry failed. Skipping update.")
                return
            else:
                self.clear_warn("vo_failed", "Visual odometry source valid again.")

            success = self.estimator.update_with_visual(rel_pose)
            if success:
                self.publish_state()
                self.clear_warn("vo_update_failed", "Visual odometry update successful.")
            else:
                self.warn_once("vo_update_failed", "Visual odometry update failed. Skipping publish.")

        except Exception as e:
            self.warn_once("open3d_failed", f"Open3D conversion failed: {e}")

        finally:
            self.color_msg = None
            self.depth_msg = None

    # ───── State Publishing ──────────────────────────────
    def publish_state(self):
        pose = self.estimator.latest_pose()
        twist = self.estimator.latest_twist()

        if pose is None or twist is None:
            self.warn_once("state_publish_failed", "Skipping publish: Pose and/or twist not available yet.")
            return
        else:
            self.clear_warn("state_publish_failed", "Pose and twist available. Publishing resumed.")

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        t = pose.translation()
        q = pose.rotation().toQuaternion()
        pose_msg.pose.position.x = float(t[0])
        pose_msg.pose.position.y = float(t[1])
        pose_msg.pose.position.z = float(t[2])
        pose_msg.pose.orientation.x = float(q.x())
        pose_msg.pose.orientation.y = float(q.y())
        pose_msg.pose.orientation.z = float(q.z())
        pose_msg.pose.orientation.w = float(q.w())
        self.pub_pose.publish(pose_msg)

        twist_msg = TwistStamped()
        twist_msg.header = pose_msg.header
        twist_msg.twist.linear.x = twist[0]
        twist_msg.twist.linear.y = twist[1]
        twist_msg.twist.linear.z = twist[2]
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.pub_twist.publish(twist_msg)

    # ───── Logging Helpers ───────────────────────────────
    def warn_once(self, key: str, message: str):
        if not self.warn_cache.get(key, False):
            self.get_logger().warn(message)
            self.warn_cache[key] = True

    def clear_warn(self, key: str, resolved_msg: str = None):
        if self.warn_cache.get(key, False):
            self.warn_cache[key] = False
            if resolved_msg:
                self.get_logger().info(resolved_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    rclpy.spin(node)
    rclpy.shutdown()
