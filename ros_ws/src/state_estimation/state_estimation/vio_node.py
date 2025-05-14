import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped

import numpy as np
# import open3d as o3d
from open3d_ros2_helper import open3d_ros2_helper as o3d_helper

from state_estimation.vio import VisualInertiaOdometry

class VIONode(Node):
    def __init__(self):
        super().__init__('vio_node')

        self.frame_id = 'world'

        # Placeholders
        self.intrinsic = None
        self.vio       = None
        self.last_imu  = None
        self._rgbd_msgs = {'color': None, 'depth': None}

        # Subscribers
        self.create_subscription(CameraInfo, '/model/variable_tilt_hexacopter/camera/camera_info', self.info_cb, 1)
        self.create_subscription(Image, '/model/variable_tilt_hexacopter/camera/image', lambda msg: self.image_cb(msg, 'color'), 20)
        self.create_subscription(Image, '/model/variable_tilt_hexacopter/camera/depth_image', lambda msg: self.image_cb(msg, 'depth'), 20)
        self.create_subscription(Imu, '/model/variable_tilt_hexacopter/imu', self.imu_cb, 100)

        # Publishers
        self.pub_pose  = self.create_publisher(PoseStamped, '/model/variable_tilt_hexacopter/vio/pose', 10)
        self.pub_twist = self.create_publisher(TwistStamped, '/model/variable_tilt_hexacopter/vio/twist', 10)

        # Create a timer to publish the state
        self.create_timer(0.01, self.publish_state)

        self.get_logger().info("VIO node initialized.")

    def info_cb(self, msg: CameraInfo):
        if self.intrinsic is None or self.vio is None:
            try:
                self.intrinsic = o3d_helper.convert_intrinsics(msg)
                self.get_logger().info("VIO intrinsics loaded.")
                self.vio       = VisualInertiaOdometry(self.intrinsic)
                self.get_logger().info("VIO initialized.")
            except Exception as e:
                self.get_logger().warn(f"Failed to load intrinsics: {e}")

    def image_cb(self, msg: Image, typ: str):
        self._rgbd_msgs[typ] = msg
        self.try_process()

    def imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu is None:
            self.last_imu = t
            return
        
        if self.vio is None:
            self.get_logger().warn("VIO not initialized—skipping IMU update")
            self.last_imu = t
            return
        
        dt = t - self.last_imu
        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        gyro  = np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
        self.vio.integrate_imu(accel, gyro, dt)
        self.last_imu = t

    def try_process(self):
        if not (self.vio and self._rgbd_msgs['color'] and self._rgbd_msgs['depth']):
            return
        try:
            rgbd = o3d_helper.convert_rgbd(self._rgbd_msgs['color'],
                                           self._rgbd_msgs['depth'])
            self.vio.process_frame(rgbd)
            self.get_logger().info("VIO processed frame.")

        except Exception as e:
            self.get_logger().warn(f"VIO process failed: {e}")
        finally:
            self._rgbd_msgs = {'color': None, 'depth': None}

    def publish_state(self):
        """
        Called at 10 Hz by the timer.  Publishes the
        latest pose & twist, if available.
        """
        if self.vio is None:
            return

        pose = self.vio.latest_pose()
        twist = self.vio.latest_twist()
        if pose is None:
            return

        # --- Publish PoseStamped ---
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        t = pose.translation()
        q = pose.rotation().toQuaternion()
        ps.pose.position.x = float(t[0])
        ps.pose.position.y = float(t[1])
        ps.pose.position.z = float(t[2])
        ps.pose.orientation.x = float(q.x())
        ps.pose.orientation.y = float(q.y())
        ps.pose.orientation.z = float(q.z())
        ps.pose.orientation.w = float(q.w())
        self.pub_pose.publish(ps)

        if twist is None:
            return
        # --- Publish TwistStamped ---
        ts = TwistStamped()
        ts.header = ps.header
        ts.twist.linear.x  = float(twist[0])
        ts.twist.linear.y  = float(twist[1])
        ts.twist.linear.z  = float(twist[2])
        # Angular rates could be added here if desired
        self.pub_twist.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = VIONode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
