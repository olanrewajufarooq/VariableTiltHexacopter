import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import open3d as o3d
from open3d_ros2_helper import open3d_ros2_helper as o3d_helper
from state_estimation.vo import VisualOdometry

class VONode(Node):
    def __init__(self):
        super().__init__('vo_node')
        self.declare_parameter('frame_id', 'world')
        self.frame_id   = self.get_parameter('frame_id').value
        self.pose_topic = self.get_parameter('pose_topic').value

        self.intrinsic = None
        self.vo        = None
        self._color    = None
        self._depth    = None

        # Subscribers
        self.create_subscription(CameraInfo,
                                 '/model/variable_tilt_hexacopter/camera_info',
                                 self.info_cb, 10)
        self.create_subscription(Image,
                                 '/model/variable_tilt_hexacopter/camera/image',
                                 self.color_cb, 10)
        self.create_subscription(Image,
                                 '/model/variable_tilt_hexacopter/camera/depth_image',
                                 self.depth_cb, 10)

        # Publisher
        self.pub_pose = self.create_publisher(PoseStamped, "/model/variable_tilt_hexacopter/vo/pose", 10)

    def info_cb(self, msg: CameraInfo):
        if self.intrinsic is None:
            try:
                self.intrinsic = o3d_helper.convert_intrinsics(msg)
                self.vo        = VisualOdometry(self.intrinsic)
                self.get_logger().info("Camera intrinsics loaded.")
            except Exception as e:
                self.get_logger().warn(f"Failed to load intrinsics: {e}")

    def color_cb(self, msg: Image):
        self._color = msg
        self.try_process()

    def depth_cb(self, msg: Image):
        self._depth = msg
        self.try_process()

    def try_process(self):
        if not (self.intrinsic and self._color and self._depth):
            return
        try:
            rgbd = o3d_helper.convert_rgbd(self._color,
                                           self._depth)
            success, T = self.vo.process_rgbd(rgbd)

            # Prepare PoseStamped
            ps = PoseStamped()
            ps.header.stamp    = self.get_clock().now().to_msg()
            ps.header.frame_id = self.frame_id

            # Translation
            t = T[:3, 3]
            ps.pose.position.x = float(t[0])
            ps.pose.position.y = float(t[1])
            ps.pose.position.z = float(t[2])

            # Rotation → quaternion
            qw, qx, qy, qz = o3d_helper.rotation_matrix_to_quaternion(T[:3, :3])
            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)
            ps.pose.orientation.w = float(qw)

            self.pub_pose.publish(ps)

        except Exception as e:
            self.get_logger().warn(f"VO processing failed: {e}")
        finally:
            self._color = None
            self._depth = None

def main(args=None):
    rclpy.init(args=args)
    node = VONode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
