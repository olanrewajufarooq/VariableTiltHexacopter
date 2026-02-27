#!/usr/bin/env python3

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

from geometric_controllers.trajectories import PathGenerator


class TrajectoryNode(Node):
    """Trajectory generator node.

    Publishes:
    - desired pose: `/model/variable_tilt_hexacopter/desired_pose` (PoseStamped)
    - desired velocity: `/model/variable_tilt_hexacopter/desired_velocity` (TwistStamped)
    - desired acceleration: `/model/variable_tilt_hexacopter/desired_acceleration` (TwistStamped)
      (angular = omega_dot, linear = a_body)

    Optional:
    - payload pick/drop command: `/model/variable_tilt_hexacopter/payload/pick` (Bool)
    """

    def __init__(self) -> None:
        super().__init__("trajectory_node")

        self.declare_parameter("rate_hz", 100.0)

        self.declare_parameter("path", "hover")
        self.declare_parameter("path_scale", 5.0)
        self.declare_parameter("path_period", 20.0)
        self.declare_parameter("path_altitude", 5.0)
        self.declare_parameter("path_start_with_hover", True)

        self.declare_parameter("payload_events_enabled", False)
        self.declare_parameter("payload_pick_wait_s", 2.0)
        self.declare_parameter("payload_drop_time_s", -1.0)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.path_name = str(self.get_parameter("path").value)
        self.path_scale = float(self.get_parameter("path_scale").value)
        self.path_period = float(self.get_parameter("path_period").value)
        self.path_altitude = float(self.get_parameter("path_altitude").value)
        self.path_start_with_hover = bool(
            self.get_parameter("path_start_with_hover").value
        )

        self.payload_events_enabled = bool(
            self.get_parameter("payload_events_enabled").value
        )
        self.payload_pick_wait_s = float(self.get_parameter("payload_pick_wait_s").value)
        drop_time = float(self.get_parameter("payload_drop_time_s").value)
        if drop_time < 0.0:
            drop_time = self.payload_pick_wait_s + (self.path_period / 2.0)
        self.payload_drop_time_s = drop_time

        self.start_time: float | None = None
        self.pick_sent = False
        self.drop_sent = False

        self.path_generator = PathGenerator(
            path_name=self.path_name,
            scale=self.path_scale,
            period=self.path_period,
            altitude=self.path_altitude,
            start_with_hover=self.path_start_with_hover,
        )

        cb_group = MutuallyExclusiveCallbackGroup()
        self.desired_pose_pub = self.create_publisher(
            PoseStamped, "/model/variable_tilt_hexacopter/desired_pose", 10
        )
        self.desired_velocity_pub = self.create_publisher(
            TwistStamped, "/model/variable_tilt_hexacopter/desired_velocity", 10
        )
        self.desired_accel_pub = self.create_publisher(
            TwistStamped, "/model/variable_tilt_hexacopter/desired_acceleration", 10
        )

        self.pick_pub = None
        if self.payload_events_enabled:
            self.pick_pub = self.create_publisher(
                Bool, "/model/variable_tilt_hexacopter/payload/pick", 10
            )

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._step, callback_group=cb_group)

        self.get_logger().info(
            " | ".join(
                [
                    f"path={self.path_name}",
                    f"scale={self.path_scale:g}",
                    f"period={self.path_period:g}",
                    f"altitude={self.path_altitude:g}",
                    f"rate_hz={self.rate_hz:g}",
                    f"payload_events_enabled={self.payload_events_enabled}",
                ]
            )
        )

    def _maybe_publish_payload_events(self, elapsed: float) -> None:
        if not self.payload_events_enabled or self.pick_pub is None:
            return

        if elapsed <= self.payload_pick_wait_s and not self.pick_sent:
            self.pick_pub.publish(Bool(data=True))
            self.pick_sent = True
            return

        if elapsed >= self.payload_drop_time_s and not self.drop_sent:
            self.pick_pub.publish(Bool(data=False))
            self.drop_sent = True

    def _publish_desired(self, now) -> None:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(self.H_des[0, 3])
        pose_msg.pose.position.y = float(self.H_des[1, 3])
        pose_msg.pose.position.z = float(self.H_des[2, 3])
        quat = R.from_matrix(self.H_des[:3, :3]).as_quat()
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.desired_pose_pub.publish(pose_msg)

        twist_msg = TwistStamped()
        twist_msg.header.stamp = now.to_msg()
        twist_msg.header.frame_id = "base_link"
        V_flat = self.V_des.flatten()
        twist_msg.twist.angular.x = float(V_flat[0])
        twist_msg.twist.angular.y = float(V_flat[1])
        twist_msg.twist.angular.z = float(V_flat[2])
        twist_msg.twist.linear.x = float(V_flat[3])
        twist_msg.twist.linear.y = float(V_flat[4])
        twist_msg.twist.linear.z = float(V_flat[5])
        self.desired_velocity_pub.publish(twist_msg)

        accel_msg = TwistStamped()
        accel_msg.header.stamp = now.to_msg()
        accel_msg.header.frame_id = "base_link"
        A_flat = self.A_des.flatten()
        accel_msg.twist.angular.x = float(A_flat[0])
        accel_msg.twist.angular.y = float(A_flat[1])
        accel_msg.twist.angular.z = float(A_flat[2])
        accel_msg.twist.linear.x = float(A_flat[3])
        accel_msg.twist.linear.y = float(A_flat[4])
        accel_msg.twist.linear.z = float(A_flat[5])
        self.desired_accel_pub.publish(accel_msg)

    def _step(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        if self.start_time is None:
            self.start_time = t

        elapsed = t - self.start_time
        self._maybe_publish_payload_events(elapsed)

        self.H_des, self.V_des, self.A_des = self.path_generator.generate(elapsed)
        self._publish_desired(now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
