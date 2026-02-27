#!/usr/bin/env python3

from __future__ import annotations

from typing import Any, Iterable, SupportsFloat, cast

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped, Wrench
from nav_msgs.msg import Odometry
from geometric_controllers.control.controller import Controller


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        self.declare_parameter("rate_hz", 100.0)

        # --- Vehicle params -------------------------------------------------------
        self.declare_parameter("gravity", 9.8)
        self.declare_parameter("mass", 3.646)
        self.declare_parameter("I", [0.1] * 6)
        self.declare_parameter("CoG", [0.0, 0.0, 0.0])

        # --- Controller params ----------------------------------------------------
        self.declare_parameter("controller_type", "PD")  # PD|FeedLin|FeedForward
        self.declare_parameter("adaptation_type", "None")  # None|Euclidean|GeoAware
        self.declare_parameter("potential_type", "liealgebra")  # liealgebra|separate

        self.declare_parameter("Kp_pos", [5.5, 5.5, 5.5])
        self.declare_parameter("Kp_att", [5.5, 5.5, 5.5])
        self.declare_parameter("Kd", [2.05] * 6)

        # --- Load params ----------------------------------------------------------
        def _p(name: str) -> Any:
            return self.get_parameter(name).value

        def _p_float(name: str, default: float) -> float:
            v = _p(name)
            if v is None:
                return float(default)
            return float(cast(SupportsFloat, v))

        def _p_floats(name: str, default: Iterable[float]) -> list[float]:
            v = _p(name)
            if v is None:
                return [float(x) for x in default]
            return [float(x) for x in cast(Iterable[Any], v)]

        self.rate_hz = _p_float("rate_hz", 100.0)

        self.gravity = _p_float("gravity", 9.8)
        self.mass = _p_float("mass", 3.646)
        self.I = _p_floats("I", [0.1] * 6)
        self.CoG = _p_floats("CoG", [0.0, 0.0, 0.0])

        self.controller_type = str(_p("controller_type"))
        self.adaptation_type = str(_p("adaptation_type"))
        self.potential_type = str(_p("potential_type"))

        self.Kp_pos = _p_floats("Kp_pos", [5.5, 5.5, 5.5])
        self.Kp_att = _p_floats("Kp_att", [5.5, 5.5, 5.5])
        self.Kd = _p_floats("Kd", [2.05] * 6)

        # --- State ----------------------------------------------------------------
        self.H = np.eye(4)
        self.V = np.zeros((6, 1))
        self.odometry_received = False

        self.H_des = np.eye(4)
        self.V_des = np.zeros((6, 1))
        self.A_des = np.zeros((6, 1))
        self.desired_received = False

        self.previous_time: float | None = None

        # --- Core controller ------------------------------------------------------
        self.controller = Controller(
            controller_type=self.controller_type,
            adaptation_type=self.adaptation_type,
            potential_type=self.potential_type,
            Kp_att=self.Kp_att,
            Kp_pos=self.Kp_pos,
            Kd=self.Kd,
            m=self.mass,
            I=self.I,
            CoG=self.CoG,
            gravity=self.gravity,
        )

        # --- ROS I/O --------------------------------------------------------------
        cb_group = MutuallyExclusiveCallbackGroup()
        self.odom_sub = self.create_subscription(
            Odometry,
            "/model/variable_tilt_hexacopter/odometry",
            self.odom_callback,
            10,
            callback_group=cb_group,
        )

        self.des_pose_sub = self.create_subscription(
            PoseStamped,
            "/model/variable_tilt_hexacopter/desired_pose",
            self.desired_pose_callback,
            10,
            callback_group=cb_group,
        )

        self.des_vel_sub = self.create_subscription(
            TwistStamped,
            "/model/variable_tilt_hexacopter/desired_velocity",
            self.desired_velocity_callback,
            10,
            callback_group=cb_group,
        )

        self.des_acc_sub = self.create_subscription(
            TwistStamped,
            "/model/variable_tilt_hexacopter/desired_acceleration",
            self.desired_accel_callback,
            10,
            callback_group=cb_group,
        )

        self.wrench_pub = self.create_publisher(
            Wrench, "/model/variable_tilt_hexacopter/desired_wrench", 10
        )
        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._step, callback_group=cb_group)

        self.get_logger().info(
            " | ".join(
                [
                    f"controller_type={self.controller_type}",
                    f"adaptation_type={self.adaptation_type}",
                    f"potential_type={self.potential_type}",
                    f"rate_hz={self.rate_hz:g}",
                ]
            )
        )

    def odom_callback(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist.linear
        omega = msg.twist.twist.angular

        quat = np.array([ori.x, ori.y, ori.z, ori.w])
        R_mat = R.from_quat(quat).as_matrix()

        self.H = np.eye(4)
        self.H[:3, :3] = R_mat
        self.H[:3, 3] = np.array([pos.x, pos.y, pos.z])

        vel_vec = np.array([vel.x, vel.y, vel.z]).reshape(3, 1)
        omega_vec = np.array([omega.x, omega.y, omega.z]).reshape(3, 1)
        self.V = np.vstack((omega_vec, vel_vec))

        self.odometry_received = True

    def desired_pose_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        R_mat = R.from_quat(quat).as_matrix()

        self.H_des = np.eye(4)
        self.H_des[:3, :3] = R_mat
        self.H_des[:3, 3] = np.array([p.x, p.y, p.z])
        self.desired_received = True

    def desired_velocity_callback(self, msg: TwistStamped) -> None:
        w = msg.twist.angular
        v = msg.twist.linear
        omega = np.array([w.x, w.y, w.z]).reshape(3, 1)
        vel = np.array([v.x, v.y, v.z]).reshape(3, 1)
        self.V_des = np.vstack((omega, vel))
        self.desired_received = True

    def desired_accel_callback(self, msg: TwistStamped) -> None:
        w = msg.twist.angular
        a = msg.twist.linear
        omega_dot = np.array([w.x, w.y, w.z]).reshape(3, 1)
        accel = np.array([a.x, a.y, a.z]).reshape(3, 1)
        self.A_des = np.vstack((omega_dot, accel))
        self.desired_received = True

    def _step(self) -> None:
        if not self.odometry_received or not self.desired_received:
            return

        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        if self.previous_time is None:
            dt = None
        else:
            dt = max(t - self.previous_time, 0.0)
        self.previous_time = t

        W = self.controller.compute_wrench(
            H_des=self.H_des,
            H=self.H,
            V_des=self.V_des,
            V=self.V,
            A_des=self.A_des,
            dt=dt,
        ).flatten()

        msg = Wrench()
        msg.torque.x, msg.torque.y, msg.torque.z = float(W[0]), float(W[1]), float(W[2])
        msg.force.x, msg.force.y, msg.force.z = float(W[3]), float(W[4]), float(W[5])
        self.wrench_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
