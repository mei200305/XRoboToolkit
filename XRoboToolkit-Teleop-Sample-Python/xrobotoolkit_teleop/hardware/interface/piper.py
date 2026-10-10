from __future__ import annotations

from typing import Iterable

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2


class PiperSingleArmController:
    """
    ROS 2 interface for a single Piper arm driven by `agilexrobotics/piper_ros`.

    The controller writes desired joint targets to `/joint_states` and reads
    feedback from `/joint_states_single`, matching the public piper_ros README.
    """

    def __init__(
        self,
        joint_control_topic: str = "/joint_states",
        joint_state_topic: str = "/joint_states_single",
        enable_topic: str = "/enable_flag",
        rate_hz: float = 100.0,
        auto_enable: bool = True,
        joint_names: list[str] | None = None,
    ):
        self.joint_names = joint_names or [f"joint{i}" for i in range(1, 8)]
        self.node = ros2.ensure_node("piper_single_arm")

        self.command_pub = self.node.create_publisher(JointState, joint_control_topic, 1)
        self.enable_pub = self.node.create_publisher(Bool, enable_topic, ros2.latched_qos(1))
        self.state_sub = self.node.create_subscription(
            JointState, joint_state_topic, self._joint_state_callback, 10
        )
        self.rate_hz = float(rate_hz)

        self.qpos = [0.0] * len(self.joint_names)
        self.qvel = [0.0] * len(self.joint_names)
        self.effort = [0.0] * len(self.joint_names)
        self.timestamp = 0.0

        self.q_des = None
        self.v_des = [0.0] * len(self.joint_names)
        self.effort_des = [0.0] * len(self.joint_names)

        self.command_msg = JointState()
        self.command_msg.name = self.joint_names.copy()

        if auto_enable:
            self.set_enable(True)

    def _joint_state_callback(self, msg: JointState):
        joint_count = len(self.joint_names)
        if len(msg.position) < joint_count:
            self.node.get_logger().warn(
                f"Received Piper state with {len(msg.position)} joints, expected at least {joint_count}"
            )
            return

        self.qpos = list(msg.position[:joint_count])
        self.qvel = list(msg.velocity[:joint_count]) if len(msg.velocity) >= joint_count else [0.0] * joint_count
        self.effort = list(msg.effort[:joint_count]) if len(msg.effort) >= joint_count else [0.0] * joint_count
        self.q_des = self.qpos.copy() if self.q_des is None else self.q_des
        stamp_sec = ros2.stamp_to_sec(msg.header.stamp)
        self.timestamp = stamp_sec if stamp_sec else ros2.get_time(self.node)

    def set_enable(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.enable_pub.publish(msg)

    def set_joint_positions(
        self,
        positions: Iterable[float],
        velocities: Iterable[float] | None = None,
        efforts: Iterable[float] | None = None,
    ):
        self.q_des = list(positions)[: len(self.joint_names)]
        if len(self.q_des) < len(self.joint_names):
            self.q_des.extend([0.0] * (len(self.joint_names) - len(self.q_des)))

        if velocities is not None:
            self.v_des = list(velocities)[: len(self.joint_names)]
        if len(self.v_des) < len(self.joint_names):
            self.v_des.extend([0.0] * (len(self.joint_names) - len(self.v_des)))

        if efforts is not None:
            self.effort_des = list(efforts)[: len(self.joint_names)]
        if len(self.effort_des) < len(self.joint_names):
            self.effort_des.extend([0.0] * (len(self.joint_names) - len(self.effort_des)))

    def publish_joint_control(self):
        if self.q_des is None:
            return

        self.command_msg.header.stamp = ros2.now_msg(self.node)
        self.command_msg.position = list(self.q_des)
        self.command_msg.velocity = list(self.v_des)
        self.command_msg.effort = list(self.effort_des)
        self.command_pub.publish(self.command_msg)

    def stop(self):
        try:
            self.node.destroy_publisher(self.command_pub)
            self.node.destroy_publisher(self.enable_pub)
            self.node.destroy_subscription(self.state_sub)
        except Exception:
            pass
