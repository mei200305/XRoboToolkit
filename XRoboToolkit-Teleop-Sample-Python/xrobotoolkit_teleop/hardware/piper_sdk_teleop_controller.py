from typing import Dict, List

import numpy as np
from sensor_msgs.msg import JointState

from xrobotoolkit_teleop.common.base_hardware_teleop_controller import (
    HardwareTeleopController,
)
from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2
from xrobotoolkit_teleop.hardware.interface.piper_sdk_interface import PiperSDKInterface
from xrobotoolkit_teleop.hardware.piper_teleop_controller import (
    DEFAULT_MANIPULATOR_CONFIG,
    DEFAULT_PIPER_ARM_JOINT_NAMES,
    DEFAULT_PIPER_JOINT_NAMES,
    DEFAULT_PIPER_URDF_PATH,
)
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD
from xrobotoolkit_teleop.utils.path_utils import resolve_ros_package_uris_in_urdf


class PiperSDKTeleopController(HardwareTeleopController):
    def __init__(
        self,
        robot_urdf_path: str = DEFAULT_PIPER_URDF_PATH,
        manipulator_config: dict = DEFAULT_MANIPULATOR_CONFIG,
        joint_names: List[str] = DEFAULT_PIPER_JOINT_NAMES,
        arm_joint_names: List[str] = DEFAULT_PIPER_ARM_JOINT_NAMES,
        can_name: str = "can0",
        judge_flag: bool = True,
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        scale_factor: float = 1.0,
        visualize_placo: bool = False,
        control_rate_hz: int = 100,
        enable_log_data: bool = True,
        log_dir: str = "logs/piper",
        log_freq: float = 50,
        enable_camera: bool = False,
        camera_fps: int = 30,
        auto_enable: bool = True,
        piper_joint_speed: int = 50,
        command_filter_alpha: float = 0.35,
        max_joint_velocity_rad_s: float = 1.5,
        max_gripper_velocity_m_s: float = 0.08,
        disable_on_shutdown: bool = False,
    ):
        self.joint_names = joint_names
        self.arm_joint_names = arm_joint_names
        self.can_name = can_name
        self.judge_flag = judge_flag
        self.auto_enable = auto_enable
        self.piper_joint_speed = piper_joint_speed
        self.command_filter_alpha = max(0.0, min(1.0, command_filter_alpha))
        self.max_joint_velocity_rad_s = max_joint_velocity_rad_s
        self.max_gripper_velocity_m_s = max_gripper_velocity_m_s
        self.disable_on_shutdown = disable_on_shutdown
        self.joint_state_topic = "/joint_states_single"
        self._smoothed_q_des = None
        self._go_zero_requested = False
        super().__init__(
            robot_urdf_path=resolve_ros_package_uris_in_urdf(
                robot_urdf_path,
                package_map={"piper_description": "/home/piper_ros/src/piper_description"},
            ),
            manipulator_config=manipulator_config,
            R_headset_world=R_headset_world,
            floating_base=False,
            scale_factor=scale_factor,
            visualize_placo=visualize_placo,
            control_rate_hz=control_rate_hz,
            enable_log_data=enable_log_data,
            log_dir=log_dir,
            log_freq=log_freq,
            enable_camera=enable_camera,
            camera_fps=camera_fps,
        )

    def _placo_setup(self):
        super()._placo_setup()
        self.placo_arm_joint_slice: Dict[str, slice] = {}
        for arm_name in self.manipulator_config.keys():
            self.placo_arm_joint_slice[arm_name] = slice(
                self.placo_robot.get_joint_offset(self.arm_joint_names[0]),
                self.placo_robot.get_joint_offset(self.arm_joint_names[-1]) + 1,
            )

    def _robot_setup(self):
        self._ros_node = ros2.ensure_node("piper_sdk_teleop_controller")
        self.arm_controller = PiperSDKInterface(
            can_name=self.can_name,
            judge_flag=self.judge_flag,
            auto_enable=self.auto_enable,
            gripper_exist=True,
            default_joint_speed=self.piper_joint_speed,
        )
        self.state_publisher = self._ros_node.create_publisher(
            JointState,
            self.joint_state_topic,
            1,
        )
        self.arm_controller.refresh_state()

    def _initialize_camera(self):
        self.camera_interface = None

    def _get_rosbag_topics(self) -> list[str]:
        return [self.joint_state_topic]

    def _publish_joint_state_for_logging(self):
        msg = JointState()
        msg.header.stamp = ros2.now_msg(self._ros_node)
        msg.name = self.joint_names
        msg.position = list(self.arm_controller.qpos)
        msg.velocity = list(self.arm_controller.qvel)
        msg.effort = list(self.arm_controller.effort)
        self.state_publisher.publish(msg)

    def _update_robot_state(self):
        self.arm_controller.refresh_state()
        self._publish_joint_state_for_logging()
        q_slice = self.placo_arm_joint_slice["right_arm"]
        self.placo_robot.state.q[q_slice] = self.arm_controller.qpos[: len(self.arm_joint_names)]

    def _send_command(self):
        q_des = self.arm_controller.qpos.copy()
        if self._go_zero_requested:
            q_des[: len(self.arm_joint_names)] = [0.0] * len(self.arm_joint_names)
        elif self.active.get("right_arm", False):
            q_des[: len(self.arm_joint_names)] = list(
                self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]].copy()
            )[: len(self.arm_joint_names)]

        if "right_arm" in self.gripper_pos_target:
            joint_name = self.manipulator_config["right_arm"]["gripper_config"]["joint_names"][0]
            q_des[6] = self.gripper_pos_target["right_arm"][joint_name]

        q_des = self._smooth_joint_command(q_des)
        self.arm_controller.set_joint_positions(q_des)

        if self._go_zero_requested and self._is_near_zero():
            self._go_zero_requested = False
            print("Piper go-zero target reached.")

    def _handle_a_button_press(self):
        self._go_zero_requested = True
        self.ref_ee_xyz["right_arm"] = None
        self.ref_controller_xyz["right_arm"] = None
        self.ref_controller_quat["right_arm"] = None
        try:
            self.arm_controller.go_zero(speed=self.piper_joint_speed)
        except Exception as exc:
            print(f"WARNING: Piper go_zero command failed: {exc}")
        print("A button pressed: Piper is returning to zero position.")

    def _is_near_zero(self) -> bool:
        current = np.array(self.arm_controller.qpos[: len(self.arm_joint_names)], dtype=float)
        return bool(np.max(np.abs(current)) < 0.03)

    def _smooth_joint_command(self, q_target: List[float]) -> List[float]:
        if self._smoothed_q_des is None or len(self._smoothed_q_des) != len(q_target):
            self._smoothed_q_des = list(self.arm_controller.qpos)

        filtered = np.array(self._smoothed_q_des, dtype=float)
        target = np.array(q_target, dtype=float)

        if self.command_filter_alpha < 1.0:
            target = filtered + self.command_filter_alpha * (target - filtered)

        max_steps = np.full(len(target), np.inf)
        if self.max_joint_velocity_rad_s > 0:
            max_steps[: len(self.arm_joint_names)] = self.max_joint_velocity_rad_s * self.dt
        if len(target) > 6 and self.max_gripper_velocity_m_s > 0:
            max_steps[6] = self.max_gripper_velocity_m_s * self.dt

        delta = np.clip(target - filtered, -max_steps, max_steps)
        smoothed = filtered + delta
        self._smoothed_q_des = smoothed.tolist()
        return self._smoothed_q_des

    def _get_robot_state_for_logging(self) -> Dict:
        return {
            "qpos": self.arm_controller.qpos,
            "qvel": self.arm_controller.qvel,
            "effort": self.arm_controller.effort,
            "qpos_des": self.arm_controller.q_des,
        }

    def _get_camera_frame_for_logging(self) -> Dict:
        return {}

    def _should_keep_running(self) -> bool:
        return super()._should_keep_running() and ros2.ok()

    def _shutdown_robot(self):
        if self.disable_on_shutdown:
            try:
                self.arm_controller.disable()
            except Exception:
                pass
        self.arm_controller.stop()
        print("Piper SDK controller stopped.")
