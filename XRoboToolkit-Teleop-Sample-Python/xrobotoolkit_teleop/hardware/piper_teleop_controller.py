import os
from typing import Dict

import numpy as np

from xrobotoolkit_teleop.common.base_hardware_teleop_controller import (
    HardwareTeleopController,
)
from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2
from xrobotoolkit_teleop.hardware.interface.piper import PiperSingleArmController
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD
from xrobotoolkit_teleop.utils.path_utils import namespaced_topic, resolve_ros_package_uris_in_urdf

DEFAULT_PIPER_URDF_PATH = os.environ.get(
    "PIPER_URDF_PATH",
    "/home/piper_ros/src/piper_description/urdf/piper_description.urdf",
)
DEFAULT_SCALE_FACTOR = 1.0
DEFAULT_PIPER_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
DEFAULT_PIPER_ARM_JOINT_NAMES = DEFAULT_PIPER_JOINT_NAMES[:6]

DEFAULT_MANIPULATOR_CONFIG = {
    "right_arm": {
        "link_name": "link6",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["joint7"],
            "open_pos": [0.0],
            "close_pos": [0.08],
        },
    },
}


class PiperTeleopController(HardwareTeleopController):
    def __init__(
        self,
        robot_urdf_path: str = DEFAULT_PIPER_URDF_PATH,
        manipulator_config: dict = DEFAULT_MANIPULATOR_CONFIG,
        joint_names: list[str] = DEFAULT_PIPER_JOINT_NAMES,
        arm_joint_names: list[str] = DEFAULT_PIPER_ARM_JOINT_NAMES,
        joint_control_topic: str = "/joint_states",
        joint_state_topic: str = "/joint_states_single",
        enable_topic: str = "/enable_flag",
        namespace: str = "",
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        scale_factor: float = DEFAULT_SCALE_FACTOR,
        visualize_placo: bool = False,
        control_rate_hz: int = 100,
        enable_log_data: bool = True,
        log_dir: str = "logs/piper",
        log_freq: float = 50,
        enable_camera: bool = False,
        camera_fps: int = 30,
        auto_enable: bool = True,
        disable_on_shutdown: bool = False,
    ):
        self.joint_names = joint_names
        self.arm_joint_names = arm_joint_names
        self.joint_control_topic = namespaced_topic(namespace, joint_control_topic)
        self.joint_state_topic = namespaced_topic(namespace, joint_state_topic)
        self.enable_topic = namespaced_topic(namespace, enable_topic)
        self.auto_enable = auto_enable
        self.disable_on_shutdown = disable_on_shutdown
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
        ros2.ensure_node("piper_teleop_controller")

        self.arm_controller = PiperSingleArmController(
            joint_control_topic=self.joint_control_topic,
            joint_state_topic=self.joint_state_topic,
            enable_topic=self.enable_topic,
            rate_hz=1.0 / self.dt,
            auto_enable=self.auto_enable,
            joint_names=self.joint_names,
        )

        print("Waiting for initial joint states from Piper...")
        while ros2.ok() and self.arm_controller.timestamp == 0:
            ros2.sleep(0.1)
        print("Piper arm controller received initial state.")

    def _initialize_camera(self):
        self.camera_interface = None

    def _get_rosbag_topics(self) -> list[str]:
        return [self.joint_state_topic]

    def _update_robot_state(self):
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
            gripper_joint_name = self.manipulator_config["right_arm"]["gripper_config"]["joint_names"][0]
            gripper_target = self.gripper_pos_target["right_arm"][gripper_joint_name]
            if len(q_des) < 7:
                q_des.extend([0.0] * (7 - len(q_des)))
            q_des[6] = gripper_target

        self.arm_controller.set_joint_positions(q_des)
        self.arm_controller.publish_joint_control()

        if self._go_zero_requested and self._is_near_zero():
            self._go_zero_requested = False
            print("Piper go-zero target reached.")

    def _handle_a_button_press(self):
        self._go_zero_requested = True
        self.ref_ee_xyz["right_arm"] = None
        self.ref_controller_xyz["right_arm"] = None
        print("A button pressed: Piper is returning to zero position.")

    def _is_near_zero(self) -> bool:
        current = np.array(self.arm_controller.qpos[: len(self.arm_joint_names)], dtype=float)
        return bool(np.max(np.abs(current)) < 0.03)

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
                self.arm_controller.set_enable(False)
            except Exception:
                pass
        self.arm_controller.stop()
        print("Piper arm controller stopped.")
