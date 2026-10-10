from typing import Dict

import numpy as np
from sensor_msgs.msg import JointState

from xrobotoolkit_teleop.common.base_hardware_teleop_controller import (
    HardwareTeleopController,
)
from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2
from xrobotoolkit_teleop.hardware.interface.piper_sdk_interface import PiperSDKInterface
from xrobotoolkit_teleop.hardware.piper_dual_teleop_controller import (
    DEFAULT_DUAL_PIPER_MANIPULATOR_CONFIG,
)
from xrobotoolkit_teleop.hardware.piper_teleop_controller import DEFAULT_PIPER_URDF_PATH
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD
from xrobotoolkit_teleop.utils.path_utils import build_dual_arm_urdf_from_single, namespaced_topic


class PiperSDKDualTeleopController(HardwareTeleopController):
    def __init__(
        self,
        single_arm_urdf_path: str = DEFAULT_PIPER_URDF_PATH,
        manipulator_config: dict = DEFAULT_DUAL_PIPER_MANIPULATOR_CONFIG,
        left_can_name: str = "can0",
        right_can_name: str = "can1",
        left_namespace: str = "/left_piper",
        right_namespace: str = "/right_piper",
        judge_flag: bool = True,
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        scale_factor: float = 1.0,
        arm_spacing: float = 0.70,
        visualize_placo: bool = False,
        control_rate_hz: int = 100,
        enable_log_data: bool = True,
        log_dir: str = "logs/piper_dual",
        log_freq: float = 50,
        enable_camera: bool = False,
        camera_fps: int = 30,
        auto_enable: bool = True,
    ):
        self.left_can_name = left_can_name
        self.right_can_name = right_can_name
        self.left_namespace = left_namespace
        self.right_namespace = right_namespace
        self.judge_flag = judge_flag
        self.auto_enable = auto_enable
        self.arm_joint_names = {
            "left_arm": [f"left_joint{i}" for i in range(1, 7)],
            "right_arm": [f"right_joint{i}" for i in range(1, 7)],
        }
        self.robot_urdf_path_resolved = build_dual_arm_urdf_from_single(
            single_arm_urdf_path,
            left_prefix="left_",
            right_prefix="right_",
            left_mount_xyz=f"0 {arm_spacing / 2.0:.4f} 0",
            right_mount_xyz=f"0 {-arm_spacing / 2.0:.4f} 0",
        )

        super().__init__(
            robot_urdf_path=self.robot_urdf_path_resolved,
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
        for arm_name, joint_names in self.arm_joint_names.items():
            self.placo_arm_joint_slice[arm_name] = slice(
                self.placo_robot.get_joint_offset(joint_names[0]),
                self.placo_robot.get_joint_offset(joint_names[-1]) + 1,
            )

    def _robot_setup(self):
        self._ros_node = ros2.ensure_node("piper_sdk_dual_teleop_controller")
        self.arm_controllers = {
            "left_arm": PiperSDKInterface(
                can_name=self.left_can_name,
                judge_flag=self.judge_flag,
                auto_enable=self.auto_enable,
                gripper_exist=True,
            ),
            "right_arm": PiperSDKInterface(
                can_name=self.right_can_name,
                judge_flag=self.judge_flag,
                auto_enable=self.auto_enable,
                gripper_exist=True,
            ),
        }
        self.state_publishers = {
            "left_arm": self._ros_node.create_publisher(
                JointState,
                namespaced_topic(self.left_namespace, "joint_states_single"),
                1,
            ),
            "right_arm": self._ros_node.create_publisher(
                JointState,
                namespaced_topic(self.right_namespace, "joint_states_single"),
                1,
            ),
        }
        for controller in self.arm_controllers.values():
            controller.refresh_state()

    def _initialize_camera(self):
        self.camera_interface = None

    def _get_rosbag_topics(self) -> list[str]:
        return [
            namespaced_topic(self.left_namespace, "joint_states_single"),
            namespaced_topic(self.right_namespace, "joint_states_single"),
        ]

    def _publish_joint_state_for_logging(self, arm_name: str, controller: PiperSDKInterface):
        msg = JointState()
        msg.header.stamp = ros2.now_msg(self._ros_node)
        prefix = "left" if arm_name == "left_arm" else "right"
        msg.name = [f"{prefix}_joint{i}" for i in range(1, 7)] + [f"{prefix}_joint7"]
        msg.position = list(controller.qpos)
        msg.velocity = list(controller.qvel)
        msg.effort = list(controller.effort)
        self.state_publishers[arm_name].publish(msg)

    def _update_robot_state(self):
        for arm_name, controller in self.arm_controllers.items():
            controller.refresh_state()
            self._publish_joint_state_for_logging(arm_name, controller)
            self.placo_robot.state.q[self.placo_arm_joint_slice[arm_name]] = controller.qpos[:6]

    def _send_command(self):
        for arm_name, controller in self.arm_controllers.items():
            q_des = controller.qpos.copy()
            if self.active.get(arm_name, False):
                q_des[:6] = list(self.placo_robot.state.q[self.placo_arm_joint_slice[arm_name]].copy())[:6]

            if arm_name in self.gripper_pos_target:
                joint_name = self.manipulator_config[arm_name]["gripper_config"]["joint_names"][0]
                q_des[6] = self.gripper_pos_target[arm_name][joint_name]

            controller.set_joint_positions(q_des)

    def _get_robot_state_for_logging(self) -> Dict:
        return {
            arm_name: {
                "qpos": controller.qpos,
                "qvel": controller.qvel,
                "effort": controller.effort,
                "qpos_des": controller.q_des,
            }
            for arm_name, controller in self.arm_controllers.items()
        }

    def _get_camera_frame_for_logging(self) -> Dict:
        return {}

    def _should_keep_running(self) -> bool:
        return super()._should_keep_running() and ros2.ok()

    def _shutdown_robot(self):
        for controller in self.arm_controllers.values():
            try:
                controller.disable()
            except Exception:
                pass
            controller.stop()
        print("Dual Piper SDK controllers stopped.")
