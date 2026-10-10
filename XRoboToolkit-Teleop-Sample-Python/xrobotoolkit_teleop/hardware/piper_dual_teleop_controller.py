from typing import Dict

import numpy as np

from xrobotoolkit_teleop.common.base_hardware_teleop_controller import (
    HardwareTeleopController,
)
from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2
from xrobotoolkit_teleop.hardware.interface.piper import PiperSingleArmController
from xrobotoolkit_teleop.hardware.piper_teleop_controller import DEFAULT_PIPER_URDF_PATH
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD
from xrobotoolkit_teleop.utils.path_utils import (
    build_dual_arm_urdf_from_single,
    namespaced_topic,
)

DEFAULT_DUAL_PIPER_MANIPULATOR_CONFIG = {
    "right_arm": {
        "link_name": "right_link6",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["right_joint7"],
            "open_pos": [0.0],
            "close_pos": [0.035],
        },
    },
    "left_arm": {
        "link_name": "left_link6",
        "pose_source": "left_controller",
        "control_trigger": "left_grip",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "left_trigger",
            "joint_names": ["left_joint7"],
            "open_pos": [0.0],
            "close_pos": [0.035],
        },
    },
}


class PiperDualTeleopController(HardwareTeleopController):
    def __init__(
        self,
        single_arm_urdf_path: str = DEFAULT_PIPER_URDF_PATH,
        manipulator_config: dict = DEFAULT_DUAL_PIPER_MANIPULATOR_CONFIG,
        left_namespace: str = "/left_piper",
        right_namespace: str = "/right_piper",
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
        self.left_namespace = left_namespace
        self.right_namespace = right_namespace
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
        ros2.ensure_node("piper_dual_teleop_controller")

        self.arm_controllers = {
            "left_arm": PiperSingleArmController(
                joint_control_topic=namespaced_topic(self.left_namespace, "joint_states"),
                joint_state_topic=namespaced_topic(self.left_namespace, "joint_states_single"),
                enable_topic=namespaced_topic(self.left_namespace, "enable_flag"),
                rate_hz=1.0 / self.dt,
                auto_enable=self.auto_enable,
            ),
            "right_arm": PiperSingleArmController(
                joint_control_topic=namespaced_topic(self.right_namespace, "joint_states"),
                joint_state_topic=namespaced_topic(self.right_namespace, "joint_states_single"),
                enable_topic=namespaced_topic(self.right_namespace, "enable_flag"),
                rate_hz=1.0 / self.dt,
                auto_enable=self.auto_enable,
            ),
        }

        print("Waiting for initial joint states from both Piper arms...")
        while ros2.ok():
            ready = all(controller.timestamp != 0 for controller in self.arm_controllers.values())
            if ready:
                break
            ros2.sleep(0.1)
        print("Both Piper arm controllers received initial state.")

    def _initialize_camera(self):
        self.camera_interface = None

    def _get_rosbag_topics(self) -> list[str]:
        return [
            namespaced_topic(self.left_namespace, "joint_states_single"),
            namespaced_topic(self.right_namespace, "joint_states_single"),
        ]

    def _update_robot_state(self):
        for arm_name, controller in self.arm_controllers.items():
            self.placo_robot.state.q[self.placo_arm_joint_slice[arm_name]] = controller.qpos[:6]

    def _send_command(self):
        for arm_name, controller in self.arm_controllers.items():
            q_des = controller.qpos.copy()

            if self.active.get(arm_name, False):
                q_des[:6] = list(self.placo_robot.state.q[self.placo_arm_joint_slice[arm_name]].copy())[:6]

            if arm_name in self.gripper_pos_target:
                gripper_joint_name = self.manipulator_config[arm_name]["gripper_config"]["joint_names"][0]
                gripper_target = self.gripper_pos_target[arm_name][gripper_joint_name]
                if len(q_des) < 7:
                    q_des.extend([0.0] * (7 - len(q_des)))
                q_des[6] = gripper_target

            controller.set_joint_positions(q_des)
            controller.publish_joint_control()

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
                controller.set_enable(False)
            except Exception:
                pass
            controller.stop()
        print("Dual Piper arm controllers stopped.")
