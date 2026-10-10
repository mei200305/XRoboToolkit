import os
from typing import Dict

import numpy as np

from xrobotoolkit_teleop.common.base_hardware_teleop_controller import (
    HardwareTeleopController,
)
from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2
from xrobotoolkit_teleop.hardware.interface.qj2 import (
    QJ2DualArmController,
    R1LiteChassisController,
    R1LiteTorsoController,
)
from xrobotoolkit_teleop.hardware.interface.ros_camera import RosCameraInterface
from xrobotoolkit_teleop.utils.geometry import (
    R_HEADSET_TO_WORLD,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH

# Default paths and configurations for QJ2 dual arm
DEFAULT_DUAL_A1X_URDF_PATH = os.path.join(ASSET_PATH, "qj2/urdf/arms_gen2_hand.urdf")
DEFAULT_SCALE_FACTOR = 1.0
CONTROLLER_DEADZONE = 0.1

# QJ2 robot has both arms with 7 joints each
# End effector links are r_hand_base_Link and l_hand_base_Link
DEFAULT_MANIPULATOR_CONFIG = {
    "right_arm": {
        "link_name": "r_hand_base_Link",  # End effector link for right arm
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "gripper_config": {
            "type": "parallel",  # Robotiq gripper type
            "gripper_trigger": "right_trigger",
            "joint_names": ["r_gripper_finger_joint"],  # Robotiq doesn't use joint names
            "open_pos": [0.0],  # 0.0 = open (maps to 0 in Robotiq 0-255)
            "close_pos": [1.0],  # 1.0 = close (maps to 255 in Robotiq 0-255)
        },
    },
    "left_arm": {
        "link_name": "l_hand_base_Link",  # End effector link for left arm
        "pose_source": "left_controller",
        "control_trigger": "left_grip",
        "gripper_config": {
            "type": "parallel",  # Robotiq gripper type
            "gripper_trigger": "left_trigger",
            "joint_names": ["l_gripper_finger_joint"],  # Robotiq doesn't use joint names
            "open_pos": [0.0],  # 0.0 = open
            "close_pos": [1.0],  # 1.0 = close
        },
    },
}


class QJ2TeleopController(HardwareTeleopController):
    def __init__(
        self,
        robot_urdf_path: str = DEFAULT_DUAL_A1X_URDF_PATH,
        manipulator_config: dict = DEFAULT_MANIPULATOR_CONFIG,
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        scale_factor: float = DEFAULT_SCALE_FACTOR,
        chassis_velocity_scale: list[float] = [0.75, 0.75, 1.0],
        visualize_placo: bool = False,
        control_rate_hz: int = 100,
        enable_log_data: bool = True,
        log_dir: str = "logs/qj2",
        log_freq: float = 50,
        enable_camera: bool = False,  # Disable by default, can enable if camera topics available
        camera_fps: int = 30,
    ):
        super().__init__(
            robot_urdf_path=robot_urdf_path,
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
        self.chassis_velocity_scale = chassis_velocity_scale

    def _placo_setup(self):
        super()._placo_setup()
        # QJ2 has both left and right arms with 7 joints each
        self.placo_arm_joint_slice = {}
        for arm_name in ["left_arm", "right_arm"]:
            config = self.manipulator_config[arm_name]
            ee_link_name = config["link_name"]
            # Extract arm prefix: "r_hand_base_Link" -> "r_arm_", "l_hand_base_Link" -> "l_arm_"
            if ee_link_name.startswith("r_"):
                arm_prefix = "r_arm_"
            elif ee_link_name.startswith("l_"):
                arm_prefix = "l_arm_"
            else:
                raise ValueError(f"Unknown arm prefix for link {ee_link_name}")
            
            arm_joint_names = [f"{arm_prefix}Joint{i}" for i in range(1, 8)]  # Joint1 to Joint7
            self.placo_arm_joint_slice[arm_name] = slice(
                self.placo_robot.get_joint_offset(arm_joint_names[0]),
                self.placo_robot.get_joint_offset(arm_joint_names[-1]) + 1,
            )

    def _robot_setup(self):
        ros2.ensure_node("qj2_teleop_controller")

        # Setup dual arm controller (handles both arms together)
        self.arm_controller = QJ2DualArmController(
            arm_control_topic="/motor_command/arm_position_control",
            left_gripper_topic="Robotiq2FGripperRobotOutput",
            right_gripper_topic="Robotiq2FGripperRobotOutputRight",
            arm_state_topic="/motor_state/arm_motor_state_actual",
            rate_hz=1.0 / self.dt,
        )

        print("Waiting for initial joint states from QJ2 robot...")
        while ros2.ok() and self.arm_controller.timestamp == 0:
            ros2.sleep(0.1)
        print("QJ2 arm controller received initial state.")

        # Note: QJ2 robot may not have chassis/torso, so these are optional
        # Uncomment if your robot has these components
        # Setup chassis controller (optional)
        # self.chassis_controller = R1LiteChassisController(
        #     chassis_state_topic="/hdas/feedback_chassis",
        #     chassis_control_topic="/motion_target/target_speed_chassis",
        #     rate_hz=1.0 / self.dt,
        # )
        # print("Waiting for initial chassis state...")
        # while ros2.ok() and self.chassis_controller.timestamp == -1:
        #     ros2.sleep(0.1)
        # print("Chassis controller received initial state.")

        # self.torso_controller = R1LiteTorsoController(
        #     torso_state_topic="/hdas/feedback_torso",
        #     torso_control_topic="/motion_target/target_speed_torso",
        #     rate_hz=1.0 / self.dt,
        # )
        # print("Waiting for initial torso state...")
        # while ros2.ok() and self.torso_controller.timestamp == -1:
        #     ros2.sleep(0.1)
        # print("Torso controller received initial state.")

    def _initialize_camera(self):
        if self.enable_camera:
            print("Initializing camera...")
            try:
                # Update these topics according to your robot's camera setup
                camera_topics = {
                    "left": {
                        "color": "/camera_left/color/image_raw/compressed",
                    },
                    "right": {
                        "color": "/camera_right/color/image_raw/compressed",
                    },
                }
                self.camera_interface = RosCameraInterface(
                    camera_topics=camera_topics, width=424, height=240, enable_depth=False
                )
                self.camera_interface.start()
                print("Camera initialized successfully.")
            except Exception as e:
                print(f"Error initializing camera: {e}")
                self.enable_camera = False

    def _update_robot_state(self):
        """Reads current joint states from QJ2 controller and updates Placo."""
        # Get left and right arm states (7 joints each)
        left_qpos = self.arm_controller.get_left_arm_state()
        right_qpos = self.arm_controller.get_right_arm_state()
        
        # Update Placo state
        self.placo_robot.state.q[self.placo_arm_joint_slice["left_arm"]] = left_qpos
        self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]] = right_qpos

    def _send_command(self):
        """Sends the solved joint targets to QJ2 controller."""
        # Update desired positions for active arms
        if self.active.get("left_arm", False):
            self.arm_controller.q_des_left = self.placo_robot.state.q[
                self.placo_arm_joint_slice["left_arm"]
            ].copy()[:7]  # 7 joints
        
        if self.active.get("right_arm", False):
            self.arm_controller.q_des_right = self.placo_robot.state.q[
                self.placo_arm_joint_slice["right_arm"]
            ].copy()[:7]  # 7 joints

        # Publish arm control (both arms together)
        self.arm_controller.publish_arm_control()

        # Update gripper positions based on gripper targets
        # Convert from normalized [0, 1] to Robotiq format
        left_gripper_pos = None
        right_gripper_pos = None
        
        for arm_name in ["left_arm", "right_arm"]:
            if arm_name in self.gripper_pos_target:
                # Get gripper target (should be normalized 0-1)
                gripper_targets = list(self.gripper_pos_target[arm_name].values())
                if gripper_targets:
                    gripper_val = gripper_targets[0]  # Take first value
                    # Map from config range to 0-1
                    config = self.manipulator_config[arm_name]["gripper_config"]
                    open_pos = config["open_pos"][0]
                    close_pos = config["close_pos"][0]
                    if close_pos != open_pos:
                        normalized = (gripper_val - open_pos) / (close_pos - open_pos)
                        normalized = max(0.0, min(1.0, normalized))  # Clamp to [0, 1]
                    else:
                        normalized = 0.0
                    
                    if arm_name == "left_arm":
                        left_gripper_pos = normalized
                    else:
                        right_gripper_pos = normalized

        # Set gripper positions
        self.arm_controller.set_gripper_position(
            left_pos=left_gripper_pos,
            right_pos=right_gripper_pos
        )
        self.arm_controller.publish_gripper_control()

        # Publish chassis and torso control if available
        # if hasattr(self, 'chassis_controller'):
        #     self.chassis_controller.publish_chassis_control()
        # if hasattr(self, 'torso_controller'):
        #     self.torso_controller.publish_torso_control()

    def _pre_ik_update(self):
        """Updates the chassis and torso velocity commands based on joystick input."""
        # Uncomment if chassis/torso are available
        # self._update_joystick_velocity_command()
        # self._update_torso_velocity_command()
        pass

    def _update_joystick_velocity_command(self):
        """Updates the chassis velocity commands based on joystick input."""
        if not hasattr(self, 'chassis_controller'):
            return
        
        left_axis = self.xr_client.get_joystick_state("left")
        right_axis = self.xr_client.get_joystick_state("right")

        vx = left_axis[1] * self.chassis_velocity_scale[0]
        vy = -left_axis[0] * self.chassis_velocity_scale[1]
        omega = -right_axis[0] * self.chassis_velocity_scale[2]

        self.chassis_controller.set_velocity_command(vx, vy, omega)

    def _update_torso_velocity_command(self):
        """Updates the torso velocity commands based on button input."""
        if not hasattr(self, 'torso_controller'):
            return
        
        buttonY = self.xr_client.get_button_state_by_name("Y")
        buttonX = self.xr_client.get_button_state_by_name("X")

        vz = 2.5 if buttonY else -2.5 if buttonX else 0.0
        self.torso_controller.set_velocity_command(vz)

    def _get_robot_state_for_logging(self) -> Dict:
        """Returns a dictionary of robot-specific data for logging."""
        return {
            "qpos_left": self.arm_controller.qpos_left,
            "qpos_right": self.arm_controller.qpos_right,
            "qvel_left": self.arm_controller.qvel_left,
            "qvel_right": self.arm_controller.qvel_right,
            "qpos_des_left": self.arm_controller.q_des_left if self.arm_controller.q_des_left else None,
            "qpos_des_right": self.arm_controller.q_des_right if self.arm_controller.q_des_right else None,
            "gripper_left": self.arm_controller.left_gripper_pos,
            "gripper_right": self.arm_controller.right_gripper_pos,
        }

    def _get_camera_frame_for_logging(self) -> Dict:
        """Returns a dictionary of camera frames for logging with camera names as keys."""
        if not self.camera_interface:
            return {}

        # Use compressed frames for logging to reduce file size
        if self.camera_interface.enable_compression:
            frames = self.camera_interface.get_compressed_frames()
        else:
            # Fallback to regular frames for compatibility
            frames = self.camera_interface.get_frames()

        # ROS camera interface already uses camera names as keys, so no mapping needed
        return frames if frames else {}

    def _should_keep_running(self) -> bool:
        """Returns True if the main loop should continue running."""
        return super()._should_keep_running() and ros2.ok()

    def _shutdown_robot(self):
        """Performs graceful shutdown of the robot hardware."""
        self.arm_controller.stop()
        print("QJ2 arm controller stopped.")
        
        # if hasattr(self, 'torso_controller'):
        #     self.torso_controller.stop_torso()
        #     print("Torso stopped.")
        # if hasattr(self, 'chassis_controller'):
        #     self.chassis_controller.stop_chassis()
        #     print("Chassis stopped.")
