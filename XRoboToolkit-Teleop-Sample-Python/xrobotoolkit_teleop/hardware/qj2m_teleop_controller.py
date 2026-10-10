"""
QJ2M (QJ Full Humanoid) Teleoperation Controller — Real Hardware.

Uses the real QJ2M ROS interface:
  Command:  /throw/arm_joint_command  (JointState, 28D pos + 28D vel + kp/kd)
  State:    /motor/q2w_upper_motor_joint_state (JointState, 32 motors)
  Hand:     /chatter1 (left), /chatter1_right (right)  (Int32MultiArray, 6 finger joints)
            Trigger continuous value [0,1] → linear interp DEX_HAND_OPEN ↔ DEX_HAND_CLOSE
Gripper:  Xense left 192.168.99.30 / right 192.168.98.3
            Same trigger: 0 = open (85 mm), 1 = closed (0 mm)

Left joystick up: one-key homing (left arm first, then right; each J1–J3 → J4–J7).
Left joystick down: both arms simultaneous interp to measured stow pose (5s).
Left joystick right: DISABLED (CMD0→CMD→CMD2 path commented out).
Right joystick up: toggle left-follow mode (right Pico world X/Z; Y and orientations locked).
Right joystick down: reset yaw alignment.
X-button: lock/unlock LEFT hand trigger control (hold last position when locked).
B-button: start one rosbag; press again to stop and save.
  Every topic below is published once per D435i color frame (30 Hz) with the same header
  stamp, the color exposure time. Faster streams use the sample nearest that time.
  /vla/{left,right}_arm/joint_state and joint_command (JointState, 7)
  /xense/gripper/usb and /xense/gripper/nic state, position_mm (Float64, mm)
  /xense/tactile/{usb,nic}/<serial>/image
  /camera/d435i/color/image_raw (640x480 bgr8)
  /camera/d435i/aligned_depth_to_color/image_raw (640x480 16UC1, mm)
  /force/left_arm, /wrench_filtered_left (left wrist)
  /force/right_arm, /wrench_filtered_right, /force (right wrist) in the same bag.
Y-button: toggle hand open mode between:
  MODE1 (default): joints 0,1 stay closed [1000,1000,0,0,0,700]
  MODE2: joints 0,1 also fully open [0,0,0,0,0,700]
"""
import os
import sys
import threading
import time
from array import array
from collections import deque
from typing import Dict

import meshcat.transformations as tf
import numpy as np
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, Int32MultiArray

from xrobotoolkit_teleop.common.base_hardware_teleop_controller import HardwareTeleopController
from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2
from xrobotoolkit_teleop.hardware.interface.qj2m import QJ2MDualArmController
from xrobotoolkit_teleop.hardware.interface.force_sensor import (
    FORCE_ARMS,
    WristForceSensors,
    force_record_topics,
    round_half_away,
)
from xrobotoolkit_teleop.hardware.interface.xense_gripper import (
    XENSE_GRIPPER_TOPICS,
    XENSE_TACTILE_POLL_HZ,
    XenseTeleopGrippers,
    xense_record_topics,
)
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD
from xrobotoolkit_teleop.utils.path_utils import QJ2M_URDF_PATH
from xrobotoolkit_teleop.utils.qj2m_homing import (
    CMD0_DURATION,
    CMD0_Q_LEFT,
    CMD0_Q_RIGHT,
    CMD2_DURATION,
    CMD2_Q_LEFT,
    CMD2_Q_RIGHT,
    CMD_DURATION,
    CMD_Q_LEFT,
    CMD_Q_RIGHT,
    HOME_Q_LEFT as QJ2M_HOME_Q_LEFT,
    HOME_Q_RIGHT as QJ2M_HOME_Q_RIGHT,
    HOMING_DURATION,
    JOINTS_TASK_WEIGHT,
    STOW_DURATION,
    STOW_Q_LEFT,
    STOW_Q_RIGHT,
    pref_joints_dict,
    proximal_first_via,
    sample_left_then_right_homing,
    sample_simultaneous_arms,
    sample_simultaneous_three_waypoint,
    sample_simultaneous_two_waypoint,
    shortest_angular_path_safe as _shortest_angular_path_safe,
)

DEFAULT_QJ2M_URDF_PATH = QJ2M_URDF_PATH

# Joint names match the QJ2M_URDF_PATH teleop URDF (suffix _joint to avoid link-name clash)
QJ2M_LEFT_ARM_JOINTS = [
    "arml_1_joint", "arml_2_joint", "arml_3_joint", "arml_4_joint",
    "arml_5_joint", "arml_6_joint", "arml_7_joint",
]
QJ2M_RIGHT_ARM_JOINTS = [
    "armr_1_joint", "armr_2_joint", "armr_3_joint", "armr_4_joint",
    "armr_5_joint", "armr_6_joint", "armr_7_joint",
]

# CSP is stiff and the motor stack runs it at 500 Hz, so a 50 Hz staircase of targets
# is heard as a buzz at the step rate. Commands go out faster than IK and are low-passed.
QJ2M_COMMAND_RATE_HZ = 200
# Joint low-pass time constant in seconds; 0.046 equals the old per-tick alpha 0.35 at 50 Hz.
JOINT_SMOOTH_TAU = 0.046

VLA_ARM_TOPICS = {
    "left_state": "/vla/left_arm/joint_state",
    "right_state": "/vla/right_arm/joint_state",
    "left_command": "/vla/left_arm/joint_command",
    "right_command": "/vla/right_arm/joint_command",
}

# D435IF on USB 2.1, serial from rs-enumerate-devices.
# Depth is aligned to color so both images are 640x480 and share a stamp.
D435I_SERIAL = "243622070764"
D435I_COLOR_TOPIC = "/camera/d435i/color/image_raw"
D435I_DEPTH_TOPIC = "/camera/d435i/aligned_depth_to_color/image_raw"
D435I_WIDTH = 640
D435I_HEIGHT = 480
D435I_FPS = 30

# Rosbag master clock. Driven by D435i color frames; a timer takes over without the camera.
VLA_FRAME_HZ = D435I_FPS
# Arm (50 Hz) and force (100 Hz) samples farther than this from a frame are left out.
VLA_MAX_SAMPLE_AGE_S = 0.1
# Tactile images are held until this old; get_img is slow while nothing is gripped.
VLA_TACTILE_MAX_AGE_S = 1.0
# A color timestamp this far from the host clock is not in the global time domain.
D435I_MAX_CLOCK_SKEW_S = 0.5


class _SampleHistory:
    """Short (time, value) buffer so a frame can take the sample nearest its capture time."""

    def __init__(self, maxlen: int):
        self._samples = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def append(self, t: float, value) -> None:
        with self._lock:
            self._samples.append((t, value))

    def nearest(self, t: float, max_age: float):
        with self._lock:
            samples = list(self._samples)
        best = None
        best_dt = max_age
        for sample_t, value in reversed(samples):
            dt = abs(sample_t - t)
            if dt <= best_dt:
                best, best_dt = value, dt
            elif sample_t < t:
                break
        return best


# Joint limits (radians) — order matches QJ2M_LEFT_ARM_JOINTS / QJ2M_RIGHT_ARM_JOINTS
# Elbow (J4) ±130° ≈ ±2.2689 rad
_ELBOW_LIMIT = 120.0 * np.pi / 180.0
QJ2M_LEFT_ARM_LOWER = [-3.14, -1.3, -3.14, -_ELBOW_LIMIT, -3.14, -1.91, -1.4]
QJ2M_LEFT_ARM_UPPER = [1.57, 3.66, 3.14, _ELBOW_LIMIT, 3.14, 1.91, 1.4]
QJ2M_RIGHT_ARM_LOWER = [-3.14, -3.66, -3.14, -_ELBOW_LIMIT, -3.14, -1.91, -1.4]
QJ2M_RIGHT_ARM_UPPER = [1.57, 1.3, 3.14, _ELBOW_LIMIT, 3.14, 1.91, 1.4]

MOTOR_STATE_TIMEOUT = 3.0  # seconds — exit if no motor state received within this window

# Dexterous hand joint limits (0 = fully open, 1000 = fully closed)
# Last joint fixed at 700
DEX_HAND_OPEN_MODE1 = [1000, 1000, 0, 0, 0, 700]  # joints 0,1 stay closed
DEX_HAND_OPEN_MODE2 = [0, 0, 0, 0, 0, 700]  # joints 0,1 also fully open
DEX_HAND_CLOSE = [1000, 1000, 1000, 1000, 1000, 700]


def _interp_hand_pos(trigger_val: float, hand_open: list) -> list:
    """Two-phase trigger mapping:
    - Phase 1 [0, 0.5]: joints 2, 3 close (0 → 1000)
    - Phase 2 [0.5, 1.0]: joint 4 closes (0 → 1000)
    Joint 5 always 700.  Joints 0,1 stay at hand_open[0], hand_open[1].
    """
    trigger_val = max(0.0, min(1.0, trigger_val))

    if trigger_val <= 0.5:
        phase1 = trigger_val / 0.5  # [0, 1] during first half
        phase2 = 0.0
    else:
        phase1 = 1.0
        phase2 = (trigger_val - 0.5) / 0.5  # [0, 1] during second half

    return [
        int(hand_open[0] + phase1 * (DEX_HAND_CLOSE[0] - hand_open[0])),  # joint 0
        int(hand_open[1] + phase1 * (DEX_HAND_CLOSE[1] - hand_open[1])),  # joint 1
        int(hand_open[2] + phase1 * (DEX_HAND_CLOSE[2] - hand_open[2])),  # joint 2 — 前半程
        int(hand_open[3] + phase1 * (DEX_HAND_CLOSE[3] - hand_open[3])),  # joint 3 — 前半程
        int(hand_open[4] + phase2 * (DEX_HAND_CLOSE[4] - hand_open[4])),  # joint 4 — 后半程
        int(hand_open[5] + phase1 * (DEX_HAND_CLOSE[5] - hand_open[5])),  # joint 5 — 不变
    ]

DEFAULT_MANIPULATOR_CONFIG = {
    "right_arm": {
        "link_name": "armr_7",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["right_gripper"],
            "open_pos": [0.0],
            "close_pos": [1.0],
        },
    },
    "left_arm": {
        "link_name": "arml_7",
        "pose_source": "left_controller",
        "control_trigger": "left_grip",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "left_trigger",
            "joint_names": ["left_gripper"],
            "open_pos": [0.0],
            "close_pos": [1.0],
        },
    },
}


class QJ2MTeleopController(HardwareTeleopController):
    """Hardware teleop controller for QJ2M using /throw/arm_joint_command.

    Left joystick up: one-key homing — 2-phase (via → WBC default home).
    Left joystick down: both arms simultaneous interp to measured stow pose (5s).
    Left joystick right: DISABLED (CMD0→CMD→CMD2 path commented out).
    Right joystick up: toggle left-follow mode (right Pico world X/Z; Y and orientations locked).
    Right joystick down: reset yaw alignment.
    X-button: lock/unlock LEFT hand trigger control.
    B-button: one rosbag of arm JointState, gripper Float64, tactile images, and D435i color plus depth.
    Y-button: toggle hand open mode (joints 0,1 closed vs all open).
    """

    def __init__(
        self,
        robot_urdf_path: str = DEFAULT_QJ2M_URDF_PATH,
        manipulator_config: dict = DEFAULT_MANIPULATOR_CONFIG,
        R_headset_world: np.ndarray = R_HEADSET_TO_WORLD,
        scale_factor: float = 1.0,
        visualize_placo: bool = False,
        control_rate_hz: int = 50,
        enable_log_data: bool = True,
        log_dir: str = "logs/qj2m",
        log_freq: float = 50,
        enable_camera: bool = False,
        camera_fps: int = 30,
        enable_xense_gripper: bool = True,
        command_rate_hz: int = QJ2M_COMMAND_RATE_HZ,
        joint_smooth_tau: float = JOINT_SMOOTH_TAU,
    ):
        self._enable_xense_gripper = enable_xense_gripper
        self.xense_grippers = None
        self._dataset_pubs_ready = False
        self._tactile_stop = threading.Event()
        self._tactile_thread = None
        self._d435_stop = threading.Event()
        self._d435_thread = None
        self._d435_pub = None
        self._d435_depth_pub = None
        self._force_sensor = None
        self._force_pubs = {}
        # About one second of each stream, so camera pipeline latency stays covered.
        self._arm_history = _SampleHistory(maxlen=64)
        self._force_history = {arm_name: _SampleHistory(maxlen=128) for arm_name in FORCE_ARMS}
        self._tactile_lock = threading.Lock()
        self._latest_tactile = {}
        # Set before super().__init__, which already calls _robot_setup.
        self.command_rate_hz = max(int(command_rate_hz), int(control_rate_hz))
        # Trigger/hand/dataset side effects keep the IK rate even when commands go faster.
        self._aux_period = 1.0 / control_rate_hz
        self._last_aux_time = 0.0
        self._joint_smooth_tau = float(joint_smooth_tau)
        # Arm joints from the last finished IK solve, (left, right). The IK thread
        # overwrites placo state.q with measured joints before every solve, so the
        # control thread must not read state.q directly.
        self._ik_q_latest = None
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
        # Homing / stow motion state
        self._homing = False
        self._homing_mode = "sequential"  # sequential | simultaneous | two_waypoint | three_waypoint
        self._homing_duration = HOMING_DURATION
        self._homing_via_fraction = 0.5
        self._homing_phase_f1 = 1.0 / 3.0
        self._homing_phase_f2 = 1.0 / 3.0
        self._homing_start_time = 0.0
        self._homing_start_q_left = None
        self._homing_start_q_right = None
        self._homing_via_left = None
        self._homing_via_right = None
        self._homing_mid_left = None
        self._homing_mid_right = None
        self._homing_target_left = None
        self._homing_target_right = None

        # Y-button toggle: hand open mode (False → mode1, True → mode2)
        self._hand_open_mode2 = False
        self._prev_y_button_state = False

        # X-button toggle: lock/unlock LEFT hand trigger control
        self._left_hand_locked = False
        self._prev_x_button_state = False

        # Left joystick up → homing; down → stow; right → snapshot CMD
        self._prev_left_stick_up = False
        self._prev_left_stick_down = False
        self._prev_left_stick_right = False

        # Right joystick up → left-follow mode toggle; down → yaw alignment reset
        self._prev_right_stick_up = False
        self._prev_right_stick_down = False

        # Joint-space EMA smoothing (suppress IK solver micro-variations)
        self._ema_q_left = None
        self._ema_q_right = None
        self._ema_last_time = None
        self._ik_q_latest = None

        # Last teleop command while grip was held — re-grip resumes from here
        # instead of measured (possibly gravity-sagged) joint state.
        self._held_q_left = None
        self._held_q_right = None

    # ---- Hold / re-grip continuity ----
    def _get_held_q(self, arm_name: str):
        """Return last held teleop command for an arm, or None."""
        if arm_name == "left_arm":
            if self._held_q_left is not None:
                return self._held_q_left
            if self.arm_ctrl.q_des_left is not None:
                return np.asarray(self.arm_ctrl.q_des_left, dtype=float)[:7]
        else:
            if self._held_q_right is not None:
                return self._held_q_right
            if self.arm_ctrl.q_des_right is not None:
                return np.asarray(self.arm_ctrl.q_des_right, dtype=float)[:7]
        return None

    def _ee_pose_from_held_q(self, arm_name: str, config: dict):
        """FK of the last held joint command → EE pose used for re-grip reference."""
        held = self._get_held_q(arm_name)
        if held is None:
            return self._get_link_pose(config["link_name"])
        arm_slice = self.placo_arm_joint_slice[arm_name]
        saved_q = self.placo_robot.state.q[arm_slice].copy()
        self.placo_robot.state.q[arm_slice] = held
        self.placo_robot.update_kinematics()
        ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
        self.placo_robot.state.q[arm_slice] = saved_q
        self.placo_robot.update_kinematics()
        return ee_xyz, ee_quat

    def _get_activation_ee_pose(self, src_name: str, config: dict):
        # Re-grip: anchor delta teleop at the last commanded pose, not sagged actual.
        return self._ee_pose_from_held_q(src_name, config)

    def _get_inactive_ee_pose(self, src_name: str, config: dict):
        # While grip released, park IK target on the held pose so soft IK
        # does not drift the arm joints toward a lower configuration.
        return self._ee_pose_from_held_q(src_name, config)

    def _post_ik_solve(self):
        # Restore inactive arm joints to the held command after soft IK solve.
        # Otherwise placo state drifts and the next re-grip inherits a drop.
        for arm_name in ("left_arm", "right_arm"):
            if self.active.get(arm_name, False):
                continue
            held = self._get_held_q(arm_name)
            if held is None:
                continue
            self.placo_robot.state.q[self.placo_arm_joint_slice[arm_name]] = held
        q = self.placo_robot.state.q
        self._ik_q_latest = (
            q[self.placo_arm_joint_slice["left_arm"]].copy()[:7],
            q[self.placo_arm_joint_slice["right_arm"]].copy()[:7],
        )

    # ---- Joint limit clamping ----
    @staticmethod
    def _clamp_arm_joints(q_left, q_right):
        """Clamp left and right arm joint positions to URDF limits.

        Returns (clamped_left, clamped_right) — new arrays, originals unchanged.
        """
        clamped_left = np.clip(np.asarray(q_left, dtype=float),
                               QJ2M_LEFT_ARM_LOWER, QJ2M_LEFT_ARM_UPPER)
        clamped_right = np.clip(np.asarray(q_right, dtype=float),
                                QJ2M_RIGHT_ARM_LOWER, QJ2M_RIGHT_ARM_UPPER)
        return clamped_left, clamped_right

    # ---- Placo setup (lock non-arm joints) ----
    def _placo_setup(self):
        super()._placo_setup()

        arm_names = set(QJ2M_LEFT_ARM_JOINTS + QJ2M_RIGHT_ARM_JOINTS)
        locked = 0
        for name in self.placo_robot.model.names:
            if name in ("universe", "root_joint"):
                continue
            if name not in arm_names:
                self.solver.mask_dof(name)
                locked += 1

        self.placo_arm_joint_slice = {}
        for side, joint_list in [("left_arm", QJ2M_LEFT_ARM_JOINTS),
                                  ("right_arm", QJ2M_RIGHT_ARM_JOINTS)]:
            first = self.placo_robot.get_joint_offset(joint_list[0])
            last = self.placo_robot.get_joint_offset(joint_list[-1])
            self.placo_arm_joint_slice[side] = slice(first, last + 1)

        print(f"[QJ2M] Locked {locked} non-arm joints. "
              f"Left: {self.placo_arm_joint_slice['left_arm']}, "
              f"Right: {self.placo_arm_joint_slice['right_arm']}")

        # The URDF arm limits are 0/0, which Placo reads as unlimited. Without these,
        # IK solves past the wrist range and _clamp_arm_joints then silently changes
        # the gripper orientation that reaches the robot.
        for joint_list, lower, upper in (
            (QJ2M_LEFT_ARM_JOINTS, QJ2M_LEFT_ARM_LOWER, QJ2M_LEFT_ARM_UPPER),
            (QJ2M_RIGHT_ARM_JOINTS, QJ2M_RIGHT_ARM_LOWER, QJ2M_RIGHT_ARM_UPPER),
        ):
            for name, lo, hi in zip(joint_list, lower, upper):
                self.placo_robot.set_joint_limits(name, lo, hi)
        self.solver.enable_joint_limits(True)

        # Seed Placo at WBC default home; refresh EE tasks
        home_left = getattr(self, "_home_q_left", QJ2M_HOME_Q_LEFT)
        home_right = getattr(self, "_home_q_right", QJ2M_HOME_Q_RIGHT)
        self.placo_robot.state.q[self.placo_arm_joint_slice["left_arm"]] = home_left
        self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]] = home_right
        self.placo_robot.update_kinematics()
        for name, config in self.manipulator_config.items():
            if name not in self.effector_task:
                continue
            ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
            ee_target = tf.quaternion_matrix(ee_quat)
            ee_target[:3, 3] = ee_xyz
            self.effector_task[name].T_world_frame = ee_target

        # Null-space preference: elbow-back / symmetric ideal pose (not HOME).
        # Soft weight << EE so wrist tracking stays primary (ISSUE-001).
        self._joints_task = self.solver.add_joints_task()
        self._joints_task.set_joints(
            pref_joints_dict(QJ2M_LEFT_ARM_JOINTS, QJ2M_RIGHT_ARM_JOINTS)
        )
        self._joints_task.configure("joints_regularization", "soft", JOINTS_TASK_WEIGHT)
        print(
            f"[QJ2M] joints_task preference weight={JOINTS_TASK_WEIGHT} "
            f"toward ideal PREF_Q=[0.52, 0, 0, -2.09, 0, 0, 0] (both arms)"
        )

    # ---- ROS 2 setup ----
    def _robot_setup(self):
        self._ros_node = ros2.ensure_node("qj2m_teleop")
        self._ensure_dataset_publishers()
        self.arm_ctrl = QJ2MDualArmController(
            arm_command_topic="/throw/arm_joint_command",
            hand_topic="hand_control",
            arm_state_topic="/motor/q2w_upper_motor_joint_state",
            rate_hz=self.command_rate_hz,
        )
        print(
            f"[QJ2M] IK {1.0 / self._aux_period:.0f} Hz, arm command {self.command_rate_hz} Hz, "
            f"joint smoothing tau {self._joint_smooth_tau * 1000:.0f} ms"
        )

        # Dexterous hand publishers
        self.left_hand_pub = self._ros_node.create_publisher(Int32MultiArray, "/chatter1", 1)
        self.right_hand_pub = self._ros_node.create_publisher(Int32MultiArray, "/chatter1_right", 1)
        print("Waiting for initial QJ2M joint states...")
        start_wait = time.time()
        while ros2.ok() and self.arm_ctrl.timestamp == 0:
            if time.time() - start_wait > MOTOR_STATE_TIMEOUT:
                self._ros_node.get_logger().error(
                    f"[QJ2M] Timed out waiting for motor state after {MOTOR_STATE_TIMEOUT:.0f}s. Exiting."
                )
                sys.exit(1)
            ros2.sleep(0.1)
        # Home = WBC default_dof_pos / arm_teleoperation_default_pos
        self._home_q_left = QJ2M_HOME_Q_LEFT.copy()
        self._home_q_right = QJ2M_HOME_Q_RIGHT.copy()
        # Seed held pose from current measured state so first grip has a baseline
        self._held_q_left = np.array(self.arm_ctrl.get_left_arm_state(), dtype=float)[:7].copy()
        self._held_q_right = np.array(self.arm_ctrl.get_right_arm_state(), dtype=float)[:7].copy()
        self.arm_ctrl.q_des_left = self._held_q_left.copy()
        self.arm_ctrl.q_des_right = self._held_q_right.copy()
        print(
            "QJ2M controller ready. Home = WBC default "
            f"L={np.round(self._home_q_left, 3)} R={np.round(self._home_q_right, 3)}"
        )
        self._setup_xense_grippers()
        self._start_d435_publisher()
        self._start_force_sensor()
        # No yaw alignment here: at launch the headset is often off or facing an arbitrary
        # direction. The base controller aligns on the first grip instead.

    # ---- Camera ----
    def _initialize_camera(self):
        if not self.enable_camera:
            return
        print("Initializing cameras...")
        try:
            self.camera_interface = RosCameraInterface(
                camera_topics={
                    "left": {"color": "/camera_left/color/image_raw/compressed"},
                    "right": {"color": "/camera_right/color/image_raw/compressed"},
                },
                width=424, height=240, enable_depth=False,
            )
            self.camera_interface.start()
            print("Camera OK.")
        except Exception as e:
            print(f"Camera error: {e}")
            self.enable_camera = False

    # ---- Left joystick up / down: pose motions ----
    def _force_release_teleop(self):
        """Force-release teleop so a held grip does not resume motion after a pose move."""
        for name in self.manipulator_config:
            self.active[name] = False
        for k in list(self.ref_ee_xyz.keys()):
            self.ref_ee_xyz[k] = None
        for k in list(self.ref_ee_quat.keys()):
            self.ref_ee_quat[k] = None
        for k in list(self.ref_controller_xyz.keys()):
            self.ref_controller_xyz[k] = None
        for k in list(self.ref_controller_quat.keys()):
            self.ref_controller_quat[k] = None
        self._ema_q_left = None
        self._ema_q_right = None
        self._ik_q_latest = None

    def _handle_a_button_press(self):
        """Start sequential homing: left arm first, then right arm."""
        if self._home_q_left is None:
            print("[QJ2M] Home not captured yet, ignoring.")
            return
        if self._homing:
            return
        self._force_release_teleop()

        lq = np.array(self.arm_ctrl.get_left_arm_state(), dtype=float)
        rq = np.array(self.arm_ctrl.get_right_arm_state(), dtype=float)
        self._homing = True
        self._homing_mode = "sequential"
        self._homing_duration = HOMING_DURATION
        self._homing_start_time = time.time()
        self._homing_start_q_left = lq.copy()
        self._homing_start_q_right = rq.copy()
        via_l = _shortest_angular_path_safe(
            lq, proximal_first_via(lq, self._home_q_left),
            QJ2M_LEFT_ARM_LOWER, QJ2M_LEFT_ARM_UPPER)
        via_r = _shortest_angular_path_safe(
            rq, proximal_first_via(rq, self._home_q_right),
            QJ2M_RIGHT_ARM_LOWER, QJ2M_RIGHT_ARM_UPPER)
        home_l = _shortest_angular_path_safe(
            via_l, self._home_q_left, QJ2M_LEFT_ARM_LOWER, QJ2M_LEFT_ARM_UPPER)
        home_r = _shortest_angular_path_safe(
            via_r, self._home_q_right, QJ2M_RIGHT_ARM_LOWER, QJ2M_RIGHT_ARM_UPPER)
        self._homing_via_left = via_l
        self._homing_via_right = via_r
        self._homing_target_left = home_l
        self._homing_target_right = home_r
        print(
            f"[QJ2M] Left stick up → sequential homing (left then right) "
            f"via L={np.round(via_l, 2)} → home L={np.round(home_l, 2)} "
            f"| via R={np.round(via_r, 2)} → home R={np.round(home_r, 2)}"
        )

    def _start_simultaneous_pose_move(self, target_left, target_right, duration, label):
        """Both arms ease-in-out to target over duration (seconds)."""
        if self._homing:
            return
        self._force_release_teleop()

        lq = np.array(self.arm_ctrl.get_left_arm_state(), dtype=float)
        rq = np.array(self.arm_ctrl.get_right_arm_state(), dtype=float)
        target_l = _shortest_angular_path_safe(
            lq, target_left, QJ2M_LEFT_ARM_LOWER, QJ2M_LEFT_ARM_UPPER)
        target_r = _shortest_angular_path_safe(
            rq, target_right, QJ2M_RIGHT_ARM_LOWER, QJ2M_RIGHT_ARM_UPPER)
        self._homing = True
        self._homing_mode = "simultaneous"
        self._homing_duration = float(duration)
        self._homing_start_time = time.time()
        self._homing_start_q_left = lq.copy()
        self._homing_start_q_right = rq.copy()
        self._homing_via_left = target_l  # unused in simultaneous mode
        self._homing_via_right = target_r
        self._homing_target_left = target_l
        self._homing_target_right = target_r
        print(
            f"[QJ2M] {label} ({self._homing_duration:.0f}s, both arms) "
            f"L={np.round(target_l, 2)} R={np.round(target_r, 2)}"
        )

    def _handle_left_stick_down(self):
        """Start simultaneous dual-arm interp to measured stow pose (5s)."""
        self._start_simultaneous_pose_move(
            STOW_Q_LEFT, STOW_Q_RIGHT, STOW_DURATION, "Left stick down → stow pose"
        )

    def _handle_left_stick_right(self):
        """CMD0 (4s) → CMD (5s) → CMD2 (5s): both arms three-waypoint interp."""
        if self._homing:
            return
        self._force_release_teleop()

        lq = np.array(self.arm_ctrl.get_left_arm_state(), dtype=float)
        rq = np.array(self.arm_ctrl.get_right_arm_state(), dtype=float)
        w0_l = _shortest_angular_path_safe(
            lq, CMD0_Q_LEFT, QJ2M_LEFT_ARM_LOWER, QJ2M_LEFT_ARM_UPPER)
        w0_r = _shortest_angular_path_safe(
            rq, CMD0_Q_RIGHT, QJ2M_RIGHT_ARM_LOWER, QJ2M_RIGHT_ARM_UPPER)
        w1_l = _shortest_angular_path_safe(
            w0_l, CMD_Q_LEFT, QJ2M_LEFT_ARM_LOWER, QJ2M_LEFT_ARM_UPPER)
        w1_r = _shortest_angular_path_safe(
            w0_r, CMD_Q_RIGHT, QJ2M_RIGHT_ARM_LOWER, QJ2M_RIGHT_ARM_UPPER)
        w2_l = _shortest_angular_path_safe(
            w1_l, CMD2_Q_LEFT, QJ2M_LEFT_ARM_LOWER, QJ2M_LEFT_ARM_UPPER)
        w2_r = _shortest_angular_path_safe(
            w1_r, CMD2_Q_RIGHT, QJ2M_RIGHT_ARM_LOWER, QJ2M_RIGHT_ARM_UPPER)

        total = float(CMD0_DURATION) + float(CMD_DURATION) + float(CMD2_DURATION)
        self._homing = True
        self._homing_mode = "three_waypoint"
        self._homing_duration = total
        self._homing_phase_f1 = float(CMD0_DURATION) / total if total > 0 else 1.0 / 3.0
        self._homing_phase_f2 = float(CMD_DURATION) / total if total > 0 else 1.0 / 3.0
        self._homing_start_time = time.time()
        self._homing_start_q_left = lq.copy()
        self._homing_start_q_right = rq.copy()
        self._homing_via_left = w0_l
        self._homing_via_right = w0_r
        self._homing_mid_left = w1_l
        self._homing_mid_right = w1_r
        self._homing_target_left = w2_l
        self._homing_target_right = w2_r
        print(
            f"[QJ2M] Left stick right → CMD0 ({CMD0_DURATION:.0f}s) → "
            f"CMD ({CMD_DURATION:.0f}s) → CMD2 ({CMD2_DURATION:.0f}s) "
            f"L={np.round(w0_l, 2)} → {np.round(w1_l, 2)} → {np.round(w2_l, 2)} "
            f"| R={np.round(w0_r, 2)} → {np.round(w1_r, 2)} → {np.round(w2_r, 2)}"
        )

    def _check_logging_button(self):
        """B starts a rosbag; the next press stops and saves it.

        A is left to the joystick homing path, so this does not call the base
        handler (that one also treats A as a robot action).
        """
        b_button_state = self.xr_client.get_button_state_by_name("B")
        right_axis_click = self.xr_client.get_button_state_by_name("right_axis_click")

        if b_button_state and not self._prev_b_button_state:
            if not self._is_logging:
                self._start_rosbag_recording()
                self._is_logging = True
                if self._rosbag_process is None:
                    self._is_logging = False
            else:
                self._stop_rosbag_recording(save=True)
                self._is_logging = False

        if right_axis_click and self._is_logging:
            self._stop_rosbag_recording(save=False)
            self._is_logging = False

        self._prev_b_button_state = b_button_state

    def _apply_homing(self):
        """Interpolate arm joints toward home/stow during a pose move."""
        if not self._homing:
            return False
        elapsed = time.time() - self._homing_start_time
        duration = self._homing_duration if self._homing_duration > 0 else HOMING_DURATION
        t = elapsed / duration
        if t >= 1.0:
            ql = self._homing_target_left.copy()
            qr = self._homing_target_right.copy()
            if self._homing_mode in ("two_waypoint", "three_waypoint"):
                done_label = "CMD2"
            elif self._homing_mode == "simultaneous":
                done_label = "Stow"
            else:
                done_label = "Homing"
            self._homing = False
            # Reset control references so teleop resumes cleanly
            # Set all refs to None instead of clearing — _update_ik() in
            # the IK thread iterates these dicts by key and will KeyError
            # on an empty dict.
            for k in list(self.ref_ee_xyz.keys()):
                self.ref_ee_xyz[k] = None
            for k in list(self.ref_ee_quat.keys()):
                self.ref_ee_quat[k] = None
            for k in list(self.ref_controller_xyz.keys()):
                self.ref_controller_xyz[k] = None
            for k in list(self.ref_controller_quat.keys()):
                self.ref_controller_quat[k] = None
            # Held pose becomes the finished posture for subsequent re-grip
            self._held_q_left = ql.copy()
            self._held_q_right = qr.copy()
            self._ema_q_left = None
            self._ema_q_right = None
            self._ik_q_latest = None
            # Ensure teleop stays off until user re-grips
            for name in self.manipulator_config:
                self.active[name] = False
            # Also re-align yaw so operator forward maps correctly post-move
            try:
                self.reset_yaw_alignment()
            except Exception as e:
                print(f"[QJ2M] Warning: post-{done_label.lower()} yaw reset failed: {e}")
            print(f"[QJ2M] {done_label} complete — arms holding pose (re-grip to resume).")
            self._recapture_follow_offset_if_enabled()
        elif self._homing_mode == "three_waypoint":
            ql, qr = sample_simultaneous_three_waypoint(
                self._homing_start_q_left,
                self._homing_via_left,
                self._homing_mid_left,
                self._homing_target_left,
                self._homing_start_q_right,
                self._homing_via_right,
                self._homing_mid_right,
                self._homing_target_right,
                t,
                f1=self._homing_phase_f1,
                f2=self._homing_phase_f2,
            )
        elif self._homing_mode == "two_waypoint":
            ql, qr = sample_simultaneous_two_waypoint(
                self._homing_start_q_left,
                self._homing_via_left,
                self._homing_target_left,
                self._homing_start_q_right,
                self._homing_via_right,
                self._homing_target_right,
                t,
                via_fraction=self._homing_via_fraction,
            )
        elif self._homing_mode == "simultaneous":
            ql, qr = sample_simultaneous_arms(
                self._homing_start_q_left,
                self._homing_target_left,
                self._homing_start_q_right,
                self._homing_target_right,
                t,
            )
        else:
            ql, qr = sample_left_then_right_homing(
                self._homing_start_q_left,
                self._homing_via_left,
                self._homing_target_left,
                self._homing_start_q_right,
                self._homing_via_right,
                self._homing_target_right,
                t,
            )

        self.placo_robot.state.q[self.placo_arm_joint_slice["left_arm"]] = ql
        self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]] = qr
        self.placo_robot.update_kinematics()
        # Keep IK visualization / tasks glued to the wrists during/after pose move
        for name, config in self.manipulator_config.items():
            if name not in self.effector_task:
                continue
            ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
            if self.effector_control_mode.get(name, "pose") == "position":
                self.effector_task[name].target_world = ee_xyz
            else:
                ee_target = tf.quaternion_matrix(ee_quat)
                ee_target[:3, 3] = ee_xyz
                self.effector_task[name].T_world_frame = ee_target
        self.arm_ctrl.q_des_left = ql.copy()
        self.arm_ctrl.q_des_right = qr.copy()
        self._held_q_left = ql.copy()
        self._held_q_right = qr.copy()
        return True

    # ---- State read ----
    def _update_robot_state(self):
        if self._homing:
            return  # skip reading hardware state during homing

        # Check for motor state timeout (runtime safety)
        if (self.arm_ctrl.last_update_time > 0 and
                time.time() - self.arm_ctrl.last_update_time > MOTOR_STATE_TIMEOUT):
            self._ros_node.get_logger().warn(
                f"[QJ2M] Motor state not received for "
                f"{time.time() - self.arm_ctrl.last_update_time:.0f}s. Holding position."
            )

        lq = self.arm_ctrl.get_left_arm_state()
        rq = self.arm_ctrl.get_right_arm_state()
        self.placo_robot.state.q[self.placo_arm_joint_slice["left_arm"]] = lq
        self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]] = rq

    def _update_ik(self):
        """Skip soft-IK drift when idle/homing; only solve while gripping."""
        if self._homing:
            return

        left_grip = self.xr_client.get_key_value_by_name(
            self.manipulator_config["left_arm"]["control_trigger"])
        right_grip = self.xr_client.get_key_value_by_name(
            self.manipulator_config["right_arm"]["control_trigger"])
        if self.follow_mode:
            any_grip = (
                right_grip > self.control_activate_threshold
                or self.active.get("right_arm", False)
            )
        else:
            any_grip = (
                left_grip > self.control_activate_threshold
                or right_grip > self.control_activate_threshold
                or self.active.get("left_arm", False)
                or self.active.get("right_arm", False)
            )
        if not any_grip:
            self._ik_q_latest = None
            for name in self.manipulator_config:
                self.active[name] = False
            # Park Placo at last held command so re-grip starts from home/hold
            if self._held_q_left is not None:
                self.placo_robot.state.q[self.placo_arm_joint_slice["left_arm"]] = (
                    self._held_q_left)
            if self._held_q_right is not None:
                self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]] = (
                    self._held_q_right)
            self.placo_robot.update_kinematics()
            for name, config in self.manipulator_config.items():
                if name not in self.effector_task:
                    continue
                ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
                ee_target = tf.quaternion_matrix(ee_quat)
                ee_target[:3, 3] = ee_xyz
                self.effector_task[name].T_world_frame = ee_target
            return

        super()._update_ik()

    # ---- Command send ----
    def _send_command(self):
        # Joysticks: always sample so edge flags stay valid across pose moves / homing
        left_axis = self.xr_client.get_joystick_state("left")
        left_stick_up = left_axis[1] > 0.7
        left_stick_down = left_axis[1] < -0.7
        left_stick_right = left_axis[0] > 0.7
        right_axis = self.xr_client.get_joystick_state("right")
        right_stick_up = right_axis[1] > 0.7
        right_stick_down = right_axis[1] < -0.7

        if not self._homing:
            if left_stick_up and not self._prev_left_stick_up:
                self._handle_a_button_press()
            elif left_stick_down and not self._prev_left_stick_down:
                self._handle_left_stick_down()
            # elif left_stick_right and not self._prev_left_stick_right:
            #     self._handle_left_stick_right()  # temporarily disabled

            if right_stick_up and not self._prev_right_stick_up:
                self.toggle_follow_mode()
                # Drop EMA so re-grip after mode switch starts clean
                self._ema_q_left = None
                self._ema_q_right = None
                self._ik_q_latest = None
            elif right_stick_down and not self._prev_right_stick_down:
                try:
                    self.reset_yaw_alignment()
                except Exception as e:
                    print(f"[QJ2M] Yaw reset failed: {e}")

        self._prev_left_stick_up = left_stick_up
        self._prev_left_stick_down = left_stick_down
        self._prev_left_stick_right = left_stick_right
        self._prev_right_stick_up = right_stick_up
        self._prev_right_stick_down = right_stick_down

        # Homing / stow overrides normal control
        if self._apply_homing():
            self.arm_ctrl.q_des_left = self.placo_robot.state.q[
                self.placo_arm_joint_slice["left_arm"]].copy()[:7]
            self.arm_ctrl.q_des_right = self.placo_robot.state.q[
                self.placo_arm_joint_slice["right_arm"]].copy()[:7]
            # Clamp to joint limits before sending to hardware
            self.arm_ctrl.q_des_left, self.arm_ctrl.q_des_right = self._clamp_arm_joints(
                self.arm_ctrl.q_des_left, self.arm_ctrl.q_des_right)
            self.arm_ctrl.publish_arm_command()
            self._update_trigger_end_effectors_throttled()
            return

        now = time.monotonic()
        ema_dt = 0.0 if self._ema_last_time is None else now - self._ema_last_time
        self._ema_last_time = now
        # Clamp so a stalled tick cannot snap the arm straight onto the IK target.
        ema_dt = min(max(ema_dt, 0.0), 4.0 / self.command_rate_hz)
        a = 1.0 - np.exp(-ema_dt / self._joint_smooth_tau) if self._joint_smooth_tau > 0 else 1.0
        ik_q = self._ik_q_latest

        if self.active.get("left_arm", False):
            if self._ema_q_left is None:
                # Rising edge: resume from last held command to avoid vertical drop
                held = self._get_held_q("left_arm")
                if held is None and ik_q is not None:
                    held = ik_q[0]
                self._ema_q_left = held.copy() if held is not None else None
            elif ik_q is not None:
                self._ema_q_left = a * ik_q[0] + (1.0 - a) * self._ema_q_left
            if self._ema_q_left is not None:
                self.arm_ctrl.q_des_left = self._ema_q_left.copy()
                self._held_q_left = self.arm_ctrl.q_des_left.copy()
        else:
            self._ema_q_left = None
            # Keep publishing last q_des (frozen) so the arm holds pose

        if self.active.get("right_arm", False):
            if self._ema_q_right is None:
                held = self._get_held_q("right_arm")
                if held is None and ik_q is not None:
                    held = ik_q[1]
                self._ema_q_right = held.copy() if held is not None else None
            elif ik_q is not None:
                self._ema_q_right = a * ik_q[1] + (1.0 - a) * self._ema_q_right
            if self._ema_q_right is not None:
                self.arm_ctrl.q_des_right = self._ema_q_right.copy()
                self._held_q_right = self.arm_ctrl.q_des_right.copy()
        else:
            self._ema_q_right = None
        # Clamp to joint limits before sending to hardware
        self.arm_ctrl.q_des_left, self.arm_ctrl.q_des_right = self._clamp_arm_joints(
            self.arm_ctrl.q_des_left, self.arm_ctrl.q_des_right)
        # Keep held snapshots clamped as well (source of truth for re-grip)
        if self.active.get("left_arm", False):
            self._held_q_left = np.asarray(self.arm_ctrl.q_des_left, dtype=float)[:7].copy()
        if self.active.get("right_arm", False):
            self._held_q_right = np.asarray(self.arm_ctrl.q_des_right, dtype=float)[:7].copy()
        self.arm_ctrl.publish_arm_command()
        self._update_trigger_end_effectors_throttled()

    def _update_trigger_end_effectors_throttled(self):
        now = time.monotonic()
        if now - self._last_aux_time < self._aux_period * 0.9:
            return
        self._last_aux_time = now
        self._update_trigger_end_effectors()

    def _setup_xense_grippers(self):
        if self.xense_grippers is not None:
            return
        if not self._enable_xense_gripper:
            print("[Xense] gripper teleop disabled")
            return
        try:
            grippers = XenseTeleopGrippers()
            grippers.connect()
            self.xense_grippers = grippers
            print("[Xense] trigger 0 = open 85 mm, trigger 1 = close 0 mm")
            self._start_tactile_publisher()
        except Exception as exc:
            self.xense_grippers = None
            print(f"[Xense] gripper teleop unavailable: {exc}")

    def _update_trigger_end_effectors(self):
        # Dexterous hand + Xense gripper via trigger continuous value.
        # X-button toggles LEFT hand/gripper lock state
        x_button_state = self.xr_client.get_button_state_by_name("X")
        if x_button_state and not self._prev_x_button_state:
            self._left_hand_locked = not self._left_hand_locked
            state_label = "LOCKED" if self._left_hand_locked else "UNLOCKED"
            print(f"[QJ2M] X-button → LEFT trigger (hand + gripper): {state_label}")
        self._prev_x_button_state = x_button_state

        # Y-button toggles between two open modes
        y_button_state = self.xr_client.get_button_state_by_name("Y")
        if y_button_state and not self._prev_y_button_state:
            self._hand_open_mode2 = not self._hand_open_mode2
            mode_label = "MODE2 (all open)" if self._hand_open_mode2 else "MODE1 (joints 0,1 closed)"
            print(f"[QJ2M] Y-button → hand open: {mode_label}")
        self._prev_y_button_state = y_button_state

        hand_open = DEX_HAND_OPEN_MODE2 if self._hand_open_mode2 else DEX_HAND_OPEN_MODE1

        for arm_name in ["left_arm", "right_arm"]:
            # Skip locked hands — hold last position
            if arm_name == "left_arm" and self._left_hand_locked:
                continue
            trigger_name = self.manipulator_config[arm_name]["gripper_config"]["gripper_trigger"]
            trigger_val = self.xr_client.get_key_value_by_name(trigger_name)
            trigger_val = max(0.0, min(1.0, float(trigger_val)))
            if self.xense_grippers is not None:
                self.xense_grippers.set_close_fraction(arm_name, trigger_val)
            pos = _interp_hand_pos(trigger_val, hand_open)
            msg = Int32MultiArray()
            msg.data = pos
            if arm_name == "left_arm":
                self.left_hand_pub.publish(msg)
            else:
                self.right_hand_pub.publish(msg)
            # DEBUG: verify hand control messages are being sent
            if arm_name == "right_arm":
                if not hasattr(self, "_hand_debug_counter"):
                    self._hand_debug_counter = 0
                self._hand_debug_counter += 1
                if self._hand_debug_counter % 50 == 1:  # ~every 0.5s at 100Hz
                    print(f"[QJ2M] RIGHT hand pub: trigger={trigger_val:.2f}, pos={pos}")
        self._record_arm_sample()

    def _ensure_dataset_publishers(self):
        if self._dataset_pubs_ready:
            return
        qos = ros2.default_qos(10)
        image_qos = ros2.default_qos(2)
        self._arm_pubs = {
            key: self._ros_node.create_publisher(JointState, topic, qos)
            for key, topic in VLA_ARM_TOPICS.items()
        }
        self._gripper_pubs = {}
        for spec in XENSE_GRIPPER_TOPICS.values():
            for topic in spec.values():
                self._gripper_pubs[topic] = self._ros_node.create_publisher(Float64, topic, qos)
        self._tactile_pubs = {
            topic: self._ros_node.create_publisher(Image, topic, image_qos)
            for topic in xense_record_topics()
            if topic.endswith("/image")
        }
        self._d435_pub = self._ros_node.create_publisher(Image, D435I_COLOR_TOPIC, qos)
        self._d435_depth_pub = self._ros_node.create_publisher(Image, D435I_DEPTH_TOPIC, qos)
        self._force_pubs = {
            topic: self._ros_node.create_publisher(WrenchStamped, topic, qos)
            for topic in force_record_topics()
        }
        self._dataset_pubs_ready = True

    def _joint_msg(self, names, values, stamp) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(names)
        q = [float(v) for v in list(values)[:7]]
        if len(q) < 7:
            q.extend([0.0] * (7 - len(q)))
        msg.position = q
        return msg

    def _image_msg(self, image, stamp) -> Image:
        frame = np.ascontiguousarray(image)
        msg = Image()
        msg.header.stamp = stamp
        if frame.ndim == 2:
            msg.height, msg.width = int(frame.shape[0]), int(frame.shape[1])
            if frame.dtype == np.uint16:
                msg.encoding = "16UC1"
                msg.step = msg.width * 2
            else:
                msg.encoding = "mono8"
                msg.step = msg.width
        else:
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = "bgr8"
            msg.step = msg.width * frame.shape[2]
        msg.is_bigendian = 0
        msg.data = array("B", frame.tobytes())
        return msg

    def _record_arm_sample(self):
        """Keep this control tick's 7-DOF arm state and command for the 30 Hz frame."""
        if not hasattr(self, "arm_ctrl"):
            return
        left_state = np.array(self.arm_ctrl.qpos_left, dtype=float)
        right_state = np.array(self.arm_ctrl.qpos_right, dtype=float)
        left_cmd = self.arm_ctrl.q_des_left
        right_cmd = self.arm_ctrl.q_des_right
        left_cmd = left_state if left_cmd is None else np.array(left_cmd, dtype=float)
        right_cmd = right_state if right_cmd is None else np.array(right_cmd, dtype=float)
        self._arm_history.append(time.time(), (left_state, right_state, left_cmd, right_cmd))

    def _publish_vla_frame(self, t_capture: float, color=None, depth=None):
        """Publish every rosbag topic once, all stamped with the frame time t_capture."""
        if not self._dataset_pubs_ready:
            return
        stamp = ros2.stamp_from_sec(t_capture)
        try:
            if color is not None and depth is not None:
                self._d435_pub.publish(self._image_msg(color, stamp))
                self._d435_depth_pub.publish(self._image_msg(depth, stamp))

            arm = self._arm_history.nearest(t_capture, VLA_MAX_SAMPLE_AGE_S)
            if arm is not None:
                left_state, right_state, left_cmd, right_cmd = arm
                for key, names, values in (
                    ("left_state", QJ2M_LEFT_ARM_JOINTS, left_state),
                    ("right_state", QJ2M_RIGHT_ARM_JOINTS, right_state),
                    ("left_command", QJ2M_LEFT_ARM_JOINTS, left_cmd),
                    ("right_command", QJ2M_RIGHT_ARM_JOINTS, right_cmd),
                ):
                    self._arm_pubs[key].publish(self._joint_msg(names, values, stamp))

            pose = self.xense_grippers.pose_state() if self.xense_grippers else {}
            for arm_name, spec in XENSE_GRIPPER_TOPICS.items():
                arm_pose = pose.get(arm_name) or {}
                current = arm_pose.get("current_mm")
                if current is None:
                    continue
                target = arm_pose.get("target_mm")
                if target is None:
                    target = current
                state = Float64()
                state.data = float(current)
                command = Float64()
                command.data = float(target)
                self._gripper_pubs[spec["state"]].publish(state)
                self._gripper_pubs[spec["command"]].publish(command)

            for arm_name, spec in FORCE_ARMS.items():
                sample = self._force_history[arm_name].nearest(t_capture, VLA_MAX_SAMPLE_AGE_S)
                if sample is None:
                    continue
                raw, filtered = sample
                self._force_pubs[spec["topic"]].publish(self._wrench_msg(raw, stamp, spec["frame"]))
                self._force_pubs[spec["filtered"]].publish(
                    self._wrench_msg(filtered, stamp, spec["frame"])
                )
                mirror = spec.get("mirror")
                if mirror:
                    self._force_pubs[mirror].publish(
                        self._wrench_msg(raw, stamp, spec["mirror_frame"])
                    )

            with self._tactile_lock:
                tactile = dict(self._latest_tactile)
            for topic, (t_image, image) in tactile.items():
                pub = self._tactile_pubs.get(topic)
                if pub is None or abs(t_capture - t_image) > VLA_TACTILE_MAX_AGE_S:
                    continue
                pub.publish(self._image_msg(image, stamp))
        except Exception as exc:
            now = time.time()
            if now - getattr(self, "_vla_pub_error_time", 0.0) > 1.0:
                self._vla_pub_error_time = now
                print(f"[QJ2M] dataset publish failed: {exc}")

    def _frame_clock_loop(self):
        """30 Hz frame clock without images, used when the D435i cannot stream."""
        period = 1.0 / VLA_FRAME_HZ
        next_tick = time.time()
        print(f"[QJ2M] rosbag frame clock: {VLA_FRAME_HZ} Hz timer (no D435i images)")
        while not self._d435_stop.is_set():
            self._publish_vla_frame(time.time())
            next_tick += period
            remain = next_tick - time.time()
            if remain > 0:
                self._d435_stop.wait(remain)
            else:
                next_tick = time.time()

    def _start_tactile_publisher(self):
        if self._tactile_thread is not None or self.xense_grippers is None:
            return
        self._tactile_stop.clear()
        self._tactile_thread = threading.Thread(
            target=self._tactile_publish_loop,
            name="xense-tactile-pub",
            daemon=True,
        )
        self._tactile_thread.start()

    def _tactile_publish_loop(self):
        """Poll tactile images as fast as get_img allows; the frame clock publishes them."""
        period = 1.0 / XENSE_TACTILE_POLL_HZ
        while not self._tactile_stop.is_set():
            started = time.time()
            frames = {}
            try:
                frames = self.xense_grippers.fetch_tactile_images()
            except Exception as exc:
                print(f"[Xense] tactile fetch failed: {exc}")
            if frames:
                t_image = time.time()
                with self._tactile_lock:
                    for topic, image in frames.items():
                        if image is not None:
                            self._latest_tactile[topic] = (t_image, image)
            remain = period - (time.time() - started)
            if remain > 0:
                self._tactile_stop.wait(remain)

    def _start_d435_publisher(self):
        if self._d435_thread is not None:
            return
        self._d435_stop.clear()
        self._d435_thread = threading.Thread(
            target=self._d435_publish_loop,
            name="d435i-rgbd",
            daemon=True,
        )
        self._d435_thread.start()

    def _prepare_d435_color(self, profile) -> None:
        """Turn on auto exposure, auto white balance, and 50 Hz flicker rejection."""
        try:
            import pyrealsense2 as rs
            sensor = profile.get_device().first_color_sensor()
            sensor.set_option(rs.option.enable_auto_exposure, 1)
            sensor.set_option(rs.option.enable_auto_white_balance, 1)
            sensor.set_option(rs.option.power_line_frequency, 1)
        except Exception as exc:
            print(f"[D435i] color option setup failed: {exc}")

    def _d435_publish_loop(self):
        """D435i color frames are the rosbag master clock: one full frame per color image."""
        try:
            import pyrealsense2 as rs
        except Exception as exc:
            print(f"[D435i] pyrealsense2 unavailable: {exc}")
            self._frame_clock_loop()
            return
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(D435I_SERIAL)
        config.enable_stream(
            rs.stream.color, D435I_WIDTH, D435I_HEIGHT, rs.format.bgr8, D435I_FPS
        )
        config.enable_stream(
            rs.stream.depth, D435I_WIDTH, D435I_HEIGHT, rs.format.z16, D435I_FPS
        )
        align = rs.align(rs.stream.color)
        try:
            profile = pipeline.start(config)
        except Exception as exc:
            print(f"[D435i] color+depth stream failed: {exc}")
            self._frame_clock_loop()
            return
        self._prepare_d435_color(profile)
        print(
            f"[D435i] color+depth {D435I_WIDTH}x{D435I_HEIGHT} @ {D435I_FPS} Hz "
            f"-> {D435I_COLOR_TOPIC} , {D435I_DEPTH_TOPIC} (rosbag master clock)"
        )
        global_time = rs.timestamp_domain.global_time
        skew_warned = False
        try:
            # Auto white balance needs a few seconds; early frames are green.
            warmup_until = time.time() + 3.0
            while not self._d435_stop.is_set() and time.time() < warmup_until:
                try:
                    pipeline.wait_for_frames(timeout_ms=1000)
                except RuntimeError:
                    continue
            while not self._d435_stop.is_set():
                try:
                    frames = pipeline.wait_for_frames(timeout_ms=1000)
                except RuntimeError:
                    continue
                aligned = align.process(frames)
                color = aligned.get_color_frame()
                depth = aligned.get_depth_frame()
                if color is None or depth is None:
                    continue
                now = time.time()
                t_capture = now
                if color.get_frame_timestamp_domain() == global_time:
                    t_capture = color.get_timestamp() / 1000.0
                if abs(now - t_capture) > D435I_MAX_CLOCK_SKEW_S:
                    if not skew_warned:
                        skew_warned = True
                        print(
                            f"[D435i] color timestamp {now - t_capture:+.3f}s off host clock; "
                            "stamping frames with arrival time"
                        )
                    t_capture = now
                self._publish_vla_frame(
                    t_capture,
                    np.asanyarray(color.get_data()),
                    np.asanyarray(depth.get_data()),
                )
        finally:
            pipeline.stop()
            print("[D435i] color+depth stream stopped")

    def _start_force_sensor(self):
        if self._force_sensor is not None:
            return
        self._force_sensor = WristForceSensors()
        self._force_sensor.start(self._on_force_sample)

    def _wrench_msg(self, values, stamp, frame_id: str) -> WrenchStamped:
        msg = WrenchStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.wrench.force.x = round_half_away(values[0])
        msg.wrench.force.y = round_half_away(values[1])
        msg.wrench.force.z = round_half_away(values[2])
        msg.wrench.torque.x = round_half_away(values[3])
        msg.wrench.torque.y = round_half_away(values[4])
        msg.wrench.torque.z = round_half_away(values[5])
        return msg

    def _on_force_sample(self, arm_name, raw, filtered) -> None:
        self._force_history[arm_name].append(time.time(), (tuple(raw), tuple(filtered)))

    def _get_rosbag_topics(self) -> list:
        return list(VLA_ARM_TOPICS.values()) + xense_record_topics() + [
            D435I_COLOR_TOPIC,
            D435I_DEPTH_TOPIC,
        ] + force_record_topics()

    # ---- Logging ----
    def _get_robot_state_for_logging(self) -> Dict:
        return {
            "qpos_left": self.arm_ctrl.qpos_left,
            "qpos_right": self.arm_ctrl.qpos_right,
            "qvel_left": self.arm_ctrl.qvel_left,
            "qvel_right": self.arm_ctrl.qvel_right,
            "xense_grippers": (
                self.xense_grippers.pose_state() if self.xense_grippers else {}
            ),
        }

    def _get_camera_frame_for_logging(self) -> Dict:
        if not self.camera_interface:
            return {}
        return self.camera_interface.get_compressed_frames() or {}

    def _should_keep_running(self) -> bool:
        return super()._should_keep_running() and ros2.ok()

    def _shutdown_robot(self):
        self._tactile_stop.set()
        if self._tactile_thread is not None:
            self._tactile_thread.join(timeout=6.0)
            self._tactile_thread = None
        self._d435_stop.set()
        if self._d435_thread is not None:
            self._d435_thread.join(timeout=3.0)
            self._d435_thread = None
        if self._force_sensor is not None:
            self._force_sensor.stop()
            self._force_sensor = None
        if self.xense_grippers is not None:
            self.xense_grippers.stop()
            self.xense_grippers = None
        self.arm_ctrl.stop()
        print("QJ2M arm controller stopped.")
