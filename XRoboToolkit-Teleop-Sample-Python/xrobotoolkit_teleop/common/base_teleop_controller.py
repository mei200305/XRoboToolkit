import abc
import threading
import os
import webbrowser
from typing import Any, Dict

import meshcat.transformations as tf
import numpy as np
import placo
from placo_utils.visualization import (
    frame_viz,
    robot_frame_viz,
    robot_viz,
)

from xrobotoolkit_teleop.common.data_logger import DataLogger
from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.utils.geometry import (
    apply_delta_pose,
    quat_diff_as_angle_axis,
    yaw_rotation_matrix,
)
from xrobotoolkit_teleop.utils.parallel_gripper_utils import (
    calc_parallel_gripper_position,
)

# Headset up-axis world Z below this (tilted > ~75°, upside down, lying on a table)
# makes its heading unreliable for yaw alignment.
HMD_UPRIGHT_MIN_UP_Z = 0.25


class BaseTeleopController(abc.ABC):
    def __init__(
        self,
        robot_urdf_path: str,
        manipulator_config: Dict[str, Dict[str, Any]],
        floating_base: bool,
        R_headset_world: np.ndarray,
        scale_factor: float,
        q_init: np.ndarray,
        dt: float,
        enable_log_data: bool = False,
        log_dir: str = "logs",
        log_freq: float = 50,
    ):
        self.robot_urdf_path = robot_urdf_path
        self.manipulator_config = manipulator_config
        self.floating_base = floating_base
        self.R_headset_world = R_headset_world
        # Dynamic yaw offset — rotate about world Z to align operator forward with robot forward.
        # Reset via reset_yaw_alignment() (e.g. bound to a controller button).
        self._R_yaw_offset = np.eye(3)
        self._enable_dynamic_yaw = True
        # False until an alignment succeeds with the headset upright; the next grip aligns.
        self._yaw_aligned = False
        self.scale_factor = scale_factor
        self.q_init = q_init
        self.dt = dt
        self.xr_client = XrClient()

        self.enable_log_data = enable_log_data
        self.log_dir = log_dir
        self.log_freq = log_freq
        if enable_log_data:
            self.data_logger = DataLogger(log_dir=log_dir)

        # Initial poses
        self.ref_ee_xyz = {name: None for name in manipulator_config.keys()}
        self.ref_ee_quat = {name: None for name in manipulator_config.keys()}
        self.ref_controller_xyz = {name: None for name in manipulator_config.keys()}
        self.ref_controller_quat = {name: None for name in manipulator_config.keys()}
        self.effector_task = {}
        self.effector_control_mode = {}  # Store control mode for each end effector
        self.active = {}
        self.gripper_pos_target = {}
        self.control_activate_threshold = 0.3
        self.control_deactivate_threshold = 0.2

        # Pose delta deadband (suppress jitter when controller is held still)
        self._delta_deadband_xyz = 0.001   # 2 mm position deadband (was 0.5 mm)
        self._delta_deadband_rot = 0.016   # ~1.5° rotation deadband (was ~0.5°)

        # Left-follow mode: right Pico drives right-arm world X and Z. Y and both
        # EE orientations stay frozen at the pose captured on toggle. Left EE X/Z
        # keep the captured world offset vs the right EE.
        # Toggle via subclass button binding (e.g. right stick up on QJ2M).
        self.follow_mode = False
        self._follow_leader = "right_arm"
        self._follow_follower = "left_arm"
        self._follow_delta_xyz = None
        self._follow_xyz_leader = None
        self._follow_xyz_follower = None
        self._follow_quat_leader = None
        self._follow_quat_follower = None

        # Motion tracker support
        self.motion_tracker_task = {}
        self.ref_tracker_xyz = {}  # Store initial tracker positions
        self.ref_robot_xyz = {}  # Store initial robot end-effector positions
        for name, config in self.manipulator_config.items():
            if "gripper_config" in config:
                gripper_config = config["gripper_config"]
                self.gripper_pos_target[name] = {
                    joint_name: joint_pos
                    for joint_name, joint_pos in zip(gripper_config["joint_names"], gripper_config["open_pos"])
                }

        self._stop_event = threading.Event()

        self._robot_setup()
        self._placo_setup()

    def _clear_teleop_activation(self):
        """Drop grip state and pose references so the operator must re-grip."""
        for name in self.manipulator_config:
            self.active[name] = False
        for store in (
            self.ref_ee_xyz,
            self.ref_ee_quat,
            self.ref_controller_xyz,
            self.ref_controller_quat,
        ):
            for k in store:
                store[k] = None

    def _is_follow_follower(self, src_name: str) -> bool:
        return bool(
            self.follow_mode
            and src_name == self._follow_follower
            and self._follow_leader in self.manipulator_config
        )

    def _clear_follow_offset(self):
        self._follow_delta_xyz = None
        self._follow_xyz_leader = None
        self._follow_xyz_follower = None
        self._follow_quat_leader = None
        self._follow_quat_follower = None

    def _follow_xz_only_xyz(self, src_name: str, xyz):
        """Keep captured world Y; use ``xyz[0]`` and ``xyz[2]`` as commanded X/Z."""
        out = np.asarray(xyz, dtype=float).reshape(3).copy()
        frozen = None
        if src_name == self._follow_leader:
            frozen = self._follow_xyz_leader
        elif src_name == self._follow_follower:
            frozen = self._follow_xyz_follower
        if frozen is not None:
            out[1] = frozen[1]
        return out

    def _assign_ee_target(self, src_name: str, xyz, quat=None):
        """Write an EE IK target; ``quat`` is ignored for position-only tasks."""
        if self.effector_control_mode[src_name] == "position":
            self.effector_task[src_name].target_world = np.asarray(xyz, dtype=float)
            return
        if quat is None:
            ee_target = np.eye(4)
            ee_target[:3, 3] = np.asarray(xyz, dtype=float)
            self.effector_task[src_name].T_world_frame = ee_target
            return
        target_pose = tf.quaternion_matrix(np.asarray(quat, dtype=float))
        target_pose[:3, 3] = np.asarray(xyz, dtype=float)
        self.effector_task[src_name].T_world_frame = target_pose

    def _capture_follow_offset(self) -> bool:
        """Freeze Y and orientations; record world X/Z offset."""
        leader = self._follow_leader
        follower = self._follow_follower
        if leader not in self.manipulator_config or follower not in self.manipulator_config:
            self._clear_follow_offset()
            return False

        p_r, q_r = self._get_activation_ee_pose(leader, self.manipulator_config[leader])
        p_l, q_l = self._get_activation_ee_pose(follower, self.manipulator_config[follower])
        self._follow_xyz_leader = np.asarray(p_r, dtype=float).reshape(3).copy()
        self._follow_xyz_follower = np.asarray(p_l, dtype=float).reshape(3).copy()
        self._follow_delta_xyz = self._follow_xyz_follower - self._follow_xyz_leader
        self._follow_quat_leader = np.asarray(q_r, dtype=float).reshape(4).copy()
        self._follow_quat_follower = np.asarray(q_l, dtype=float).reshape(4).copy()
        print(
            f"[跟随模式] 已记录 X/Z 位置差 Δx={self._follow_delta_xyz[0]:.4f} m "
            f"Δz={self._follow_delta_xyz[2]:.4f} m；Y 与左右臂末端姿态已锁定",
            flush=True,
        )
        return True

    def _recapture_follow_offset_if_enabled(self):
        """Refresh xyz offset and frozen orientations after a pose move."""
        if self.follow_mode:
            self._capture_follow_offset()

    def toggle_follow_mode(self) -> bool:
        """Toggle left-arm follow mode on/off.

        When enabled, Y and both EE orientations are locked. Only the right
        Pico world X/Z drive the right arm; the left EE X/Z track the captured
        offset. Left Pico is ignored for arm motion. Clears grip references so
        the operator must re-grip the right controller.
        Returns the new follow_mode state.
        """
        enabling = not self.follow_mode
        self._clear_teleop_activation()
        if enabling:
            if not self._capture_follow_offset():
                self.follow_mode = False
                print("[跟随模式] 切换失败：需要同时配置左臂和右臂", flush=True)
                return False
            self.follow_mode = True
            print("=" * 60, flush=True)
            print(
                "[跟随模式] 已开启 — 右手柄仅控制右臂 X/Z 轴，Y 与左右臂末端姿态保持不变；"
                "左臂 X/Z 跟随右臂。请重新握住右手柄。",
                flush=True,
            )
            print("=" * 60, flush=True)
        else:
            self.follow_mode = False
            self._clear_follow_offset()
            print("=" * 60, flush=True)
            print("[跟随模式] 已关闭 — 恢复双手独立遥操作。请重新握住手柄。", flush=True)
            print("=" * 60, flush=True)
        return self.follow_mode

    def _apply_follow_follower_task(self):
        """Plan the follower EE: same world X/Z offset, frozen Y and orientation."""
        if not self._is_follow_follower(self._follow_follower):
            return
        if self._follow_delta_xyz is None:
            return

        leader = self._follow_leader
        follower = self._follow_follower
        follower_cfg = self.manipulator_config[follower]
        frozen_quat = self._follow_quat_follower

        if not self.active.get(leader, False):
            if self.active.get(follower, False):
                print(f"{follower} is deactivated (left-follow).", flush=True)
            self.active[follower] = False
            self.ref_ee_xyz[follower] = None
            self.ref_ee_quat[follower] = None
            self.ref_controller_xyz[follower] = None
            self.ref_controller_quat[follower] = None
            ee_xyz, ee_quat = self._get_inactive_ee_pose(follower, follower_cfg)
            if frozen_quat is not None:
                ee_quat = frozen_quat
            ee_xyz = self._follow_xz_only_xyz(follower, ee_xyz)
            self._assign_ee_target(follower, ee_xyz, ee_quat)
            return

        if not self.active.get(follower, False):
            print(f"{follower} is activated (left-follow).", flush=True)
        self.active[follower] = True

        if self.effector_control_mode[leader] == "position":
            leader_xyz = np.asarray(self.effector_task[leader].target_world, dtype=float)
        else:
            T_leader = np.asarray(self.effector_task[leader].T_world_frame, dtype=float)
            leader_xyz = T_leader[:3, 3].copy()
        follower_xyz = self._follow_xz_only_xyz(
            follower,
            np.array(
                [
                    leader_xyz[0] + self._follow_delta_xyz[0],
                    0.0,
                    leader_xyz[2] + self._follow_delta_xyz[2],
                ]
            ),
        )
        self._assign_ee_target(follower, follower_xyz, frozen_quat)

    def reset_yaw_alignment(self) -> bool:
        """Re-align the operator's current forward direction with the robot's forward.

        Reads the current HMD pose, extracts the yaw angle, and sets the yaw-offset
        rotation so that the operator's "forward" maps to the robot's +X direction.

        Call this on a button press (or at startup) whenever the operator or robot
        has physically re-oriented.

        Returns False and keeps the previous offset when the headset is not worn
        upright (e.g. lying on a table), since its heading is then meaningless.
        """
        if not self._enable_dynamic_yaw:
            return False
        hmd_pose = self.xr_client.get_pose_by_name("headset")
        # XR SDK returns [x, y, z, qx, qy, qz, qw]; convert to [w, x, y, z]
        hmd_quat = np.array([hmd_pose[6], hmd_pose[3], hmd_pose[4], hmd_pose[5]])
        if np.linalg.norm(hmd_quat) < 0.5:
            print("[YawAlign] Skipped: no headset pose yet. Put on the headset, face forward, "
                  "then press right stick down (or just grip).")
            return False
        R_hmd_world = self.R_headset_world @ tf.quaternion_matrix(hmd_quat)[:3, :3]
        up = R_hmd_world @ np.array([0.0, 1.0, 0.0])
        if up[2] < HMD_UPRIGHT_MIN_UP_Z:
            print(f"[YawAlign] Skipped: headset not upright (up.z={up[2]:.2f}), offset unchanged. "
                  "Put on the headset, face forward, then press right stick down (or just grip).")
            return False
        # Heading from the headset's right axis: unaffected by looking up or down.
        right = R_hmd_world @ np.array([1.0, 0.0, 0.0])
        yaw = float(np.arctan2(right[0], -right[1]))
        self._R_yaw_offset = yaw_rotation_matrix(-yaw)
        self._yaw_aligned = True

        # Clear all grip references so active control doesn't use
        # mismatched coordinate frames (old ref vs new controller coords).
        # User must re-grip to resume control — the arm stays in place.
        cleared = sum(1 for k in self.ref_ee_xyz if self.ref_ee_xyz[k] is not None)
        self.ref_ee_xyz = {k: None for k in self.ref_ee_xyz}
        self.ref_ee_quat = {k: None for k in self.ref_ee_quat}
        self.ref_controller_xyz = {k: None for k in self.ref_controller_xyz}
        self.ref_controller_quat = {k: None for k in self.ref_controller_quat}

        yaw_deg = float(np.degrees(yaw))
        clear_msg = f" ({cleared} active grip(s) cleared)" if cleared else ""
        print(f"[YawAlign] Reset: HMD yaw={yaw_deg:.1f} deg, "
              f"offset applied — operator forward → robot +X{clear_msg}")
        return True

    @staticmethod
    def _soft_deadband(value: np.ndarray, threshold: float) -> np.ndarray:
        """Smooth deadband with cubic ease-in to avoid motion discontinuity.

        - [0, threshold]:     zero (suppress noise)
        - [threshold, 2*thr]: smoothstep ramp from 0 → 1
        - [2*threshold, ∞):   pass-through (full delta)
        """
        norm = float(np.linalg.norm(value))
        if norm < threshold:
            return np.zeros_like(value)
        if norm < 2.0 * threshold:
            t = (norm - threshold) / threshold  # [0, 1]
            s = t * t * (3.0 - 2.0 * t)  # smoothstep
            return value * s
        return value

    def _process_xr_pose(self, xr_pose, src_name):
        """Process the current XR controller pose with soft deadband to suppress jitter."""
        # Get position and orientation
        controller_xyz = np.array([xr_pose[0], xr_pose[1], xr_pose[2]])
        controller_quat = [
            xr_pose[6],  # w
            xr_pose[3],  # x
            xr_pose[4],  # y
            xr_pose[5],  # z
        ]

        R_eff = self._R_yaw_offset @ self.R_headset_world
        controller_xyz = R_eff @ controller_xyz

        R_transform = np.eye(4)
        R_transform[:3, :3] = R_eff
        R_quat = tf.quaternion_from_matrix(R_transform)
        controller_quat = tf.quaternion_multiply(
            tf.quaternion_multiply(R_quat, controller_quat),
            tf.quaternion_conjugate(R_quat),
        )

        if self.ref_controller_xyz[src_name] is None:
            self.ref_controller_xyz[src_name] = controller_xyz
            self.ref_controller_quat[src_name] = controller_quat

            delta_xyz = np.zeros(3)
            delta_rot = np.array([0.0, 0.0, 0.0])
        else:
            delta_xyz = (controller_xyz - self.ref_controller_xyz[src_name]) * self.scale_factor
            delta_rot = quat_diff_as_angle_axis(self.ref_controller_quat[src_name], controller_quat)

            # Soft deadband: suppress noise without introducing motion discontinuity
            delta_xyz = self._soft_deadband(delta_xyz, self._delta_deadband_xyz)
            delta_rot = self._soft_deadband(delta_rot, self._delta_deadband_rot)

        return delta_xyz, delta_rot

    def _placo_setup(self):
        """Set up the placo inverse kinematics solver."""
        from xrobotoolkit_teleop.utils.path_utils import resolve_ros_package_uris_in_urdf

        resolved_urdf = resolve_ros_package_uris_in_urdf(self.robot_urdf_path)
        self.placo_robot = placo.RobotWrapper(resolved_urdf)
        print("Joint names in the Placo model:")
        for joint_name in self.placo_robot.model.names:
            print(f"  {joint_name}")

        self.solver = placo.KinematicsSolver(self.placo_robot)
        self.solver.dt = self.dt
        # self.solver.add_kinetic_energy_regularization_task(1e-6)

        # Set initial configuration
        if self.q_init is not None:
            if self.floating_base:
                self.placo_robot.state.q = self.q_init.copy()
            else:
                self.solver.mask_fbase(True)
                self.placo_robot.state.q[7:] = self.q_init.copy()
        else:
            if not self.floating_base:
                self.solver.mask_fbase(True)
            self.placo_robot.state.q[:7] = np.array([0, 0, 0, 0, 0, 0, 1])  # Identity quaternion for base

        self.placo_robot.update_kinematics()

        # Set up end effector tasks
        for name, config in self.manipulator_config.items():
            # Get control mode (default to "pose" for backward compatibility)
            control_mode = config.get("control_mode", "pose")
            self.effector_control_mode[name] = control_mode
            
            ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
            
            if control_mode == "position":
                # Position-only control
                self.effector_task[name] = self.solver.add_position_task(config["link_name"], ee_xyz)
                print(f"Created position task for {name} -> {config['link_name']}")
            else:
                # Full pose control (default)
                ee_target = tf.quaternion_matrix(ee_quat)
                ee_target[:3, 3] = ee_xyz
                self.effector_task[name] = self.solver.add_frame_task(config["link_name"], ee_target)
                print(f"Created pose task for {name} -> {config['link_name']}")
            
            self.effector_task[name].configure(name, "soft", 1.0)
            manipulability = self.solver.add_manipulability_task(config["link_name"], "both", 1.0)
            manipulability.configure("manipulability", "soft", 1e-2)

            # Set up motion tracker tasks if configured (position only)
            if "motion_tracker" in config:
                tracker_config = config["motion_tracker"]
                link_target = tracker_config["link_target"]

                # Get current position of the target link
                target_xyz, _ = self._get_link_pose(link_target)

                # Create position task for motion tracker target (xyz only)
                tracker_task_name = f"{name}_tracker"
                self.motion_tracker_task[name] = self.solver.add_position_task(link_target, target_xyz)
                self.motion_tracker_task[name].configure(tracker_task_name, "soft", 1.0)

                print(f"Motion tracker position task created for {name} -> {link_target}")

        self.placo_robot.update_kinematics()

    def _update_ik(self):
        """
        This is the core IK logic block. It reads from XR, updates Placo tasks,
        and solves the kinematics.
        """
        self._update_robot_state()
        self.placo_robot.update_kinematics()

        for src_name, config in self.manipulator_config.items():
            if self._is_follow_follower(src_name):
                continue

            grip_name = config["control_trigger"]
            pose_source = config["pose_source"]
            xr_grip_val = self.xr_client.get_key_value_by_name(grip_name)
            was_active = self.active.get(src_name, False)
            if was_active:
                self.active[src_name] = xr_grip_val > self.control_deactivate_threshold
            else:
                self.active[src_name] = xr_grip_val > self.control_activate_threshold

            if self.active[src_name]:
                if self.ref_ee_xyz[src_name] is None:
                    if self._enable_dynamic_yaw and not self._yaw_aligned:
                        self.reset_yaw_alignment()
                    print(f"{src_name} is activated.")
                    # Subclasses may return the last held teleop pose so re-grip
                    # continues from the frozen command instead of a sagged measurement.
                    self.ref_ee_xyz[src_name], self.ref_ee_quat[src_name] = (
                        self._get_activation_ee_pose(src_name, config)
                    )

                xr_pose = self.xr_client.get_pose_by_name(pose_source)
                delta_xyz, delta_rot = self._process_xr_pose(xr_pose, src_name)
                target_xyz = self.ref_ee_xyz[src_name] + delta_xyz

                if (
                    self.follow_mode
                    and src_name == self._follow_leader
                    and self._follow_quat_leader is not None
                ):
                    # Follow mode: Pico world X/Z only; keep captured Y and orientation.
                    ref_xyz = self.ref_ee_xyz[src_name]
                    target_xyz = self._follow_xz_only_xyz(
                        src_name,
                        np.array(
                            [
                                ref_xyz[0] + float(delta_xyz[0]),
                                0.0,
                                ref_xyz[2] + float(delta_xyz[2]),
                            ]
                        ),
                    )
                    self._assign_ee_target(src_name, target_xyz, self._follow_quat_leader)
                elif self.effector_control_mode[src_name] == "position":
                    self.effector_task[src_name].target_world = target_xyz
                else:
                    target_xyz, target_quat = apply_delta_pose(
                        self.ref_ee_xyz[src_name],
                        self.ref_ee_quat[src_name],
                        delta_xyz,
                        delta_rot,
                    )
                    target_pose = tf.quaternion_matrix(target_quat)
                    target_pose[:3, 3] = target_xyz
                    self.effector_task[src_name].T_world_frame = target_pose
            else:
                if self.ref_ee_xyz[src_name] is not None:
                    print(f"{src_name} is deactivated.")
                    self.ref_ee_xyz[src_name] = None
                    self.ref_ee_quat[src_name] = None
                    self.ref_controller_xyz[src_name] = None
                    self.ref_controller_quat[src_name] = None

                # Hold IK target at the inactive EE pose to prevent target jump on re-grip.
                # Without this, the solver keeps the last active target while the arm may
                # drift under gravity, causing a sudden drop when the user re-grips.
                ee_xyz, ee_quat = self._get_inactive_ee_pose(src_name, config)
                if (
                    self.follow_mode
                    and src_name == self._follow_leader
                    and self._follow_quat_leader is not None
                ):
                    ee_quat = self._follow_quat_leader
                    ee_xyz = self._follow_xz_only_xyz(src_name, ee_xyz)
                self._assign_ee_target(src_name, ee_xyz, ee_quat)

        self._apply_follow_follower_task()

        # Process motion tracker data
        self._update_motion_tracker_tasks()

        try:
            self.solver.solve(True)
        except RuntimeError as e:
            print(f"IK solver failed: {e}")

        self._post_ik_solve()

    def _get_activation_ee_pose(self, src_name: str, config: dict):
        """EE pose used as the teleop reference when grip is newly pressed."""
        return self._get_link_pose(config["link_name"])

    def _get_inactive_ee_pose(self, src_name: str, config: dict):
        """EE pose used to park the IK target while grip is released."""
        return self._get_link_pose(config["link_name"])

    def _post_ik_solve(self):
        """Hook for subclasses after each IK solve (e.g. restore held joints)."""
        pass

    def _update_motion_tracker_tasks(self):
        """Process motion tracker data and update corresponding Placo tasks."""
        motion_tracker_data = self.xr_client.get_motion_tracker_data()

        for src_name, config in self.manipulator_config.items():
            # Skip if no motion tracker configured for this end effector
            if "motion_tracker" not in config:
                continue

            # Skip if main controller is not active
            if not self.active.get(src_name, False):
                # Reset motion tracker references when controller is inactive
                if src_name in self.ref_tracker_xyz:
                    del self.ref_tracker_xyz[src_name]
                    del self.ref_robot_xyz[src_name]
                continue

            tracker_config = config["motion_tracker"]
            serial = tracker_config["serial"]

            # Skip if this tracker is not available
            if serial not in motion_tracker_data:
                continue

            # Get motion tracker pose
            tracker_pose = motion_tracker_data[serial]["pose"]
            tracker_xyz = (self._R_yaw_offset @ self.R_headset_world) @ np.array(tracker_pose[:3])

            # Initialize reference positions on first detection
            if src_name not in self.ref_tracker_xyz:
                self.ref_tracker_xyz[src_name] = tracker_xyz.copy()
                # Get current robot end-effector position as baseline
                robot_xyz, _ = self._get_link_pose(config["motion_tracker"]["link_target"])
                self.ref_robot_xyz[src_name] = robot_xyz.copy()
                continue

            # Calculate movement delta from tracker's initial position
            tracker_delta = tracker_xyz - self.ref_tracker_xyz[src_name]

            # Apply scaled tracker movement to robot's initial position
            final_target_xyz = self.ref_robot_xyz[src_name] + tracker_delta * self.scale_factor

            # Update motion tracker task target position
            if src_name in self.motion_tracker_task:
                self.motion_tracker_task[src_name].target_world = final_target_xyz

    def _init_placo_viz(self):
        self.placo_vis = robot_viz(self.placo_robot)
        # 在无 GUI 的容器/远程环境（例如 VSCode Dev Container）中，webbrowser.open()
        # 可能会调用 xdg-open 并阻塞，从而表现为“卡住在打印 Viewer URL 之后”。
        # 允许通过环境变量禁用自动打开浏览器：XROBOT_DISABLE_BROWSER=1
        disable_browser = os.environ.get("XROBOT_DISABLE_BROWSER", "").lower() in ("1", "true", "yes")
        headless = not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY")
        if disable_browser or headless:
            print("Browser auto-open skipped (headless or XROBOT_DISABLE_BROWSER=1).")
            print(f"Viewer URL: {self.placo_vis.viewer.url()}")
        else:
            webbrowser.open(self.placo_vis.viewer.url())
        self.placo_vis.display(self.placo_robot.state.q)
        for name, config in self.manipulator_config.items():
            robot_frame_viz(self.placo_robot, config["link_name"])
            
            # Show appropriate visualization based on control mode
            if self.effector_control_mode[name] == "position":
                # Create a frame matrix for position-only visualization
                target_frame = np.eye(4)
                target_frame[:3, 3] = self.effector_task[name].target_world
                frame_viz(f"vis_target_{name}", target_frame)
            else:
                # Full pose visualization
                frame_viz(f"vis_target_{name}", self.effector_task[name].T_world_frame)

            # Visualize motion tracker target if configured
            if "motion_tracker" in config and name in self.motion_tracker_task:
                link_target = config["motion_tracker"]["link_target"]
                robot_frame_viz(self.placo_robot, link_target)
                # Create a frame matrix for visualization
                tracker_frame = np.eye(4)
                tracker_frame[:3, 3] = self.motion_tracker_task[name].target_world
                frame_viz(f"vis_tracker_{name}", tracker_frame)

    def _update_placo_viz(self):
        self.placo_vis.display(self.placo_robot.state.q)
        for name, config in self.manipulator_config.items():
            robot_frame_viz(self.placo_robot, config["link_name"])
            
            # Show appropriate visualization based on control mode
            if self.effector_control_mode[name] == "position":
                # Create a frame matrix for position-only visualization
                target_frame = np.eye(4)
                target_frame[:3, 3] = self.effector_task[name].target_world
                frame_viz(f"vis_target_{name}", target_frame)
            else:
                # Full pose visualization
                frame_viz(f"vis_target_{name}", self.effector_task[name].T_world_frame)

            # Update motion tracker target visualization if configured
            if "motion_tracker" in config and name in self.motion_tracker_task:
                link_target = config["motion_tracker"]["link_target"]
                robot_frame_viz(self.placo_robot, link_target)
                # Create a frame matrix for visualization
                tracker_frame = np.eye(4)
                tracker_frame[:3, 3] = self.motion_tracker_task[name].target_world
                frame_viz(f"vis_tracker_{name}", tracker_frame)

    def sync_end_effector_poses_to_placo_tasks(self):
        """
        Syncs the current end effector link poses to their corresponding placo tasks.
        This is useful for initializing or resetting task targets to current robot state.
        """
        for name, config in self.manipulator_config.items():
            # Get current link pose
            ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
            
            # Update the corresponding placo task
            if self.effector_control_mode[name] == "position":
                # Position-only control: update target position
                self.effector_task[name].target_world = ee_xyz
            else:
                # Full pose control: update target pose
                ee_target = tf.quaternion_matrix(ee_quat)
                ee_target[:3, 3] = ee_xyz
                self.effector_task[name].T_world_frame = ee_target
            
            print(f"Synced {name} end effector pose to placo task: {config['link_name']}")

    def _update_gripper_target(self):
        for gripper_name in self.manipulator_config.keys():
            if "gripper_config" not in self.manipulator_config[gripper_name]:
                continue

            gripper_config = self.manipulator_config[gripper_name]["gripper_config"]
            gripper_config = self.manipulator_config[gripper_name]["gripper_config"]
            gripper_type = gripper_config["type"]
            if gripper_type == "parallel":
                trigger_value = self.xr_client.get_key_value_by_name(gripper_config["gripper_trigger"])
                for joint_name, open_pos, close_pos in zip(
                    gripper_config["joint_names"],
                    gripper_config["open_pos"],
                    gripper_config["close_pos"],
                ):
                    # Calculate the target position based on the trigger value
                    gripper_pos = calc_parallel_gripper_position(open_pos, close_pos, trigger_value)
                    self.gripper_pos_target[gripper_name][joint_name] = gripper_pos
                    self.gripper_pos_target[gripper_name][joint_name] = gripper_pos
            else:
                # TODO: add dexterous hand support
                raise ValueError(f"Unsupported gripper type: {gripper_type}")

    def _log_data(self):
        """
        Logs the current state of the robot, including joint positions, end effector poses,
        and any other relevant data
        """
        if self.enable_log_data:
            raise NotImplementedError

    # ---------------------------------------------------------
    # --- Abstract Methods (to be implemented by subclasses) ---
    # ---------------------------------------------------------

    @abc.abstractmethod
    def _robot_setup(self):
        """Initializes the specific backend (connects to robot, starts sim, etc.)."""
        raise NotImplementedError

    @abc.abstractmethod
    def _update_robot_state(self):
        """Reads the current joint states from the robot/sim and updates self.placo_robot.state.q."""
        raise NotImplementedError

    @abc.abstractmethod
    def _send_command(self):
        """Sends the calculated target joint positions from self.placo_robot.state.q to the robot/sim."""
        raise NotImplementedError

    @abc.abstractmethod
    def _get_link_pose(self, link_name):
        """Gets the current world pose for a given link name."""
        raise NotImplementedError

    @abc.abstractmethod
    def run(self):
        """
        The main entry point. Subclasses must implement this to define their
        execution model (single-threaded or multi-threaded).
        """
        raise NotImplementedError
