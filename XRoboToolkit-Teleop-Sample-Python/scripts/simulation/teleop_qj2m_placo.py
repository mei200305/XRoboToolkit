"""
QJ2M Placo Simulation — dual-arm teleop with legs/waist locked.

Left stick up: one-key homing (left arm first, then right; each J1–J3 → J4–J7).
Right stick up: toggle left-follow mode (right Pico world X/Z; Y and orientations locked).

Homing uses measured HOME_Q_*; online IK soft-prefers ideal PREF_Q
[0.52, 0, 0, -2.09, 0, 0, 0] for both arms (ISSUE-001 elbow-out bias).
"""
import os, time
import numpy as np
import tyro
import meshcat.transformations as tf

from xrobotoolkit_teleop.simulation.placo_teleop_controller import PlacoTeleopController
from xrobotoolkit_teleop.utils.path_utils import QJ2M_URDF_PATH
from xrobotoolkit_teleop.utils.qj2m_homing import (
    HOME_Q_LEFT,
    HOME_Q_RIGHT,
    HOMING_DURATION,
    JOINTS_TASK_WEIGHT,
    build_homing_targets,
    pref_joints_dict,
    sample_left_then_right_homing,
)

LEFT_ARM_JOINTS = [
    "arml_1_joint", "arml_2_joint", "arml_3_joint", "arml_4_joint",
    "arml_5_joint", "arml_6_joint", "arml_7_joint",
]
RIGHT_ARM_JOINTS = [
    "armr_1_joint", "armr_2_joint", "armr_3_joint", "armr_4_joint",
    "armr_5_joint", "armr_6_joint", "armr_7_joint",
]
ARM_JOINT_NAMES = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

LEFT_STICK_UP_THRESH = 0.7
RIGHT_STICK_UP_THRESH = 0.7


class QJ2MPlacoTeleopController(PlacoTeleopController):
    """Placo controller that locks non-arm joints and supports left-stick homing."""

    def _placo_setup(self):
        super()._placo_setup()

        # Lock non-arm joints
        locked = 0
        for name in self.placo_robot.model.names:
            if name in ("universe", "root_joint"):
                continue
            if name not in ARM_JOINT_NAMES:
                self.solver.mask_dof(name)
                locked += 1

        # Arm joint slices
        left_first = self.placo_robot.get_joint_offset("arml_1_joint")
        left_last  = self.placo_robot.get_joint_offset("arml_7_joint")
        right_first = self.placo_robot.get_joint_offset("armr_1_joint")
        right_last  = self.placo_robot.get_joint_offset("armr_7_joint")
        self._arm_slice = {
            "left_arm": slice(left_first, left_last + 1),
            "right_arm": slice(right_first, right_last + 1),
        }
        print(f"[QJ2M] Locked {locked} non-arm joints. "
              f"Left: {self._arm_slice['left_arm']}, Right: {self._arm_slice['right_arm']}")

        # Ready posture
        self._home_q_left = HOME_Q_LEFT.copy()
        self._home_q_right = HOME_Q_RIGHT.copy()
        self.placo_robot.state.q[self._arm_slice["left_arm"]] = self._home_q_left
        self.placo_robot.state.q[self._arm_slice["right_arm"]] = self._home_q_right
        self.placo_robot.update_kinematics()

        # Refresh EE task targets to match ready pose (created at zero in super())
        for name, config in self.manipulator_config.items():
            ee_xyz, ee_quat = self._get_link_pose(config["link_name"])
            ee_target = tf.quaternion_matrix(ee_quat)
            ee_target[:3, 3] = ee_xyz
            self.effector_task[name].T_world_frame = ee_target

        print(f"[QJ2M] Home ready pose: L={np.round(self._home_q_left, 3)} "
              f"R={np.round(self._home_q_right, 3)}")

        # Null-space preference: ideal symmetric PREF_Q (not HOME). ISSUE-001.
        self._joints_task = self.solver.add_joints_task()
        self._joints_task.set_joints(pref_joints_dict(LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS))
        self._joints_task.configure("joints_regularization", "soft", JOINTS_TASK_WEIGHT)
        print(
            f"[QJ2M] joints_task preference weight={JOINTS_TASK_WEIGHT} "
            f"toward ideal PREF_Q=[0.52, 0, 0, -2.09, 0, 0, 0] (both arms)"
        )

        # Held joint command while inactive (prevents IK/manipulability drift)
        self._hold_q_left = self._home_q_left.copy()
        self._hold_q_right = self._home_q_right.copy()

        # Homing state
        self._homing = False
        self._homing_t0 = 0.0
        self._homing_q0_left = None
        self._homing_q0_right = None
        self._homing_via_left = None
        self._homing_via_right = None
        self._homing_target_left = None
        self._homing_target_right = None
        self._prev_left_stick_up = False
        self._prev_right_stick_up = False

    def _force_deactivate_arms(self):
        """Release teleop so grip does not resume motion after / during homing."""
        for name in self.manipulator_config:
            self.active[name] = False
        self._reset_control_refs()

    def _start_homing(self):
        self._force_deactivate_arms()
        q = self.placo_robot.state.q
        self._homing = True
        self._homing_t0 = time.time()
        self._homing_q0_left  = q[self._arm_slice["left_arm"]].copy()
        self._homing_q0_right = q[self._arm_slice["right_arm"]].copy()
        (
            self._homing_via_left,
            self._homing_via_right,
            self._homing_target_left,
            self._homing_target_right,
        ) = build_homing_targets(
            self._homing_q0_left,
            self._homing_q0_right,
            self._home_q_left,
            self._home_q_right,
        )
        print(
            f"[QJ2M] Left stick up → sequential homing (left then right) "
            f"via L={np.round(self._homing_via_left, 2)} "
            f"→ L={np.round(self._homing_target_left, 2)} "
            f"| via R={np.round(self._homing_via_right, 2)} "
            f"→ R={np.round(self._homing_target_right, 2)}"
        )

    def _sync_effector_tasks_to_robot(self):
        """Keep IK targets on the current EE frames (prevents floating targets)."""
        self.placo_robot.update_kinematics()
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

    def _reset_control_refs(self):
        """Clear teleop references without deleting dict keys (avoids KeyError)."""
        for k in self.ref_ee_xyz:
            self.ref_ee_xyz[k] = None
        for k in self.ref_ee_quat:
            self.ref_ee_quat[k] = None
        for k in self.ref_controller_xyz:
            self.ref_controller_xyz[k] = None
        for k in self.ref_controller_quat:
            self.ref_controller_quat[k] = None

    def _apply_hold_pose(self):
        """Freeze inactive arms at last held joints (blocks soft-IK drift)."""
        if not self.active.get("left_arm", False) and self._hold_q_left is not None:
            self.placo_robot.state.q[self._arm_slice["left_arm"]] = self._hold_q_left
        if not self.active.get("right_arm", False) and self._hold_q_right is not None:
            self.placo_robot.state.q[self._arm_slice["right_arm"]] = self._hold_q_right
        self.placo_robot.update_kinematics()

    def _apply_homing(self):
        if not self._homing:
            return False
        elapsed = time.time() - self._homing_t0
        t = elapsed / HOMING_DURATION
        if t >= 1.0:
            self.placo_robot.state.q[self._arm_slice["left_arm"]] = self._homing_target_left.copy()
            self.placo_robot.state.q[self._arm_slice["right_arm"]] = self._homing_target_right.copy()
            self._hold_q_left = self._homing_target_left.copy()
            self._hold_q_right = self._homing_target_right.copy()
            self._homing = False
            self._force_deactivate_arms()
            self._sync_effector_tasks_to_robot()
            print("[QJ2M] Homing complete — arms frozen at home.")
            self._recapture_follow_offset_if_enabled()
        else:
            ql, qr = sample_left_then_right_homing(
                self._homing_q0_left,
                self._homing_via_left,
                self._homing_target_left,
                self._homing_q0_right,
                self._homing_via_right,
                self._homing_target_right,
                t,
            )
            self.placo_robot.state.q[self._arm_slice["left_arm"]] = ql
            self.placo_robot.state.q[self._arm_slice["right_arm"]] = qr
            self._hold_q_left = ql.copy()
            self._hold_q_right = qr.copy()
            self._sync_effector_tasks_to_robot()
        return True

    def _update_ik(self):
        # If neither arm is gripped, skip IK solve entirely — avoids soft-task drift.
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
            for name in self.manipulator_config:
                self.active[name] = False
            self._reset_control_refs()
            self._apply_hold_pose()
            self._sync_effector_tasks_to_robot()
            return

        super()._update_ik()
        # After solve: keep inactive arm frozen; refresh hold for active arm
        if self.active.get("left_arm", False):
            self._hold_q_left = self.placo_robot.state.q[self._arm_slice["left_arm"]].copy()
        if self.active.get("right_arm", False):
            self._hold_q_right = self.placo_robot.state.q[self._arm_slice["right_arm"]].copy()
        self._apply_hold_pose()
        self._sync_effector_tasks_to_robot()

    def run(self):
        while not self._stop_event.is_set():
            try:
                start = time.time()

                # Left stick up → one-key homing (rising edge)
                left_axis = self.xr_client.get_joystick_state("left")
                left_stick_up = left_axis[1] > LEFT_STICK_UP_THRESH
                if left_stick_up and not self._prev_left_stick_up and not self._homing:
                    self._start_homing()
                self._prev_left_stick_up = left_stick_up

                # Right stick up → toggle left-follow mode (rising edge)
                right_axis = self.xr_client.get_joystick_state("right")
                right_stick_up = right_axis[1] > RIGHT_STICK_UP_THRESH
                if right_stick_up and not self._prev_right_stick_up and not self._homing:
                    self.toggle_follow_mode()
                self._prev_right_stick_up = right_stick_up

                if self._homing:
                    self._apply_homing()
                else:
                    self._update_ik()

                self._send_command()

                elapsed = time.time() - start
                time.sleep(max(0, self.dt - elapsed))
            except KeyboardInterrupt:
                print("\nStopped.")
                self._stop_event.set()


def main(
    robot_urdf_path: str = QJ2M_URDF_PATH,
    scale_factor: float = 1.0,
):
    controller = QJ2MPlacoTeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config={
            "right_arm": {
                "link_name": "armr_7",
                "pose_source": "right_controller",
                "control_trigger": "right_grip",
                "gripper_config": {
                    "type": "parallel", "gripper_trigger": "right_trigger",
                    "joint_names": ["right_gripper_virtual"],
                    "open_pos": [0.0], "close_pos": [1.0],
                },
            },
            "left_arm": {
                "link_name": "arml_7",
                "pose_source": "left_controller",
                "control_trigger": "left_grip",
                "gripper_config": {
                    "type": "parallel", "gripper_trigger": "left_trigger",
                    "joint_names": ["left_gripper_virtual"],
                    "open_pos": [0.0], "close_pos": [1.0],
                },
            },
        },
        floating_base=False,
        scale_factor=scale_factor,
        dt=0.01,
    )

    print("=" * 60)
    print("QJ2M Placo simulation — ARMS ONLY")
    print("=" * 60)
    print("Right grip → right arm   |  Left grip → left arm")
    print("Trigger    → gripper      |  Left stick up → homing")
    print("Right stick up → toggle left-follow (right X/Z, Y and orientations locked)")
    print("=" * 60)
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
