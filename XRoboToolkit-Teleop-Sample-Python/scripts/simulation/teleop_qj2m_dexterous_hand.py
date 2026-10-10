"""
QJ2M dual-arm teleop + Inspire Dexterous Hands via PICO hand tracking.

Arms:  XR controller pose  → Placo IK → QJ2M arm joints
Hands: PICO hand tracking → dex_retargeting → Inspire Hand finger joints

The arms use the QJ2M URDF.  The hands use separate Inspire Hand URDFs.
Both are visualized together in MeshCat.
"""
import os, time
import numpy as np
import placo
import tyro
from dex_retargeting.constants import HandType, RetargetingType, RobotName
from placo_utils.visualization import robot_viz, frame_viz

from xrobotoolkit_teleop.common.xr_client import XrClient
from xrobotoolkit_teleop.utils.dex_hand_utils import (
    DexHandTracker,
    pico_hand_state_to_mediapipe,
)
from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD, apply_delta_pose, quat_diff_as_angle_axis
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH, QJ2M_URDF_PATH

import meshcat.transformations as tf

# --- Config ---
QJ2M_URDF = QJ2M_URDF_PATH
INSPIRE_LEFT_URDF = os.path.join(ASSET_PATH, "inspire_hand/inspire_hand_left.urdf")
INSPIRE_RIGHT_URDF = os.path.join(ASSET_PATH, "inspire_hand/inspire_hand_right.urdf")

ARM_JOINTS = {
    "left": [
        "arml_1_joint", "arml_2_joint", "arml_3_joint", "arml_4_joint",
        "arml_5_joint", "arml_6_joint", "arml_7_joint",
    ],
    "right": [
        "armr_1_joint", "armr_2_joint", "armr_3_joint", "armr_4_joint",
        "armr_5_joint", "armr_6_joint", "armr_7_joint",
    ],
}
EE_LINKS = {"left": "arml_7", "right": "armr_7"}


def get_link_pose(robot, link_name):
    T = robot.get_T_world_frame(link_name)
    return T[:3, 3].copy(), tf.quaternion_from_matrix(T)


def main(scale_factor: float = 1.0):
    # --- Robot models ---
    print("Loading QJ2M...")
    from xrobotoolkit_teleop.utils.path_utils import resolve_ros_package_uris_in_urdf
    robot = placo.RobotWrapper(resolve_ros_package_uris_in_urdf(QJ2M_URDF))
    solver = placo.KinematicsSolver(robot)
    solver.dt = 0.01
    solver.mask_fbase(True)

    # Lock non-arm joints (legs, waist, neck)
    arm_dofs = set()
    for names in ARM_JOINTS.values():
        for n in names:
            arm_dofs.add(robot.get_joint_offset(n))
    for i in range(7, solver.N):
        if i not in arm_dofs:
            solver.mask_dof(i, True)
    print(f"QJ2M loaded: {solver.N} DOFs total, {len(arm_dofs)} arm DOFs free")

    # End-effector tasks
    ee_tasks = {}
    ref_ee = {}
    ref_ctrl = {}
    active = {}
    for side in ["left", "right"]:
        ee_xyz, ee_quat = get_link_pose(robot, EE_LINKS[side])
        ee_target = tf.quaternion_matrix(ee_quat)
        ee_target[:3, 3] = ee_xyz
        ee_tasks[side] = solver.add_frame_task(EE_LINKS[side], ee_target)
        ee_tasks[side].configure(f"{side}_arm", "soft", 1.0)
        solver.add_manipulability_task(EE_LINKS[side], "both", 1.0).configure("manip", "soft", 1e-2)
        ref_ee[side] = None
        ref_ctrl[side] = None
        active[side] = False

    robot.update_kinematics()

    # --- Dexterous hand trackers ---
    print("Loading Inspire Hands...")
    left_hand_robot = placo.RobotWrapper(INSPIRE_LEFT_URDF)
    right_hand_robot = placo.RobotWrapper(INSPIRE_RIGHT_URDF)
    left_tracker = DexHandTracker(
        robot_name=RobotName.inspire, urdf_path=INSPIRE_LEFT_URDF,
        hand_type=HandType.left, retargeting_type=RetargetingType.vector)
    right_tracker = DexHandTracker(
        robot_name=RobotName.inspire, urdf_path=INSPIRE_RIGHT_URDF,
        hand_type=HandType.right, retargeting_type=RetargetingType.vector)

    # --- Visualization ---
    qj2m_viz = robot_viz(robot, "QJ2M")
    left_hand_viz = robot_viz(left_hand_robot, "left_hand")
    right_hand_viz = robot_viz(right_hand_robot, "right_hand")
    frame_viz("left_ee_target", np.eye(4))
    frame_viz("right_ee_target", np.eye(4))

    # --- XR client ---
    xr = XrClient()

    print("=" * 60)
    print("QJ2M + Inspire Dexterous Hands — Simulation")
    print("=" * 60)
    print("Right grip  → right arm")
    print("Left grip   → left arm")
    print("Hand tracking (PICO) → finger joints")
    print("=" * 60)

    # --- Main loop ---
    stop = False
    while not stop:
        try:
            start = time.time()

            # -- Arm IK --
            robot.update_kinematics()

            for side in ["left", "right"]:
                ctrl_name = f"{side}_controller"
                grip_name = f"{side}_grip"
                grip_val = xr.get_key_value_by_name(grip_name)
                old_active = active[side]
                active[side] = grip_val > (0.2 if old_active else 0.3)

                if active[side]:
                    if ref_ee[side] is None:
                        ref_ee[side], _ = get_link_pose(robot, EE_LINKS[side])

                    pose = xr.get_pose_by_name(ctrl_name)
                    if pose is not None:
                        ctrl_xyz = R_HEADSET_TO_WORLD @ np.array(pose[:3])
                        ctrl_quat = [pose[6], pose[3], pose[4], pose[5]]
                        ctrl_xyz = R_HEADSET_TO_WORLD @ ctrl_xyz
                        R_mat = tf.quaternion_matrix(tf.quaternion_from_matrix(
                            np.block([[R_HEADSET_TO_WORLD, np.zeros((3,1))],[0,0,0,1]])))[:3,:3]
                        ctrl_quat = tf.quaternion_multiply(
                            tf.quaternion_from_matrix(np.block([[R_mat, np.zeros((3,1))],[0,0,0,1]])), ctrl_quat)

                        if ref_ctrl[side] is None:
                            ref_ctrl[side] = (ctrl_xyz, ctrl_quat)
                            delta_xyz = np.zeros(3)
                            delta_rot = np.zeros(3)
                        else:
                            delta_xyz = (ctrl_xyz - ref_ctrl[side][0]) * scale_factor
                            delta_rot = quat_diff_as_angle_axis(ref_ctrl[side][1], ctrl_quat)

                        target_xyz, target_quat = apply_delta_pose(
                            ref_ee[side][0], ref_ee[side][1], delta_xyz, delta_rot)
                        T = tf.quaternion_matrix(target_quat)
                        T[:3, 3] = target_xyz
                        ee_tasks[side].T_world_frame = T
                else:
                    ref_ee[side] = None
                    ref_ctrl[side] = None

            solver.solve(True)
            robot.update_kinematics()

            # -- Hand retargeting --
            for hand_side, htype, dextracker, hrobot in [
                ("left", "left", left_tracker, left_hand_robot),
                ("right", "right", right_tracker, right_hand_robot),
            ]:
                hand_state = xr.get_hand_tracking_state(htype)
                if hand_state is not None and not np.all(np.array(hand_state) == 0):
                    mediapipe_state = pico_hand_state_to_mediapipe(np.array(hand_state))
                    qpos = dextracker.retarget(mediapipe_state)
                    hrobot.state.q[7:] = qpos

            # -- Visualization --
            qj2m_viz.display(robot.state.q)
            left_hand_viz.display(left_hand_robot.state.q)
            right_hand_viz.display(right_hand_robot.state.q)

            # Update hand positions to follow arm end effectors
            for side, hrobot in [("left", left_hand_robot), ("right", right_hand_robot)]:
                ee_pos, ee_quat = get_link_pose(robot, EE_LINKS[side])
                T = tf.quaternion_matrix(ee_quat)
                T[:3, 3] = ee_pos
                frame_viz(f"{side}_ee_target", T)

            elapsed = time.time() - start
            time.sleep(max(0, 0.01 - elapsed))

        except KeyboardInterrupt:
            stop = True
            print("\nStopped.")


if __name__ == "__main__":
    tyro.cli(main)
