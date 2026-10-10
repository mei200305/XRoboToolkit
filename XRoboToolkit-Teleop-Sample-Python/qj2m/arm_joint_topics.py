#!/usr/bin/env python3
"""手臂关节 ROS 话题常量（无 MuJoCo 依赖）。关节名与 assets/qj2m 新 URDF 一致。"""

from __future__ import annotations

N_UPPER_MOTORS = 32
LEFT_ARM_MOTOR_IDX = list(range(3, 10))
RIGHT_ARM_MOTOR_IDX = list(range(10, 17))
ROS_HZ = 500
DEFAULT_JOINT_CMD_TOPIC = "/throw/arm_joint_command"
DEFAULT_WAIST_YAW_TOPIC = "/throw/waist_yaw_command"
DEFAULT_POT_MOUTH_TOPIC = "/throw/pot_mouth"

LEFT_ARM_JOINT_NAMES = [
    "arml_1_joint",
    "arml_2_joint",
    "arml_3_joint",
    "arml_4_joint",
    "arml_5_joint",
    "arml_6_joint",
    "arml_7_joint",
]
RIGHT_ARM_JOINT_NAMES = [
    "armr_1_joint",
    "armr_2_joint",
    "armr_3_joint",
    "armr_4_joint",
    "armr_5_joint",
    "armr_6_joint",
    "armr_7_joint",
]
ARM_JOINT_NAMES = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES

# /throw/arm_joint_command 扁平数组（各 28）:
#   position = [左臂 q(7), 右臂 q(7), 左臂 kp(7), 右臂 kp(7)]
#   velocity = [左臂 qd(7), 右臂 qd(7), 左臂 kd(7), 右臂 kd(7)]
# Match Code/rl_control_ws config_wbc_23dof.yaml policies.disturb kps/kds (arm joints)
LEFT_ARM_KP = [200.0, 160.0, 160.0, 180.0, 80.0, 80.0, 80.0]
RIGHT_ARM_KP = [200.0, 160.0, 160.0, 180.0, 80.0, 80.0, 80.0]
LEFT_ARM_KD = [5.0, 4.0, 3.0, 3.0, 2.0, 2.0, 2.0]
RIGHT_ARM_KD = [5.0, 4.0, 3.0, 3.0, 2.0, 2.0, 2.0]
ARM_JOINT_CMD_VECTOR_LEN = 28


def build_arm_joint_command_arrays(
    q_left,
    qd_left,
    q_right,
    qd_right,
    *,
    kp_left=None,
    kd_left=None,
    kp_right=None,
    kd_right=None,
) -> tuple[list[float], list[float]]:
    """生成 /throw/arm_joint_command 的 position、velocity 各 28 元。"""
    ql = [float(x) for x in q_left]
    qdl = [float(x) for x in qd_left]
    qr = [float(x) for x in q_right]
    qdr = [float(x) for x in qd_right]
    kpl = list(kp_left if kp_left is not None else LEFT_ARM_KP)
    kdl = list(kd_left if kd_left is not None else LEFT_ARM_KD)
    kpr = list(kp_right if kp_right is not None else RIGHT_ARM_KP)
    kdr = list(kd_right if kd_right is not None else RIGHT_ARM_KD)
    position = ql + qr + kpl + kpr
    velocity = qdl + qdr + kdl + kdr
    return position, velocity


def build_arm_joint_command_msg(
    q_left,
    qd_left,
    q_right,
    qd_right,
    *,
    stamp=None,
    kp_left=None,
    kd_left=None,
    kp_right=None,
    kd_right=None,
):
    """生成 JointState（position/velocity 各 28，含 kp/kd）。"""
    from sensor_msgs.msg import JointState

    position, velocity = build_arm_joint_command_arrays(
        q_left,
        qd_left,
        q_right,
        qd_right,
        kp_left=kp_left,
        kd_left=kd_left,
        kp_right=kp_right,
        kd_right=kd_right,
    )
    msg = JointState()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.name = list(ARM_JOINT_NAMES)
    msg.position = position
    msg.velocity = velocity
    msg.effort = []
    return msg
