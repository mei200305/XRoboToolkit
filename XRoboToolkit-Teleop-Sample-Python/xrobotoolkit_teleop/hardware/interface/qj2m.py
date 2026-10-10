"""
QJ2M (QJ Full Humanoid) Robot Hardware Interface (ROS 2).

State:  /motor/q2w_upper_motor_joint_state  (JointState, 32 motors)
        Left arm: indices 4-10, Right arm: indices 11-17
Ctrl:   /throw/arm_joint_command  (JointState, 28D)
        position = [left_q(7), right_q(7), left_kp(7), right_kp(7)]
        velocity = [left_qd(7), right_qd(7), left_kd(7), right_kd(7)]
        /motor/tt_upper_motor_csp_control_data  (Float64MultiArray, 32 motors)
        can_test runs the arm motors (CAN 4-17) in CSP mode and ignores the
        WBC PT path for them, so arm targets are also written here directly.
Hand:   hand_control  (String "open"/"close")
"""
import time

import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2

LEFT_ARM_MOTOR_IDX = list(range(3, 10))   # 0-based: motor_id_04 ~ motor_id_10
RIGHT_ARM_MOTOR_IDX = list(range(10, 17))  # 0-based: motor_id_11 ~ motor_id_17
CONTROL_MOTOR_SIZE = 32  # can_test drops CSP arrays of any other length

CSP_COMMAND_TOPIC = "/motor/tt_upper_motor_csp_control_data"
# CSP is stiff drive-side position control (no kp/kd compliance), so cap joint speed.
CSP_MAX_JOINT_VEL = 3.0  # rad/s

# Match Code/rl_control_ws config_wbc_23dof.yaml policies.disturb kps/kds (arm joints)
LEFT_ARM_KP = [200.0, 180.0, 160.0, 200.0, 80.0, 80.0, 80.0]
RIGHT_ARM_KP = [200.0, 180.0, 160.0, 200.0, 80.0, 80.0, 80.0]
LEFT_ARM_KD = [5.0, 4.0, 3.0, 3.0, 2.0, 2.0, 2.0]
RIGHT_ARM_KD = [5.0, 4.0, 3.0, 3.0, 2.0, 2.0, 2.0]

# Joint names match the QJ2M_URDF_PATH teleop URDF (*_joint); ROS msgs are index-ordered
ARM_JOINT_NAMES = [
    "arml_1_joint", "arml_2_joint", "arml_3_joint", "arml_4_joint",
    "arml_5_joint", "arml_6_joint", "arml_7_joint",
    "armr_1_joint", "armr_2_joint", "armr_3_joint", "armr_4_joint",
    "armr_5_joint", "armr_6_joint", "armr_7_joint",
]


class QJ2MDualArmController:
    """QJ2M dual-arm ROS 2 interface."""

    def __init__(
        self,
        arm_command_topic: str = "/throw/arm_joint_command",
        hand_topic: str = "hand_control",
        arm_state_topic: str = "/motor/q2w_upper_motor_joint_state",
        rate_hz: float = 50,
        kp_left: list = None,
        kd_left: list = None,
        kp_right: list = None,
        kd_right: list = None,
        close_hand_at_start: bool = False,
        csp_command_topic: str = CSP_COMMAND_TOPIC,
        csp_max_joint_vel: float = CSP_MAX_JOINT_VEL,
    ):
        self.node = ros2.ensure_node("qj2m_dual_arm")
        self.arm_pub = self.node.create_publisher(JointState, arm_command_topic, 10)
        self.csp_pub = (
            self.node.create_publisher(Float64MultiArray, csp_command_topic, 10)
            if csp_command_topic
            else None
        )
        self.csp_max_joint_vel = float(csp_max_joint_vel)
        self._csp_last_q = None
        self._csp_last_time = 0.0
        self._motor_pos_all = None
        self.hand_pub = self.node.create_publisher(String, hand_topic, 1)
        self.state_sub = self.node.create_subscription(
            JointState, arm_state_topic, self._arm_state_cb, 10
        )
        self.rate_hz = float(rate_hz)
        self._rate_period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.01

        self.kp_left_vals = list(kp_left if kp_left is not None else LEFT_ARM_KP)
        self.kd_left_vals = list(kd_left if kd_left is not None else LEFT_ARM_KD)
        self.kp_right_vals = list(kp_right if kp_right is not None else RIGHT_ARM_KP)
        self.kd_right_vals = list(kd_right if kd_right is not None else RIGHT_ARM_KD)

        self.qpos_left = [0.0] * 7
        self.qpos_right = [0.0] * 7
        self.qvel_left = [0.0] * 7
        self.qvel_right = [0.0] * 7
        self.timestamp = 0.0
        self.last_update_time = 0.0  # wall-clock time of last motor state message

        self.q_des_left = None
        self.q_des_right = None
        self.v_des_left = [0.0] * 7
        self.v_des_right = [0.0] * 7

        self._hand_cmd = None
        self._first_state_logged = False

        if close_hand_at_start:
            ros2.sleep(0.3)
            self.publish_hand_command("close")

    def _arm_state_cb(self, msg: JointState):
        if len(msg.position) < 32:
            self.node.get_logger().warn(
                f"QJ2M state: got {len(msg.position)} pos, expected 32"
            )
            return
        self.last_update_time = time.time()
        self._motor_pos_all = [float(v) for v in msg.position[:CONTROL_MOTOR_SIZE]]
        self.qpos_left = [float(msg.position[i]) for i in LEFT_ARM_MOTOR_IDX]
        self.qpos_right = [float(msg.position[i]) for i in RIGHT_ARM_MOTOR_IDX]
        if len(msg.velocity) >= 32:
            self.qvel_left = [float(msg.velocity[i]) for i in LEFT_ARM_MOTOR_IDX]
            self.qvel_right = [float(msg.velocity[i]) for i in RIGHT_ARM_MOTOR_IDX]
        if self.q_des_left is None:
            self.q_des_left = self.qpos_left.copy()
        if self.q_des_right is None:
            self.q_des_right = self.qpos_right.copy()
        self.timestamp = ros2.stamp_to_sec(msg.header.stamp)
        if self.timestamp == 0:
            self.timestamp = time.time()
        if not self._first_state_logged:
            self.node.get_logger().info("[QJ2M] First state received")
            self._first_state_logged = True

    def publish_arm_command(self):
        if self.q_des_left is None:
            self.q_des_left = self.qpos_left.copy()
        if self.q_des_right is None:
            self.q_des_right = self.qpos_right.copy()
        ql = list(self.q_des_left)[:7]
        qr = list(self.q_des_right)[:7]
        vdl = list(self.v_des_left)[:7]
        vdr = list(self.v_des_right)[:7]

        msg = JointState()
        msg.header.stamp = ros2.now_msg(self.node)
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = ql + qr + self.kp_left_vals + self.kp_right_vals
        msg.velocity = vdl + vdr + self.kd_left_vals + self.kd_right_vals
        self.arm_pub.publish(msg)
        self._publish_csp_command(ql + qr)

    def _publish_csp_command(self, q_arms):
        if self.csp_pub is None or self._motor_pos_all is None:
            return
        target = np.asarray(q_arms, dtype=float)
        now = time.time()
        if self._csp_last_q is None:
            self._csp_last_q = np.asarray(self.qpos_left + self.qpos_right, dtype=float)
            self._csp_last_time = now
        dt = min(max(now - self._csp_last_time, 0.0), 4.0 * self._rate_period)
        max_step = self.csp_max_joint_vel * dt
        step = np.clip(target - self._csp_last_q, -max_step, max_step)
        self._csp_last_q = self._csp_last_q + step
        self._csp_last_time = now

        data = list(self._motor_pos_all)
        for k, idx in enumerate(LEFT_ARM_MOTOR_IDX + RIGHT_ARM_MOTOR_IDX):
            data[idx] = float(self._csp_last_q[k])
        msg = Float64MultiArray()
        msg.data = data
        self.csp_pub.publish(msg)

    def publish_hand_command(self, cmd: str = None):
        if cmd is not None:
            self._hand_cmd = cmd
        if self._hand_cmd is not None:
            msg = String()
            msg.data = self._hand_cmd
            self.hand_pub.publish(msg)
            self._hand_cmd = None

    def get_left_arm_state(self):
        return self.qpos_left

    def get_right_arm_state(self):
        return self.qpos_right

    def stop(self):
        try:
            self.node.destroy_publisher(self.arm_pub)
            self.node.destroy_publisher(self.hand_pub)
            if self.csp_pub is not None:
                self.node.destroy_publisher(self.csp_pub)
            self.node.destroy_subscription(self.state_sub)
        except Exception:
            pass
