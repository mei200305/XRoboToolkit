#!/usr/bin/env python
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from xrobotoolkit_teleop.hardware.interface import ros2_compat as ros2

LEFT_JOINT_NAMES = [f"l_arm_Joint{i}" for i in range(1, 8)]
RIGHT_JOINT_NAMES = [f"r_arm_Joint{i}" for i in range(1, 8)]

pub = None
node = None

def cb(msg):
    # 期望长度 28
    if len(msg.data) < 28:
        node.get_logger().warn(f"cmd length {len(msg.data)} < 28")
        return

    # 拆分
    left_pos = list(msg.data[0:7])
    right_pos = list(msg.data[7:14])
    # 速度在 [14:21], [21:28]，这里可以不用

    js = JointState()
    js.header.stamp = ros2.now_msg(node)
    js.name = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
    js.position = left_pos + right_pos

    pub.publish(js)

def main():
    global pub, node
    node = ros2.ensure_node("qj2_cmd_to_joint_state")
    pub = node.create_publisher(JointState, "/qj2_cmd_joint_states", 1)
    node.create_subscription(
        Float64MultiArray, "/motor_command/arm_position_control", cb, 10
    )
    # Background spin is started by ensure_node; keep process alive
    while ros2.ok():
        ros2.sleep(0.1)

if __name__ == "__main__":
    main()
