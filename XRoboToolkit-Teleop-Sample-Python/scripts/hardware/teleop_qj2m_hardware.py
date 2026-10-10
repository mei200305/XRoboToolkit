#!/usr/bin/env python3
"""
QJ2M Real Hardware Teleoperation Launch Script.

Sends arm commands via /throw/arm_joint_command (JointState, 28D + kp/kd).
Hand control via hand_control topic (String "open"/"close").
Xense grippers: left trigger -> 192.168.99.30, right trigger -> 192.168.98.3.
Trigger released = open (85 mm), trigger pulled = closed (0 mm).

Prerequisites:
  1. ROS 2 daemon / nodes running (e.g. `ros2 daemon start`)
  2. QJ2M motor controller running (receives /throw/arm_joint_command)
  3. XRoboToolkit PC Service running (/opt/apps/roboticsservice/runService.sh)
  4. PICO headset connected with controllers active

Usage:
  python scripts/hardware/teleop_qj2m_hardware.py
  python scripts/hardware/teleop_qj2m_hardware.py --visualize-placo
  python scripts/hardware/teleop_qj2m_hardware.py --scale-factor 1.0 --control-rate-hz 100
  python scripts/hardware/teleop_qj2m_hardware.py --command-rate-hz 200 --joint-smooth-tau 0.06
"""
import tyro
from xrobotoolkit_teleop.hardware.qj2m_teleop_controller import (
    DEFAULT_MANIPULATOR_CONFIG,
    DEFAULT_QJ2M_URDF_PATH,
    QJ2MTeleopController,
)


def main(
    robot_urdf_path: str = DEFAULT_QJ2M_URDF_PATH,
    scale_factor: float = 1.0,
    enable_log_data: bool = True,
    visualize_placo: bool = False,
    control_rate_hz: int = 50,
    log_dir: str = "logs/qj2m",
    enable_camera: bool = False,
    enable_xense_gripper: bool = True,
    command_rate_hz: int = 200,
    joint_smooth_tau: float = 0.046,
):
    print("=" * 60)
    print("  QJ2M 遥操作 — 真机模式 (ROS 2)")
    print(f"  URDF:    {robot_urdf_path}")
    print("  Command: /throw/arm_joint_command (JointState)")
    print("           /motor/tt_upper_motor_csp_control_data (双臂 CSP 直驱, 限速 3 rad/s)")
    print("           勿同时运行 ~/Code/2m 的抓取程序（同一 CSP 话题会互相抢）")
    print("  Hand:    hand_control (String)")
    print("  夹爪:    左扳机 -> 192.168.99.30   右扳机 -> 192.168.98.3")
    print("           松开=张开 85 mm, 扣紧=闭合 0 mm")
    print("  采集:    B 键开始一个 rosbag，再按一次停止并保存")
    print("           /vla/{left,right}_arm/joint_state|joint_command  JointState(7)")
    print("           /xense/gripper/usb|nic/state|position_mm          Float64(mm)")
    print("           /xense/tactile/usb|nic/<serial>/image             视触觉原图")
    print("           /camera/d435i/color/image_raw                     D435i 彩色 640x480")
    print("           /camera/d435i/aligned_depth_to_color/image_raw   D435i 深度 640x480 mm")
    print("           /force/left_arm   /wrench_filtered_left             左臂六维力")
    print("           /force/right_arm  /wrench_filtered_right  /force    右臂六维力")
    print("  右摇杆上: 切换左臂跟随 (右手柄仅控X/Z轴, Y与姿态锁定)")
    print("=" * 60)

    controller = QJ2MTeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=DEFAULT_MANIPULATOR_CONFIG,
        scale_factor=scale_factor,
        enable_log_data=enable_log_data,
        visualize_placo=visualize_placo,
        control_rate_hz=control_rate_hz,
        log_dir=log_dir,
        enable_camera=enable_camera,
        enable_xense_gripper=enable_xense_gripper,
        command_rate_hz=command_rate_hz,
        joint_smooth_tau=joint_smooth_tau,
    )
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
