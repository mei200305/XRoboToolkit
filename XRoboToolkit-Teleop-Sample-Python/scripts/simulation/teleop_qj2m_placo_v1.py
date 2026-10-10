#!/usr/bin/env python3
"""
QJ2M (QJ Full Humanoid) Placo Simulation Teleoperation Script.

This script runs the QJ2M dual-arm teleoperation in Placo IK simulation.
It uses the QJ2M URDF with 7-DOF arms for IK solving and MeshCat visualization.

Prerequisites:
    1. XRoboToolkit PC Service must be running (with XR headset connected)
    2. xrobotoolkit_teleop package must be installed

Usage:
    # Basic simulation
    python scripts/simulation/teleop_qj2m_placo.py

    # Custom URDF (e.g., use the original QJ2M URDF with 4-DOF arms)
    python scripts/simulation/teleop_qj2m_placo.py \
        --robot-urdf-path assets/qj2m/urdf/qj2m.urdf

    # Adjust motion scaling
    python scripts/simulation/teleop_qj2m_placo.py --scale-factor 1.5

Controls:
    - Right Grip Button  → Activate right arm teleoperation
    - Left Grip Button   → Activate left arm teleoperation
    - Right Trigger      → Control right gripper
    - Left Trigger       → Control left gripper
"""

import os
import time

import tyro

from xrobotoolkit_teleop.simulation.placo_teleop_controller import (
    PlacoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import QJ2M_URDF_PATH

# Default QJ2M URDF for teleop simulation (new assets/qj2m model)
DEFAULT_QJ2M_URDF_PATH = QJ2M_URDF_PATH


def main(
    robot_urdf_path: str = DEFAULT_QJ2M_URDF_PATH,
    scale_factor: float = 1.5,
):
    """
    Main function to run the QJ2M robot teleoperation with Placo simulation.

    This script runs the QJ2M dual arm robot in simulation using Placo for IK
    solving and MeshCat visualization. Control both arms using XR controllers.

    Args:
        robot_urdf_path: Path to the robot URDF file.
        scale_factor: Scaling factor for motion (1.5 = 1.5x faster).
    """
    # Manipulator configuration for QJ2M
    config = {
        "right_arm": {
            "link_name": "armr_7",  # Right arm end effector link
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "right_trigger",
                "joint_names": ["r_gripper_finger_joint"],
                "open_pos": [0.0],
                "close_pos": [1.0],
            },
        },
        "left_arm": {
            "link_name": "arml_7",  # Left arm end effector link
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "gripper_config": {
                "type": "parallel",
                "gripper_trigger": "left_trigger",
                "joint_names": ["l_gripper_finger_joint"],
                "open_pos": [0.0],
                "close_pos": [1.0],
            },
        },
    }

    # Create and initialize the teleoperation controller
    controller = PlacoTeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        floating_base=False,
        scale_factor=scale_factor,
        dt=0.01,  # 100 Hz control loop
    )

    print("=" * 60)
    print("  QJ2M (起江双臂人形机器人) 遥操作 - Placo 仿真模式")
    print("=" * 60)
    print()
    print("控制说明 (Controls):")
    print("  - 右手控制器 (Right Controller) → 右臂 (armr_7)")
    print("  - 左手控制器 (Left Controller)  → 左臂 (arml_7)")
    print("  - 按下握持按钮 (Grip)  → 激活对应手臂控制")
    print("  - 按下扳机 (Trigger)    → 控制夹爪开合")
    print()
    print("调试信息 (Debug Info):")
    print(f"  - URDF: {robot_urdf_path}")
    print(f"  - 机器人关节总数 (Total Joints): {len(controller.placo_robot.state.q)}")
    print(f"  - 右臂末端 Link: {config['right_arm']['link_name']}")
    print(f"  - 左臂末端 Link: {config['left_arm']['link_name']}")
    print(f"  - Scale Factor: {scale_factor}")
    print()
    print("等待 XR 控制器连接...")
    print("如果机器人不动，请检查:")
    print("  1. XRoboToolkit PC Service 是否已启动")
    print("  2. XR 设备是否已连接 (PICO 头显)")
    print("  3. 是否按下了 Grip 按钮激活控制")
    print("  4. 控制器是否在移动")
    print("=" * 60)
    print()
    print("按 Ctrl+C 停止仿真")
    print()

    # Main control loop with debug output
    last_print_time = time.time()
    iteration = 0

    while not controller._stop_event.is_set():
        try:
            start_time = time.time()

            # Update IK
            controller._update_ik()

            # Send command (updates visualization)
            controller._send_command()

            # Debug output every 2 seconds
            current_time = time.time()
            if current_time - last_print_time > 2.0:
                iteration += 1
                print(f"\n[迭代 {iteration}] 状态检查 (Status Check):")
                print(f"  - 右臂激活 (Right Active): {controller.active.get('right_arm', False)}")
                print(f"  - 左臂激活 (Left Active):   {controller.active.get('left_arm', False)}")

                # Check controller poses
                try:
                    right_pose = controller.xr_client.get_controller_pose("right")
                    left_pose = controller.xr_client.get_controller_pose("left")
                    if right_pose:
                        print(
                            f"  - 右手控制器位置: "
                            f"[{right_pose[0]:.3f}, {right_pose[1]:.3f}, {right_pose[2]:.3f}]"
                        )
                    if left_pose:
                        print(
                            f"  - 左手控制器位置: "
                            f"[{left_pose[0]:.3f}, {left_pose[1]:.3f}, {left_pose[2]:.3f}]"
                        )
                except Exception:
                    print("  - 无法获取控制器位置 (Cannot get controller poses)")

                last_print_time = current_time

            end_time = time.time()
            elapsed = end_time - start_time
            sleep_time = max(0, controller.dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\n仿真已停止 (Simulation stopped)")
            controller._stop_event.set()
        except Exception as e:
            print(f"\n错误 (Error): {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)


if __name__ == "__main__":
    tyro.cli(main)
