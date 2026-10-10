import os
import time

import tyro

from xrobotoolkit_teleop.simulation.placo_teleop_controller import (
    PlacoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


def main(
    robot_urdf_path: str = os.path.join(ASSET_PATH, "qj2/urdf/arms_gen2_hand.urdf"),
    scale_factor: float = 1.5,
):
    """
    Main function to run the QJ2 robot teleoperation with Placo simulation.
    
    This script runs the QJ2 dual arm robot in simulation using Placo for IK solving
    and visualization. You can control both arms using XR controllers.
    
    Args:
        robot_urdf_path: Path to the robot URDF file
        scale_factor: Scaling factor for motion (1.5 = 1.5x faster)
    """
    config = {
        "right_arm": {
            "link_name": "r_hand_base_Link",  # End effector link for right arm
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "gripper_config": {
                "type": "robotiq",  # Robotiq gripper type (for simulation, this is just visual)
                "gripper_trigger": "right_trigger",
                "joint_names": [],  # Robotiq doesn't use joint names in URDF
                "open_pos": [0.0],  # 0.0 = open
                "close_pos": [1.0],  # 1.0 = close
            },
        },
        "left_arm": {
            "link_name": "l_hand_base_Link",  # End effector link for left arm
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "gripper_config": {
                "type": "robotiq",  # Robotiq gripper type (for simulation, this is just visual)
                "gripper_trigger": "left_trigger",
                "joint_names": [],  # Robotiq doesn't use joint names in URDF
                "open_pos": [0.0],  # 0.0 = open
                "close_pos": [1.0],  # 1.0 = close
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
    print("QJ2 Placo simulation started!")
    print("=" * 60)
    print("\n控制说明:")
    print("  - 右手控制器 → 控制右臂 (r_hand_base_Link)")
    print("  - 左手控制器 → 控制左臂 (l_hand_base_Link)")
    print("  - 按下握持按钮 (grip) → 激活对应手臂控制")
    print("  - 按下扳机 (trigger) → 控制夹爪开合")
    print("\n调试信息:")
    print(f"  - 机器人关节数: {len(controller.placo_robot.state.q)}")
    print(f"  - 右臂末端link: {config['right_arm']['link_name']}")
    print(f"  - 左臂末端link: {config['left_arm']['link_name']}")
    print("\n等待 XR 控制器连接...")
    print("如果机器人不动，请检查:")
    print("  1. XR 设备是否已连接")
    print("  2. 是否按下了 grip 按钮激活控制")
    print("  3. 控制器是否在移动")
    print("=" * 60)
    print("\n按 Ctrl+C 停止仿真\n")
    
    # Add debug output
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
                print(f"\n[迭代 {iteration}] 状态检查:")
                print(f"  - 右臂激活: {controller.active.get('right_arm', False)}")
                print(f"  - 左臂激活: {controller.active.get('left_arm', False)}")
                
                # Check controller poses
                try:
                    right_pose = controller.xr_client.get_controller_pose("right")
                    left_pose = controller.xr_client.get_controller_pose("left")
                    if right_pose:
                        print(f"  - 右手控制器位置: [{right_pose[0]:.3f}, {right_pose[1]:.3f}, {right_pose[2]:.3f}]")
                    if left_pose:
                        print(f"  - 左手控制器位置: [{left_pose[0]:.3f}, {left_pose[1]:.3f}, {left_pose[2]:.3f}]")
                except:
                    print("  - 无法获取控制器位置")
                
                # Check joint positions
                if len(controller.placo_robot.state.q) > 7:
                    print(f"  - 右臂关节1-3: {controller.placo_robot.state.q[7:10]}")
                    if len(controller.placo_robot.state.q) > 14:
                        print(f"  - 左臂关节1-3: {controller.placo_robot.state.q[14:17]}")
                
                last_print_time = current_time
            
            end_time = time.time()
            elapsed = end_time - start_time
            sleep_time = max(0, controller.dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\n\n仿真已停止")
            controller._stop_event.set()
        except Exception as e:
            print(f"\n错误: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)


if __name__ == "__main__":
    tyro.cli(main)
