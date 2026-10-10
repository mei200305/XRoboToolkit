import os
import time

import numpy as np
import tyro

from xrobotoolkit_teleop.simulation.placo_teleop_controller import (
    PlacoTeleopController,
)
from xrobotoolkit_teleop.utils.path_utils import ASSET_PATH


class DebugPlacoTeleopController(PlacoTeleopController):
    """带调试信息的 Placo 遥操作控制器"""
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ik_solve_count = 0
        self.ik_fail_count = 0
        self.last_q = None
        
    def _update_ik(self):
        """重写 IK 更新方法，添加调试信息"""
        try:
            # 保存更新前的关节状态
            self.last_q = self.placo_robot.state.q.copy()
            
            # 调用父类方法
            super()._update_ik()
            
            self.ik_solve_count += 1
            
            # 检查关节是否变化
            if self.last_q is not None:
                q_diff = np.abs(self.placo_robot.state.q - self.last_q)
                max_diff = np.max(q_diff)
                if max_diff > 1e-6:
                    print(f"[IK #{self.ik_solve_count}] 关节已更新，最大变化: {max_diff:.6f}")
        except Exception as e:
            self.ik_fail_count += 1
            print(f"[IK 错误 #{self.ik_fail_count}]: {e}")
            import traceback
            traceback.print_exc()


def main(
    robot_urdf_path: str = os.path.join(ASSET_PATH, "qj2/urdf/arms_gen2_hand.urdf"),
    scale_factor: float = 1.00,
):
    """
    Main function to run the QJ2 robot teleoperation with Placo simulation (调试版本).
    """
    config = {
        "right_arm": {
            "link_name": "r_hand_base_Link",
            "pose_source": "right_controller",
            "control_trigger": "right_grip",
            "gripper_config": {
                "type": "robotiq",
                "gripper_trigger": "right_trigger",
                "joint_names": [],
                "open_pos": [0.0],
                "close_pos": [1.0],
            },
        },
        "left_arm": {
            "link_name": "l_hand_base_Link",
            "pose_source": "left_controller",
            "control_trigger": "left_grip",
            "gripper_config": {
                "type": "robotiq",
                "gripper_trigger": "left_trigger",
                "joint_names": [],
                "open_pos": [0.0],
                "close_pos": [1.0],
            },
        },
    }

    print("=" * 70)
    print("QJ2 Placo 仿真 - 调试模式")
    print("=" * 70)
    
    # 创建控制器
    controller = DebugPlacoTeleopController(
        robot_urdf_path=robot_urdf_path,
        manipulator_config=config,
        floating_base=False,
        scale_factor=scale_factor,
        dt=0.01,
    )

    # 打印初始信息
    print(f"\n机器人初始化完成:")
    print(f"  - 关节总数: {len(controller.placo_robot.state.q)}")
    print(f"  - 初始关节状态: {controller.placo_robot.state.q[:14] if len(controller.placo_robot.state.q) >= 14 else controller.placo_robot.state.q}")
    
    # 检查末端执行器 link
    try:
        right_ee_pos, _ = controller._get_link_pose("r_hand_base_Link")
        left_ee_pos, _ = controller._get_link_pose("l_hand_base_Link")
        print(f"  - 右臂末端位置: {right_ee_pos}")
        print(f"  - 左臂末端位置: {left_ee_pos}")
    except Exception as e:
        print(f"  - 警告: 无法获取末端位置: {e}")
    
    # 测试：手动移动一个关节看看可视化是否更新
    print("\n测试: 手动移动关节以验证可视化...")
    for i in range(5):
        # 移动右臂第一个关节
        if len(controller.placo_robot.state.q) > 7:
            controller.placo_robot.state.q[7] = 0.5 * np.sin(i * 0.5)
            controller.placo_robot.update_kinematics()
            controller._update_placo_viz()
            print(f"  测试 {i+1}/5: 右臂关节1 = {controller.placo_robot.state.q[7]:.3f}")
            time.sleep(0.5)
    
    # 重置到初始状态
    controller.placo_robot.state.q[7] = 0.0
    controller.placo_robot.update_kinematics()
    controller._update_placo_viz()
    print("  测试完成，已重置到初始状态\n")

    print("=" * 70)
    print("控制说明:")
    print("  - 右手控制器 → 控制右臂")
    print("  - 左手控制器 → 控制左臂")
    print("  - 按下握持按钮 (grip) → 激活对应手臂控制")
    print("  - 按下扳机 (trigger) → 控制夹爪开合")
    print("=" * 70)
    print("\n等待 XR 控制器连接...")
    print("如果机器人不动，请检查:")
    print("  1. XR 设备是否已连接并运行")
    print("  2. 是否按下了 grip 按钮激活控制")
    print("  3. 控制器是否在移动")
    print("  4. 查看下面的调试信息")
    print("=" * 70)
    print("\n按 Ctrl+C 停止仿真\n")
    
    # 主循环
    last_print_time = time.time()
    iteration = 0
    
    while not controller._stop_event.is_set():
        try:
            start_time = time.time()
            
            # 更新 IK
            controller._update_ik()
            
            # 发送命令（更新可视化）
            controller._send_command()
            
            # 每 2 秒打印调试信息
            current_time = time.time()
            if current_time - last_print_time > 2.0:
                iteration += 1
                print(f"\n[迭代 {iteration}] 状态:")
                print(f"  - IK 求解次数: {controller.ik_solve_count}")
                print(f"  - IK 失败次数: {controller.ik_fail_count}")
                print(f"  - 右臂激活: {controller.active.get('right_arm', False)}")
                print(f"  - 左臂激活: {controller.active.get('left_arm', False)}")
                
                # 检查控制器状态
                try:
                    right_grip = controller.xr_client.get_button_state_by_name("right_grip")
                    left_grip = controller.xr_client.get_button_state_by_name("left_grip")
                    print(f"  - 右手 grip 按钮: {right_grip}")
                    print(f"  - 左手 grip 按钮: {left_grip}")
                    
                    right_pose = controller.xr_client.get_controller_pose("right")
                    left_pose = controller.xr_client.get_controller_pose("left")
                    if right_pose:
                        print(f"  - 右手控制器位置: [{right_pose[0]:.3f}, {right_pose[1]:.3f}, {right_pose[2]:.3f}]")
                    else:
                        print(f"  - 右手控制器: 未连接")
                    if left_pose:
                        print(f"  - 左手控制器位置: [{left_pose[0]:.3f}, {left_pose[1]:.3f}, {left_pose[2]:.3f}]")
                    else:
                        print(f"  - 左手控制器: 未连接")
                except Exception as e:
                    print(f"  - 获取控制器状态失败: {e}")
                
                # 检查关节位置
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
