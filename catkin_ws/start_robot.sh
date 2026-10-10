#!/bin/bash

# 1. 确保退出所有 Conda 环境 (如果在 Shell 里没配置好，用这个更稳)
# 如果你的 ROS 在 base 或外部，保留这行；如果 ROS 在某个 conda 环境，请改为 conda activate ros_env
eval "$(conda shell.bash hook)"
conda deactivate

# 2. 加载系统 ROS 和工作空间
source /opt/ros/noetic/setup.bash 2>/dev/null || echo "警告: 找不到系统 ROS"
source ~/catkin_ws/devel/setup.bash

# 3. 捕获 Ctrl+C 信号，确保关闭脚本时，后台的所有 roslaunch 也会被杀死
trap "echo '正在关闭所有节点...'; kill 0" SIGINT SIGTERM

echo "正在启动 Robotiq 夹爪..."
roslaunch robotiq_2f_gripper_control dual_robotiq_grippers.launch &

# 稍微多等一会儿，Realsense 初始化对 CPU 和 USB 带宽压力很大
sleep 5 

echo "正在启动 Realsense 相机..."
roslaunch realsense2_camera three_realsense.launch &

echo "------------------------------------------------"
echo "✅ 所有节点已启动！"
echo "💡 提示: 查看实时画面请新开终端运行 rviz"
echo "🛑 按下 [Ctrl+C] 停止并关闭所有进程"
echo "------------------------------------------------"

# 4. 保持脚本运行
wait