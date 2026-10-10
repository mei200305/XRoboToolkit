#!/bin/bash
# 启动脚本：确保在ros_env环境中运行遥操作程序

# 激活conda环境
source /root/miniforge3/etc/profile.d/conda.sh
conda activate ros_env
cd /home/XRoboToolkit-Teleop-Sample-Python
export PYTHONPATH=$PYTHONPATH:$(pwd)

# 检查 rclpy (ROS 2) 是否可用
python -c "import rclpy; print('✓ rclpy available')" || {
    echo "✗ Error: rclpy not available (source ROS 2 Humble first)"
    exit 1
}

# 运行传入的命令
exec "$@"


cd /opt/apps/roboticsservice  
bash runService.sh


# 启动起江双臂
source ../robotiq_and_realsense/devel/setup.bash 
python scripts/hardware/teleop_qj2_hardware.py 

# 启动piper单臂仿真
python scripts/simulation/teleop_piper_placo.py 

## 启动piper单臂真机
# 1. 激活can 0
cd /home/piper_ros
bash can_activate.sh can0 1000000
# 2. 启动roscore
roscore
# 3. piper_ros 的控制节点单独启动
source devel/setup.bash
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true

# 3.启动控制程序
python scripts/hardware/teleop_piper_hardware.py --backend sdk

## 改进后的piper单臂控制
python scripts/hardware/teleop_piper_hardware.py \
  --backend sdk \
  --can-name can0 \
  --scale-factor 0.9 \
  --command-filter-alpha 0.45 \
  --max-joint-velocity-rad-s 2.0 \
  --piper-joint-speed 60


# ROS 话题接口启动piper单臂真机 
python scripts/hardware/teleop_piper_hardware.py --backend ros

## 终端1 
source /root/miniforge3/etc/profile.d/conda.sh
conda activate ros_env
# source /home/piper_ros/devel/setup.bash
roscore

## 终端2
# source /root/miniforge3/etc/profile.d/conda.sh
# conda activate ros_env
# source /home/piper_ros/devel/setup.bash
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true

## 终端3
python scripts/hardware/teleop_piper_hardware.py --backend ros


## ---录制数据
python scripts/hardware/teleop_piper_hardware.py --backend ros --enable-camera  

# 启动双piper仿真
python scripts/simulation/teleop_piper_dual_placo.py --robot-urdf-path /home/piper_ros/src/piper_description/urdf/piper_description.urdf

# SDK 直连启动双piper真机并录制关节角和相机
python scripts/hardware/teleop_piper_dual_hardware.py --backend sdk --enable-camera

# ROS 话题接口启动双piper真机并录制关节角和相机
python scripts/hardware/teleop_piper_dual_hardware.py --backend ros --enable-camera

# 如果录制失败，则需要在宿主机上xhost +


### 1.1另外一台机器人IP 
```bash
ssh root@192.168.4.242
密码是： root
cd /opt/demo2
./demo 8
```


### 现在运行的是demo2下的 ./demo 8

### 1.2组播通信 连接主站和ROS的桥梁：
```bash	
## 关闭防火墙
sudo route add -net 224.0.1.100 netmask 255.255.255.255 dev enp4s0

# 进入docker:
docker exec -it VR /bin/bash

# 启动通信
cd /home/robot_arm_v2
source ./devel/setup.bash
rosrun upper_computer node_q2c_robot_control
```
