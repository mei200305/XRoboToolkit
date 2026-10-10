# Robotiq 夹爪控制项目

这是一个用于控制 Robotiq 夹爪的 ROS (Robot Operating System) 工作空间，支持多种通信协议和夹爪型号。

## 项目简介

本项目包含以下主要功能包：

- **robotiq_2f_gripper_control**: 2指夹爪控制（支持 85mm、140mm、C2 型号）
- **robotiq_3f_gripper_control**: 3指夹爪控制
- **robotiq_2f_gripper_action_server**: 2指夹爪动作服务器（提供高级控制接口）
- **robotiq_ethercat**: EtherCAT 通信接口
- **robotiq_modbus_tcp**: Modbus TCP 通信接口
- **robotiq_modbus_rtu**: Modbus RTU 通信接口
- **robotiq_*_gripper_visualization**: 夹爪可视化包

## 系统要求

- ROS (推荐 Melodic 或 Noetic)
- Ubuntu 18.04/20.04 (或其他兼容的 Linux 发行版)
- Python 2.7 或 Python 3 (根据 ROS 版本)
- 已安装 catkin 构建工具

## 编译安装

### 1. 安装依赖

```bash
# 安装 ROS 基础依赖
sudo apt-get update
sudo apt-get install ros-<your-ros-distro>-catkin python-catkin-tools

# 安装 Python Modbus 库（用于 Modbus 通信）
sudo apt-get install python3-serial
sudo apt-get install python3-pymodbus
```

### 2. 编译工作空间

```bash
cd ~/catkin_ws
catkin_make
# 或使用 catkin build
# catkin build
```

### 3. 配置环境

```bash
source devel/setup.bash
```

建议将以下命令添加到 `~/.bashrc` 文件中：

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## 启动方式

根据您的夹爪型号和通信方式，选择相应的启动方法。

### 方式一：EtherCAT 通信（推荐用于工业机器人）

EtherCAT 通常用于工业机器人系统，需要专用的以太网卡。

#### 2指夹爪 (2F Gripper)

```bash
roslaunch robotiq_2f_gripper_control robotiq_2f_gripper_ethercat.launch \
    gripper_name:=gripper \
    slave_number:=1 \
    ifname:=eth1 \
    activate:=True
```

**参数说明：**
- `gripper_name`: 夹爪名称（默认: gripper）
- `slave_number`: EtherCAT 从站编号（默认: 1）
- `ifname`: 网络接口名称（默认: eth1，根据实际情况修改）
- `activate`: 是否激活夹爪（默认: True）

**重要提示：**
- 需要使用专用的以太网卡（不要使用 eth0）
- 首次运行前需要设置权限：
  ```bash
  sudo setcap cap_net_raw+ep ~/catkin_ws/devel/lib/robotiq_2f_gripper_control/robotiq_2f_gripper_ethercat_node
  ```

#### 3指夹爪 (3F Gripper)

```bash
roslaunch robotiq_3f_gripper_control robotiq_3f_gripper_ethercat.launch \
    gripper_name:=gripper \
    slave_number:=1 \
    ifname:=enp0s20u1 \
    activate:=True
```

### 方式二：Modbus TCP 通信（网络连接）

适用于通过以太网连接的夹爪。

#### 2指夹爪

```bash
# 启动 Modbus TCP 节点（替换 <IP_ADDRESS> 为夹爪的实际 IP 地址）
rosrun robotiq_2f_gripper_control Robotiq2FGripperTcpNode.py <IP_ADDRESS>
```

例如：
```bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperTcpNode.py 192.168.1.11
```

#### 3指夹爪

```bash
rosrun robotiq_3f_gripper_control Robotiq3FGripperTcpNode.py <IP_ADDRESS>
```

### 方式三：Modbus RTU 通信（串口连接）

适用于通过串口（USB转串口）连接的夹爪。

#### 2指夹爪

```bash
# 启动 Modbus RTU 节点（替换 <DEVICE> 为串口设备，如 /dev/ttyUSB0）
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py <DEVICE>
```

例如：
```bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
```

**注意：** 确保您的用户有权限访问串口设备：
```bash
sudo usermod -a -G dialout $USER
# 然后重新登录
```

### 方式四：使用 Action Server（高级控制接口）

Action Server 提供了更高级的控制接口，支持位置和力控制。

#### 启动 2指夹爪 Action Server

```bash
# 首先启动夹爪控制节点（选择上述任一方式）
# 然后启动 Action Server
roslaunch robotiq_2f_gripper_action_server robotiq_2f_gripper_action_server.launch \
    gripper_name:=gripper \
    min_gap:=-0.017 \
    max_gap:=0.085 \
    min_effort:=30.0 \
    max_effort:=100.0
```

## 控制夹爪

### 方法一：使用简单控制器（命令行）

启动简单控制器节点：

```bash
# 2指夹爪
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

# 3指夹爪
rosrun robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py
```

**控制命令：**
- `r`: 重置夹爪
- `a`: 激活夹爪
- `c`: 闭合夹爪
- `o`: 打开夹爪
- `0-255`: 设置位置（0=打开，255=闭合）
- `f`: 增加速度
- `l`: 降低速度
- `i`: 增加力度
- `d`: 降低力度

### 方法二：使用 ROS Topic 发布命令

#### 2指夹爪

```bash
# 查看当前状态
rostopic echo /Robotiq2FGripperRobotInput

# 发布命令（激活并打开）
rostopic pub /Robotiq2FGripperRobotOutput robotiq_2f_gripper_control/Robotiq2FGripper_robot_output \
  "rACT: 1
   rGTO: 1
   rATR: 0
   rPR: 0
   rSP: 255
   rFR: 150" -1

# 闭合夹爪（rPR: 255）
rostopic pub /Robotiq2FGripperRobotOutput robotiq_2f_gripper_control/Robotiq2FGripper_robot_output \
  "rACT: 1
   rGTO: 1
   rATR: 0
   rPR: 255
   rSP: 255
   rFR: 150" -1
```

#### 3指夹爪

```bash
# 查看当前状态
rostopic echo /Robotiq3FGripperRobotInput

# 发布命令
rostopic pub /Robotiq3FGripperRobotOutput robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput \
  "rACT: 1
   rGTO: 1
   rATR: 0
   rICF: 1
   rICS: 1
   rPRA: 0
   rSPA: 255
   rFRA: 150" -1
```

### 方法三：使用 Action Server（推荐）

如果启动了 Action Server，可以使用 action 接口：

```bash
# 查看 action 服务
rostopic list | grep gripper

# 使用 action 客户端（需要编写简单的客户端脚本）
# 或使用 MoveIt 等高级规划框架
```

## 可视化

### 查看夹爪模型

```bash
# 2指 140mm 夹爪
roslaunch robotiq_2f_140_gripper_visualization test_2f_140_model.launch

# 2指 85mm 夹爪
roslaunch robotiq_2f_85_gripper_visualization test_2f_85_model.launch

# 2指 C2 夹爪
roslaunch robotiq_2f_c2_gripper_visualization test_robotiq_c2_model.launch

# 3指夹爪
roslaunch robotiq_3f_gripper_visualization robotiq_gripper_upload.launch
```

## 常见问题

### 1. EtherCAT 连接问题

**问题：** 出现 "Could not initialize SOEM" 错误

**解决方案：**
```bash
sudo setcap cap_net_raw+ep ~/catkin_ws/devel/lib/robotiq_2f_gripper_control/robotiq_2f_gripper_ethercat_node
```

**问题：** 虚拟机无法使用 EtherCAT

**解决方案：** EtherCAT 需要真实的硬件支持，虚拟机通常无法正常工作。请使用物理机器。

### 2. 串口权限问题

**问题：** 无法打开串口设备 `/dev/ttyUSB0`

**解决方案：**
```bash
sudo usermod -a -G dialout $USER
# 重新登录后生效
# 或临时使用：
sudo chmod 666 /dev/ttyUSB0
```

### 3. 网络接口名称

**问题：** 不知道使用哪个网络接口

**解决方案：**
```bash
# 查看所有网络接口
ip addr show

# 或使用
ifconfig
```

### 4. 夹爪无法激活

**检查清单：**
1. 确认夹爪已正确连接（电源、通信线）
2. 检查通信参数（IP地址、串口、从站编号等）
3. 查看节点输出信息：`rosnode info <node_name>`
4. 检查话题是否正常：`rostopic list` 和 `rostopic echo <topic_name>`

## 项目结构

```
catkin_ws/
├── src/
│   ├── robotiq/                    # Robotiq 夹爪相关包
│   │   ├── robotiq_2f_gripper_control/      # 2指夹爪控制
│   │   ├── robotiq_3f_gripper_control/      # 3指夹爪控制
│   │   ├── robotiq_2f_gripper_action_server/ # 2指夹爪动作服务器
│   │   ├── robotiq_ethercat/                # EtherCAT 接口
│   │   ├── robotiq_modbus_tcp/              # Modbus TCP 接口
│   │   └── robotiq_modbus_rtu/              # Modbus RTU 接口
│   └── realsense-ros-ros1-legacy/  # RealSense 相机相关包
├── build/                           # 编译中间文件
├── devel/                           # 开发环境文件
└── logs/                            # 编译日志
```

## 更多资源

- [Robotiq 官方支持网站](https://support.robotiq.com)
- [ROS Industrial Robotiq Wiki](http://wiki.ros.org/robotiq)
- [项目维护者信息](https://github.com/TAMS-Group)

## 许可证

本项目采用 Apache 2.0 和 BSD 3-Clause 双重许可证。

## 贡献

本项目由 TAMS Group (University of Hamburg) 维护。欢迎提交 Issue 和 Pull Request。

---

**注意：** 使用前请确保您已正确连接夹爪硬件，并根据实际情况调整启动参数。





