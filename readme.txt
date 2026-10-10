# Simulation:
  python3 scripts/simulation/teleop_qj2m_placo.py

  # Hardware:
  python3 scripts/hardware/teleop_qj2m_hardware.py

  # Hardware with IK visualization:
  python3 scripts/hardware/teleop_qj2m_hardware.py --visualize-placo

## QJ2M 专用文件说明

### 1. URDF 文件

#### `assets/qj2m/urdf/qj2m_teleop.urdf`
遥操作专用 URDF，基于原始 `qj2m.urdf` 修改：
- 6 个腕部关节从 `fixed` 改为 `revolute`
- 保持原始关节名称、限位、力矩参数不变
- 用于 Placo IK 求解

### 2. 硬件接口 (`xrobotoolkit_teleop/hardware/interface/qj2m.py`)

`QJ2MDualArmController` 类封装了与 QJ2M 机器人控制器的 ROS 通信：
- **关节控制**: `/motor_command/arm_position_control` (Float64MultiArray, 28维)
- **关节状态**: `/motor_state/arm_motor_state_actual` (JointState, 14位置)
- **左手夹爪**: `Robotiq2FGripperRobotOutput`
- **右手夹爪**: `Robotiq2FGripperRobotOutputRight`

### 3. 遥操作控制器 (`xrobotoolkit_teleop/hardware/qj2m_teleop_controller.py`)

`QJ2MTeleopController` 继承自 `HardwareTeleopController`，关键实现：

```python
# 左臂关节名 (Placo IK 中)
QJ2M_LEFT_ARM_JOINTS = [
    "19_left_shoulder_pitch",
    "20_left_shoulder_roll", 
    "21_left_shoulder_yaw",
    "22_left_elbow_pitch",
    "23_left_wrist_yaw",
    "24_left_wrist_pitch",
    "25_left_wrist_roll",
]

# 右臂关节名
QJ2M_RIGHT_ARM_JOINTS = [
    "26_right_shoulder_pitch",
    "27_right_shoulder_roll",
    "28_right_shoulder_yaw",
    "29_right_elbow_pitch",
    "30_right_wrist_yaw",
    "31_right_wrist_pitch",
    "32_right_wrist_roll",
]

# 末端执行器
DEFAULT_MANIPULATOR_CONFIG = {
    "right_arm": {"link_name": "armr_7", ...},
    "left_arm":  {"link_name": "arml_7", ...},
}
```

### 4. 启动脚本

#### 硬件脚本 (`scripts/hardware/teleop_qj2m_hardware.py`)
```bash
# 基本运行
python scripts/hardware/teleop_qj2m_hardware.py

# 带相机
python scripts/hardware/teleop_qj2m_hardware.py --enable-camera

# 带 IK 可视化 (调试用)
python scripts/hardware/teleop_qj2m_hardware.py --visualize-placo

# 调整运动比例
python scripts/hardware/teleop_qj2m_hardware.py --scale-factor 1.5
```

#### 仿真脚本 (`scripts/simulation/teleop_qj2m_placo.py`)
```bash
# 基本仿真 (使用遥操 URDF)
python scripts/simulation/teleop_qj2m_placo.py

# 自定义 URDF
python scripts/simulation/teleop_qj2m_placo.py \
    --robot-urdf-path assets/qj2m/urdf/qj2m.urdf

# 调整运动比例
python scripts/simulation/teleop_qj2m_placo.py --scale-factor 1.5
```

---

## 仿真运行 (Placo)

### 前置条件
1. **XRoboToolkit PC Service** 必须已安装并运行
   - 下载: https://github.com/XR-Robotics/XRoboToolkit-PC-Service
   - 启动程序后，连接 PICO 头显
2. Conda 环境已激活 (或系统 Python 已配置好所有依赖)

### 运行步骤

```bash
# 1. 进入项目目录
cd /home/agx02/XRoboToolkit/XRoboToolkit-Teleop-Sample-Python

# 2. (如使用 conda) 激活环境
conda activate ros_env

# 3. 启动 XRoboToolkit PC Service (在另一个终端或后台)
#    Windows: 双击运行 XRoboToolkit-PC-Service.exe
#    确保 PICO 头显已连接并处于串流模式

# 4. 运行 QJ2M Placo 仿真
python scripts/simulation/teleop_qj2m_placo.py
```

### 仿真控制说明
| 操作 | 按钮 |
|------|------|
| 激活右臂控制 | 右手 Grip 按钮 (按住) |
| 激活左臂控制 | 左手 Grip 按钮 (按住) |
| 右手夹爪闭合 | 右手 Trigger (0.0 → 1.0) |
| 左手夹爪闭合 | 左手 Trigger (0.0 → 1.0) |
| 停止仿真 | Ctrl+C |

### MeshCat 可视化
仿真启动后会自动打开浏览器显示 MeshCat 可视化页面。
- 可以看到机器人模型的实时姿态
- 末端执行器位置以坐标轴显示
- 灰色坐标轴 = 目标位姿，彩色坐标轴 = 当前位姿

---

## 真机运行 (Hardware)

### 前置条件
1. **QJ2M 机器人控制器** 已上电并运行
2. **ROS Master** 已启动 (`roscore`)
3. **QJ 机器人 ROS 节点** 正在发布/订阅话题
4. **Robotiq 夹爪驱动** 正在运行 (`rosrun robotiq_2f_gripper_control ...`)
5. **XRoboToolkit PC Service** 已运行且 PICO 头显已连接

### 完整启动流程

#### 步骤 1: 启动 ROS 环境

```bash
# 终端 1: 启动 roscore
source /opt/ros/noetic/setup.bash
roscore
```

#### 步骤 2: 启动 QJ 机器人通信

```bash
# 终端 2: 启动 QJ 机器人控制器
# 确保机器人与主控之间的组播通信正常
# 参考 run_teleop.sh 中的说明
```

#### 步骤 3: 启动 Robotiq 夹爪驱动

```bash
# 终端 3: 启动夹爪 ROS 节点
source /path/to/catkin_ws/devel/setup.bash
roslaunch robotiq_2f_gripper_control robotiq_action_server.launch
```

#### 步骤 4: 启动遥操作

```bash
# 终端 4: 激活环境并启动遥操作
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_env
cd /home/agx02/XRoboToolkit/XRoboToolkit-Teleop-Sample-Python

# 基本模式
python scripts/hardware/teleop_qj2m_hardware.py

# 带 IK 可视化 (用于调试)
python scripts/hardware/teleop_qj2m_hardware.py --visualize-placo

# 带相机和日志
python scripts/hardware/teleop_qj2m_hardware.py \
    --enable-camera \
    --enable-log-data \
    --log-dir logs/qj2m
```

### 真机控制说明
| 操作 | 按钮 |
|------|------|
| 激活右臂控制 | 右手 Grip 按钮 (按住) |
| 激活左臂控制 | 左手 Grip 按钮 (按住) |
| 右手夹爪闭合 | 右手 Trigger |
| 左手夹爪闭合 | 左手 Trigger |
| 开始/停止记录数据 | B 按钮 |
| 丢弃当前记录 | 右摇杆按下 |
| 紧急停止 | Ctrl+C |

### 安全注意事项
1. **首次运行前**：确保机器人周围有足够的运动空间
2. **激活机制**：手臂只在按住 Grip 按钮时才被激活，松开即停止
3. **死区设置**：默认激活阈值为 0.3，防止误触发
4. **紧急停止**：按 Ctrl+C 可立即停止所有控制线程
5. **关节限位**：Placo IK 求解器会自动遵守 URDF 中定义的关节限位
6. **奇异性避免**：控制器内置 manipulability 任务，避免奇异位姿

---

## 关节映射参考

### Placo IK 关节索引

QJ2M 的完整 URDF 包含以下关节（按 URDF 中的出现顺序）：

| Placo 偏移 | 关节名 | 类型 | 所属 |
|-----------|--------|------|------|
| 0-6 | _floating base_ | (masked) | 基座 |
| 7 | `01_left_hip_roll` | revolute | 左腿 |
| 8 | `02_left_hip_yaw` | revolute | 左腿 |
| 9 | `03_left_hip_pitch` | revolute | 左腿 |
| 10 | `04_left_knee_pitch` | revolute | 左腿 |
| 11 | `05_left_ankle_pitch` | revolute | 左腿 |
| 12 | `06_left_ankle_roll` | revolute | 左腿 |
| 13 | `07_right_hip_roll` | revolute | 右腿 |
| 14 | `08_right_hip_yaw` | revolute | 右腿 |
| 15 | `09_right_hip_pitch` | revolute | 右腿 |
| 16 | `10_right_knee_pitch` | revolute | 右腿 |
| 17 | `11_right_ankle_pitch` | revolute | 右腿 |
| 18 | `12_right_ankle_roll` | revolute | 右腿 |
| 19 | `13_waist_yaw` | revolute | 腰部 |
| 20 | `14_waist_pitch` | revolute | 腰部 |
| 21 | `15_waist_roll` | revolute | 腰部 |
| 22 | `16_neck_yaw` | fixed | 颈部 |
| 23 | `17_neck_pitch` | fixed | 颈部 |
| 24 | `18_neck_roll` | fixed | 颈部 |
| **25** | **`19_left_shoulder_pitch`** | revolute | **左臂 1** |
| **26** | **`20_left_shoulder_roll`** | revolute | **左臂 2** |
| **27** | **`21_left_shoulder_yaw`** | revolute | **左臂 3** |
| **28** | **`22_left_elbow_pitch`** | revolute | **左臂 4** |
| **29** | **`23_left_wrist_yaw`** | revolute | **左臂 5** |
| **30** | **`24_left_wrist_pitch`** | revolute | **左臂 6** |
| **31** | **`25_left_wrist_roll`** | revolute | **左臂 7** |
| **32** | **`26_right_shoulder_pitch`** | revolute | **右臂 1** |
| **33** | **`27_right_shoulder_roll`** | revolute | **右臂 2** |
| **34** | **`28_right_shoulder_yaw`** | revolute | **右臂 3** |
| **35** | **`29_right_elbow_pitch`** | revolute | **右臂 4** |
| **36** | **`30_right_wrist_yaw`** | revolute | **右臂 5** |
| **37** | **`31_right_wrist_pitch`** | revolute | **右臂 6** |
| **38** | **`32_right_wrist_roll`** | revolute | **右臂 7** |

> **注意**: 以上索引基于 `qj2m_teleop.urdf`（所有腕关节为 revolute）。
> 如果使用原始 `qj2m.urdf`，fixed 关节不会出现在状态向量中，
> 因此索引会不同（每臂只有 4 个 DOF）。

### ROS 话题关节映射

ROS 话题使用与 QJ2 相同的 28 维格式：

```
motor_command/arm_position_control (Float64MultiArray, 28 dims):
  [0:7]   = 左臂 7 关节位置 (rad)
  [7:14]  = 右臂 7 关节位置 (rad)
  [14:21] = 左臂 7 关节速度 (rad/s)
  [21:28] = 右臂 7 关节速度 (rad/s)

motor_state/arm_motor_state_actual (JointState, 14 positions):
  position[0:7]  = 左臂 7 关节位置 (rad)
  position[7:14] = 右臂 7 关节位置 (rad)
```

### 坐标系说明

- **XR 坐标系 → 机器人坐标系**: 通过 `R_HEADSET_TO_WORLD` 旋转矩阵转换
- **Placo IK 求解**: 在机器人世界坐标系中进行
- **基座为 fixed**: `floating_base=False`，基座固定在世界原点
- **世界坐标系**: X 向前，Y 向左，Z 向上 (ROS 标准)

---

## 数据采集

### 自动日志记录

硬件遥操作运行时，按 B 按钮即可开始/停止 rosbag 记录。

记录的数据包括:
- `/joint_states_single` 等关节状态话题
- 相机图像话题 (如果启用)
- 所有话题都带有同步的时间戳

### 日志文件位置

```
logs/qj2m/
├── teleop_20260624_153000_1.bag      # rosbag 文件
├── teleop_20260624_153000_1.pkl      # Python pickle (如果使用旧格式)
└── ...
```

### 数据验证

```bash
# 查看 rosbag 信息
rosbag info logs/qj2m/teleop_20260624_153000_1.bag

# 回放 rosbag
rosbag play logs/qj2m/teleop_20260624_153000_1.bag
```

### 转换为 LeRobot 格式

参考 ARX Dual Arm Data Converter:
https://github.com/zhigenzhao/openpi/blob/dev/finetuning/examples/arx_r5/arx_dual/convert_dual_arm_data_to_lerobot.py

---

## 故障排除

### 1. "Waiting for initial joint states..."

**原因**: ROS 话题 `/motor_state/arm_motor_state_actual` 没有数据发布。

**解决**:
- 确认机器人控制器正在运行
- 检查话题是否在发布: `rostopic echo /motor_state/arm_motor_state_actual`
- 确认话题名称与代码中一致
- 检查网络连接 (机器人控制器 IP)

### 2. Placo IK 求解失败

**原因**: URDF 与硬件关节配置不匹配。

**解决**:
- 确认使用了正确的 URDF (`qj2m_teleop.urdf`)
- 检查 URDF 中的关节顺序是否与 ROS 话题中的顺序一致
- 查看终端输出的 Placo 关节名列表，确认关节偏移正确
- 如果关节方向相反，在 `_send_command()` 中添加符号反转

### 3. "xrobotoolkit_sdk not found"

**原因**: XRoboToolkit Python 绑定未安装或构建失败。

**解决**:
- Docker 用户: 确认在 pico:v5 容器内运行
- Conda 用户: 重新运行 `bash setup_conda.sh --install`
- 手动安装: 进入 `dependencies/XRoboToolkit-PC-Service-Pybind/` 执行 `python setup.py install`

### 4. 机器人运动方向与实际控制器方向不一致

**原因**: 坐标系变换或关节映射有误。

**解决**:
- 调整 `R_headset_world` 旋转矩阵
- 检查 `scale_factor` 参数 (正值 = 正向跟随，负值 = 反向)
- 在硬件接口层对个别关节做符号映射

### 5. 机器人抖动

**原因**: 控制频率不匹配或 IK 求解不稳定。

**解决**:
- 降低控制频率: `--control-rate-hz 50`
- 调整运动比例: `--scale-factor 0.5`
- 检查 URDF 关节限位是否合理
- 确保 ROS 话题以足够频率发布

### 6. "ModuleNotFoundError: No module named 'rospy'"

**原因**: Python 环境未包含 ROS 包。

**解决**:
- 确保 ROS Noetic 已 source: `source /opt/ros/noetic/setup.bash`
- 如果使用 conda，可能需要将 ROS 路径加入 PYTHONPATH
- 或者使用系统 Python (非 conda 环境) 运行

### 7. 夹爪不响应

**原因**: Robotiq 夹爪驱动未运行或话题名称不匹配。

**解决**:
- 确认夹爪驱动 ROS 节点已启动
- 检查话题: `rostopic list | grep Robotiq`
- 如果消息类型不同，可能需要修改 `HAS_ROBOTIQ_MSG` 检查逻辑
- 查看终端中是否有 "Robotiq message not available" 警告

---

## 参考文件

- QJ2 参考实现: `xrobotoolkit_teleop/hardware/qj2_teleop_controller.py`
- QJ2 硬件接口: `xrobotoolkit_teleop/hardware/interface/qj2.py`
- QJ 机器人 ROS 接口说明: `QJ机器人ROS接口说明.md`
- 遥操作移植指南: `README_porting_robot.md`
- 项目 README: `README.md`
- 遥操作详细说明: `teleop_details.md`

