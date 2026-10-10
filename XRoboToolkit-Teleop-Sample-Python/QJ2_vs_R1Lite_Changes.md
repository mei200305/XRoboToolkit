# QJ2 机器人遥操作实现 - 与 R1Lite 的差异说明

本文档详细说明了 QJ2 机器人遥操作代码相比 R1Lite 机器人所做的关键改动。

## 目录

1. [概述](#概述)
2. [硬件接口层改动](#硬件接口层改动)
3. [遥操作控制器改动](#遥操作控制器改动)
4. [配置参数改动](#配置参数改动)
5. [ROS 话题和消息格式](#ros-话题和消息格式)
6. [关键代码差异](#关键代码差异)

---

## 概述

### R1Lite 架构
- **双臂控制器**: 使用两个独立的 `A1XController` 实例（左臂、右臂分别控制）
- **ROS 消息**: 使用自定义 `hdas_msg/motor_control` 或标准 `sensor_msgs/JointState`
- **控制方式**: 每个手臂独立发布控制命令
- **关节数量**: 每臂 6 个关节

### QJ2 架构
- **双臂控制器**: 使用单个 `QJ2DualArmController` 统一控制双臂
- **ROS 消息**: 使用 `std_msgs/Float64MultiArray` (28维数组)
- **控制方式**: 双臂命令打包在一个消息中发送
- **关节数量**: 每臂 7 个关节

---

## 硬件接口层改动

### 1. 控制器类设计

#### R1Lite (`galaxea.py`)
```python
class A1XController:
    """单臂控制器，需要创建两个实例"""
    def __init__(
        self,
        arm_control_topic: str = "/motion_control/control_arm",
        gripper_control_topic: str = "/motion_control/control_gripper",
        arm_state_topic: str = "/hdas/feedback_arm",
        ...
    ):
        # 每个控制器独立发布/订阅
        self.pub = rospy.Publisher(arm_control_topic, motor_control, ...)
        self.sub = rospy.Subscriber(arm_state_topic, JointState, ...)
```

#### QJ2 (`qj2.py`)
```python
class QJ2DualArmController:
    """双臂统一控制器，一个实例控制双臂"""
    def __init__(
        self,
        arm_control_topic: str = "/motor_command/arm_position_control",
        arm_state_topic: str = "/motor_state/arm_motor_state_actual",
        ...
    ):
        # 单个发布者发送 28 维数组（左7+右7+左7速度+右7速度）
        self.pub = rospy.Publisher(arm_control_topic, Float64MultiArray, ...)
        self.sub = rospy.Subscriber(arm_state_topic, JointState, ...)
```

**关键差异**:
- R1Lite: 两个独立的控制器，每个控制一个手臂
- QJ2: 一个统一控制器，同时处理双臂数据

### 2. 消息格式

#### R1Lite
- **控制消息**: `hdas_msg/motor_control` 或 `sensor_msgs/JointState`
  - 包含: `kp`, `kd`, `t_ff`, `p_des`, `v_des` 等字段
  - 每个手臂单独发送一条消息

#### QJ2
- **控制消息**: `std_msgs/Float64MultiArray` (28维)
  - 格式: `[左7位置, 右7位置, 左7速度, 右7速度]`
  - 双臂数据打包在一个消息中

**代码示例**:
```python
# QJ2 的 publish_arm_control()
left_pos = list(self.q_des_left)[:7]   # 左臂 7 个关节位置
right_pos = list(self.q_des_right)[:7] # 右臂 7 个关节位置
left_vel = list(self.v_des_left)[:7]   # 左臂 7 个关节速度
right_vel = list(self.v_des_right)[:7] # 右臂 7 个关节速度

self.arm_ctrl_msg.data = left_pos + right_pos + left_vel + right_vel  # 28维
self.pub.publish(self.arm_ctrl_msg)
```

### 3. 状态回调处理

#### R1Lite
```python
def arm_state_callback(self, msg: JointState):
    self.qpos = list(msg.position[:6])  # 单臂 6 个关节
    self.qvel = list(msg.velocity[:6])
```

#### QJ2
```python
def arm_state_callback(self, msg: JointState):
    # 处理 14 个关节位置（左7+右7）
    self.qpos_left = list(msg.position[0:7])   # 左臂
    self.qpos_right = list(msg.position[7:14]) # 右臂
    self.qvel_left = list(msg.velocity[0:7]) if len(msg.velocity) >= 14 else [0.0] * 7
    self.qvel_right = list(msg.velocity[7:14]) if len(msg.velocity) >= 14 else [0.0] * 7
```

---

## 遥操作控制器改动

### 1. 控制器初始化

#### R1Lite (`galaxea_r1_lite_teleop_controller.py`)
```python
def _robot_setup(self):
    # 创建两个独立的 A1XController
    self.arm_controllers: Dict[str, A1XController] = {}
    for arm_name in ["left_arm", "right_arm"]:
        controller = A1XController(
            arm_control_topic=f"/motion_control/control_arm_{arm_prefix}",
            arm_state_topic=f"/hdas/feedback_arm_{arm_prefix}",
            ...
        )
        self.arm_controllers[arm_name] = controller
```

#### QJ2 (`qj2_teleop_controller.py`)
```python
def _robot_setup(self):
    # 创建单个 QJ2DualArmController
    self.arm_controller = QJ2DualArmController(
        arm_control_topic="/motor_command/arm_position_control",
        arm_state_topic="/motor_state/arm_motor_state_actual",
        ...
    )
```

### 2. 状态更新

#### R1Lite
```python
def _update_robot_state(self):
    for arm_name, controller in self.arm_controllers.items():
        self.placo_robot.state.q[self.placo_arm_joint_slice[arm_name]] = controller.qpos
```

#### QJ2
```python
def _update_robot_state(self):
    left_qpos = self.arm_controller.get_left_arm_state()
    right_qpos = self.arm_controller.get_right_arm_state()
    self.placo_robot.state.q[self.placo_arm_joint_slice["left_arm"]] = left_qpos
    self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]] = right_qpos
```

### 3. 命令发送

#### R1Lite
```python
def _send_command(self):
    for arm_name, controller in self.arm_controllers.items():
        if self.active.get(arm_name, False):
            controller.q_des = self.placo_robot.state.q[...].copy()
        controller.publish_arm_control()  # 每个手臂单独发布
        controller.publish_gripper_control()
```

#### QJ2
```python
def _send_command(self):
    if self.active.get("left_arm", False):
        self.arm_controller.q_des_left = self.placo_robot.state.q[...].copy()[:7]
    if self.active.get("right_arm", False):
        self.arm_controller.q_des_right = self.placo_robot.state.q[...].copy()[:7]
    
    self.arm_controller.publish_arm_control()  # 一次性发布双臂数据
    self.arm_controller.publish_gripper_control()
```

### 4. 关节映射

#### R1Lite
```python
def _placo_setup(self):
    # 每臂 6 个关节
    arm_joint_names = [f"{arm_prefix}arm_joint{i}" for i in range(1, 7)]
```

#### QJ2
```python
def _placo_setup(self):
    # 每臂 7 个关节
    arm_joint_names = [f"{arm_prefix}Joint{i}" for i in range(1, 8)]  # Joint1 to Joint7
```

---

## 配置参数改动

### 1. 末端执行器 Link 名称

#### R1Lite
```python
DEFAULT_MANIPULATOR_CONFIG = {
    "right_arm": {
        "link_name": "right_gripper_link",
        ...
    },
    "left_arm": {
        "link_name": "left_gripper_link",
        ...
    },
}
```

#### QJ2
```python
DEFAULT_MANIPULATOR_CONFIG = {
    "right_arm": {
        "link_name": "r_hand_base_Link",  # 使用 URDF 中的实际 link 名称
        ...
    },
    "left_arm": {
        "link_name": "l_hand_base_Link",
        ...
    },
}
```

### 2. 夹爪配置

#### R1Lite
```python
"gripper_config": {
    "type": "parallel",
    "joint_names": ["right_gripper_finger_joint1"],
    "open_pos": [-2.9],
    "close_pos": [0.0],
}
```

#### QJ2
```python
"gripper_config": {
    "type": "parallel",  # 注意：虽然使用 Robotiq，但配置中仍用 "parallel"
    "joint_names": [],   # Robotiq 不使用 URDF 关节名
    "open_pos": [0.0],   # 映射到 Robotiq 0-255 的 0
    "close_pos": [1.0],  # 映射到 Robotiq 0-255 的 255
}
```

**注意**: QJ2 使用 Robotiq 夹爪，但为了兼容基础遥操作框架，`gripper_config["type"]` 设置为 `"parallel"`，实际夹爪控制通过 `QJ2DualArmController.set_gripper_position()` 转换为 Robotiq 消息格式。

### 3. ROS 话题名称

#### R1Lite
- 控制话题: `/motion_control/control_arm_left`, `/motion_control/control_arm_right`
- 状态话题: `/hdas/feedback_arm_left`, `/hdas/feedback_arm_right`
- 夹爪话题: `/motion_control/control_gripper_left`, `/motion_control/control_gripper_right`

#### QJ2
- 控制话题: `/motor_command/arm_position_control` (统一话题，28维数组)
- 状态话题: `/motor_state/arm_motor_state_actual` (统一话题，14个位置)
- 夹爪话题: `Robotiq2FGripperRobotOutput`, `Robotiq2FGripperRobotOutputRight`

---

## ROS 话题和消息格式

### R1Lite 消息格式

**控制消息** (`hdas_msg/motor_control`):
```python
motor_control:
  header: Header
  kp: [float]      # 位置增益
  kd: [float]      # 速度增益
  t_ff: [float]    # 前馈力矩
  p_des: [float]   # 期望位置
  v_des: [float]   # 期望速度
```

**状态消息** (`sensor_msgs/JointState`):
- 每个手臂单独的话题
- `position`: 6 个关节位置
- `velocity`: 6 个关节速度

### QJ2 消息格式

**控制消息** (`std_msgs/Float64MultiArray`):
```python
Float64MultiArray:
  data: [28个float]
  # [0:7]   = 左臂 7 个关节位置
  # [7:14]  = 右臂 7 个关节位置
  # [14:21] = 左臂 7 个关节速度
  # [21:28] = 右臂 7 个关节速度
```

**状态消息** (`sensor_msgs/JointState`):
- 统一话题 `/motor_state/arm_motor_state_actual`
- `position`: 14 个关节位置 `[左7, 右7]`
- `effort`: 14 个关节电流 `[左7, 右7]`

**夹爪消息** (`robot_controller/Robotiq2FGripper_robot_output`):
```python
Robotiq2FGripper_robot_output:
  rACT: uint8  # 激活标志 (0=重置, 1=激活)
  rGTO: uint8  # 执行标志 (0=停止, 1=执行)
  rATR: uint8  # 自动释放 (0=禁用, 1=启用)
  rPR: uint8   # 位置命令 (0=张开, 255=闭合)
  rSP: uint8   # 速度 (0-255)
  rFR: uint8   # 力 (0-255)
```

---

## 关键代码差异

### 1. 数据拼接方式

#### R1Lite
```python
# 每个控制器独立处理，无需拼接
controller.q_des = qpos.copy()
controller.publish_arm_control()
```

#### QJ2
```python
# 需要将双臂数据拼接成 28 维数组
# 关键：必须转换为 Python list，不能直接用 numpy 数组
left_pos = list(self.q_des_left)[:7]
right_pos = list(self.q_des_right)[:7]
left_vel = list(self.v_des_left)[:7]
right_vel = list(self.v_des_right)[:7]

self.arm_ctrl_msg.data = left_pos + right_pos + left_vel + right_vel
```

**重要**: 如果使用 numpy 数组直接相加，会导致元素逐项相加而不是列表拼接，结果仍然是 7 个元素而不是 28 个。

### 2. 底盘和腰部控制

#### R1Lite
- 默认启用底盘和腰部控制
- 使用摇杆控制底盘移动
- 使用按钮控制腰部升降

#### QJ2
- 默认禁用底盘和腰部控制（代码中已注释）
- 如果 QJ2 机器人有这些组件，可以取消注释相关代码

### 3. 相机配置

#### R1Lite
```python
camera_topics = {
    "left": {"color": "/hdas/camera_wrist_left/color/image_raw/compressed"},
    "right": {"color": "/hdas/camera_wrist_right/color/image_raw/compressed"},
    ...
}
```

#### QJ2
```python
camera_topics = {
    "left": {"color": "/camera_left/color/image_raw/compressed"},
    "right": {"color": "/camera_right/color/image_raw/compressed"},
}
# 注意：需要根据实际机器人配置更新话题名称
```

---

## 总结

### 主要改动点

1. **架构变化**: 从"双控制器"改为"单统一控制器"
2. **消息格式**: 从"自定义消息/单臂消息"改为"28维数组统一消息"
3. **关节数量**: 从"每臂6关节"改为"每臂7关节"
4. **数据拼接**: 需要手动将双臂数据拼接成 28 维数组
5. **话题命名**: 使用 QJ 机器人标准的 ROS 话题命名
6. **夹爪控制**: 使用 Robotiq 消息格式，而非关节控制

### 兼容性说明

- QJ2 代码保留了 `R1LiteChassisController` 和 `R1LiteTorsoController` 类（在 `qj2.py` 中），以便未来扩展
- 夹爪配置中使用 `"type": "parallel"` 以兼容基础遥操作框架，但实际控制通过 Robotiq 消息实现

### 使用建议

1. **首次运行**: 建议先用 Placo 仿真验证 IK 和 VR 映射是否正确
2. **真机测试**: 先用小 `scale_factor` (0.5-1.0) 测试，确认无误后再调大
3. **数据验证**: 使用 `rostopic echo` 确认发布的 28 维数组格式正确
4. **RViz 可视化**: 可以使用 `qj2_cmd_to_joint_state.py` 将命令转换为 JointState，在 RViz 中可视化

---

## 文件结构

```
xrobotoolkit_teleop/
├── hardware/
│   ├── interface/
│   │   ├── qj2.py                    # QJ2 硬件接口（QJ2DualArmController）
│   │   └── galaxea.py                # R1Lite 硬件接口（A1XController）
│   ├── qj2_teleop_controller.py      # QJ2 遥操作控制器
│   └── galaxea_r1_lite_teleop_controller.py  # R1Lite 遥操作控制器
└── scripts/
    ├── hardware/
    │   ├── teleop_qj2_hardware.py    # QJ2 启动脚本
    │   └── teleop_r1lite_hardware.py # R1Lite 启动脚本
    └── simulation/
        └── teleop_qj2_placo.py       # QJ2 Placo 仿真脚本
```

---

## 参考文档

- QJ 机器人 ROS 接口说明: `QJ机器人ROS接口说明.md`
- URDF 文件: `assets/qj2/urdf/arms_gen2_hand.urdf`
