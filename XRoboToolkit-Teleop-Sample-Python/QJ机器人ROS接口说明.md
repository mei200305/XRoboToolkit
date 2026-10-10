# QJ机器人ROS接口说明

## 一、关节控制话题（发送命令给机器人）

### 话题名称
```
motor_command/arm_position_control
```

### 消息类型
```
std_msgs/Float64MultiArray
```

### 数据格式
**28维数组**，按以下顺序组织：

```
motor_msgs_.data[0:6]   = 左臂7个关节位置 (jnts_l.pos)   [单位: 弧度 rad]
motor_msgs_.data[7:13]  = 右臂7个关节位置 (jnts_r.pos)   [单位: 弧度 rad]
motor_msgs_.data[14:20] = 左臂7个关节速度 (jnts_l.vel)   [单位: 弧度/秒 rad/s]
motor_msgs_.data[21:27] = 右臂7个关节速度 (jnts_r.vel)   [单位: 弧度/秒 rad/s]
```

**详细说明：**
- **索引 0-6**：左臂关节1-7的位置（弧度）
- **索引 7-13**：右臂关节1-7的位置（弧度）
- **索引 14-20**：左臂关节1-7的速度（弧度/秒）
- **索引 21-27**：右臂关节1-7的速度（弧度/秒）

**注意：**
- 根据代码 `DataProc_Move.cpp`，gen2版本的速度部分可能未填充（注释掉了）
- gen3版本的速度部分填充为0
- 如果只需要位置控制，可以只填充前14个元素，后14个填0

### 关节名称顺序（gen2）
- 左臂：`l_arm_Joint1`, `l_arm_Joint2`, ..., `l_arm_Joint7`
- 右臂：`r_arm_Joint1`, `r_arm_Joint2`, ..., `r_arm_Joint7`

### 关节名称顺序（gen3）
- 左臂：`AJL1`, `AJL2`, ..., `AJL7`
- 右臂：`AJR1`, `AJR2`, ..., `AJR7`

### 代码位置
- 发布者：`src/robot_controller/utils/DataProc_Init.cpp` (第27行)
- 数据填充：`src/robot_controller/utils/DataProc_Move.cpp` (第68-69行, 88-89行)

---

## 二、关节实时数据话题（从机器人读取状态）

### 话题名称
```
motor_state/arm_motor_state_actual
```

### 消息类型
```
sensor_msgs/JointState
```

### 数据格式
**JointState消息包含以下字段：**

1. **position** (std::vector<double>)
   - **长度**：14个元素
   - **顺序**：
     - `position[0:6]`：左臂7个关节位置（弧度）
     - `position[7:13]`：右臂7个关节位置（弧度）

2. **effort** (std::vector<double>)
   - **长度**：14个元素
   - **顺序**：
     - `effort[0:6]`：左臂7个关节电流（单位：安培 A）
     - `effort[7:13]`：右臂7个关节电流（单位：安培 A）

3. **其他字段**（可能未使用）
   - `velocity`：关节速度（如果提供）
   - `name`：关节名称列表（如果提供）

### 数据验证
- 代码会检查 `position.size() == 14`，如果不匹配会报错并退出
- 位置限制检查：每个关节位置必须在 `[-π, π]` 范围内
- 电流限制检查：每个关节电流必须在 `[-15, 15]` 安培范围内

### 代码位置
- 订阅者：`src/robot_controller/utils/DataProc_Init.cpp` (第102行)
- 回调处理：`src/robot_controller/utils/DataProc_Callback.cpp` (第17-63行)

---

## 三、夹爪控制话题

### 话题名称
**左手夹爪：**
```
Robotiq2FGripperRobotOutput
```

**右手夹爪：**
```
Robotiq2FGripperRobotOutputRight
```

（可通过ROS参数 `~left_gripper_topic` 和 `~right_gripper_topic` 自定义）

### 消息类型
```
robot_controller/Robotiq2FGripper_robot_output
```

### 消息定义
```msg
uint8 rACT    # 激活标志 (0=重置, 1=激活)
uint8 rGTO    # 执行标志 (0=停止, 1=执行)
uint8 rATR    # 自动释放 (0=禁用, 1=启用)
uint8 rPR     # 位置命令 (0=张开, 255=闭合)
uint8 rSP     # 速度 (0-255)
uint8 rFR     # 力 (0-255)
```

### 数据说明
- **rACT**：激活夹爪（启动时设为1）
- **rGTO**：允许执行（正常运行时设为1，检测到接触时可设为0停止）
- **rATR**：自动释放（通常设为0禁用）
- **rPR**：目标位置（0=完全张开，255=完全闭合）
- **rSP**：速度（0-255，255=最快）
- **rFR**：力（0-255，数值越大夹持力越大，默认50以保护物体）

### 夹爪状态反馈话题（可选）
**左手夹爪状态：**
```
Robotiq2FGripperRobotInput
```

**右手夹爪状态：**
```
Robotiq2FGripperRobotInputRight
```

**消息类型：**
```
robot_controller/Robotiq2FGripper_robot_input
```

用于读取夹爪当前状态（位置、速度、力、接触检测等）

### 代码位置
- 发布者：`src/robot_controller/scripts/vr_to_robotiq_gripper.py`
- 消息定义：`src/robot_controller/msg/Robotiq2FGripper_robot_output.msg`

---

## 四、手部/手指控制话题（如果使用灵巧手）

### 话题名称
**左手手指：**
```
/qpos_fingers_left
```

**右手手指：**
```
/qpos_fingers_right
```

### 消息类型
```
std_msgs/Int32MultiArray
```

### 数据格式
**6个元素数组**，对应6个手指关节：
- `data[0]`：pinky（小指）
- `data[1]`：ring（无名指）
- `data[2]`：middle（中指）
- `data[3]`：index（食指）
- `data[4]`：thumb_pip（拇指PIP关节）
- `data[5]`：thumb_mcp（拇指MCP关节）

**数值范围：** 0-2000（映射后的值）

### 代码位置
- 发布者：`src/qj2_openTeleVision/scripts/tele_dual_arm_test.py` (第69-70行, 206-207行)

---

## 五、总结表格

| 功能 | 话题名称 | 消息类型 | 数据格式 | 方向 |
|------|---------|---------|---------|------|
| **关节控制** | `motor_command/arm_position_control` | `std_msgs/Float64MultiArray` | 28维：[左7位置, 右7位置, 左7速度, 右7速度] | 发布 |
| **关节状态** | `motor_state/arm_motor_state_actual` | `sensor_msgs/JointState` | position[14], effort[14] | 订阅 |
| **左手夹爪** | `Robotiq2FGripperRobotOutput` | `robot_controller/Robotiq2FGripper_robot_output` | 6个uint8字段 | 发布 |
| **右手夹爪** | `Robotiq2FGripperRobotOutputRight` | `robot_controller/Robotiq2FGripper_robot_output` | 6个uint8字段 | 发布 |
| **左手手指** | `/qpos_fingers_left` | `std_msgs/Int32MultiArray` | 6个int32 (0-2000) | 发布 |
| **右手手指** | `/qpos_fingers_right` | `std_msgs/Int32MultiArray` | 6个int32 (0-2000) | 发布 |

---

## 六、在XRoboToolkit项目中的使用

基于以上信息，在 `/home/szj/XRoboToolkit-Teleop-Sample-Python` 项目中：

### 1. 关节控制发布器
```python
import rospy
from std_msgs.msg import Float64MultiArray

pub = rospy.Publisher("/motor_command/arm_position_control", Float64MultiArray, queue_size=10)

msg = Float64MultiArray()
msg.data = [0.0] * 28
# 填充左臂位置
msg.data[0:7] = q_left  # 7个关节位置（弧度）
# 填充右臂位置
msg.data[7:14] = q_right  # 7个关节位置（弧度）
# 速度部分（如果需要）
msg.data[14:21] = [0.0] * 7  # 左臂速度
msg.data[21:28] = [0.0] * 7  # 右臂速度

pub.publish(msg)
```

### 2. 关节状态订阅器（可选，用于反馈）
```python
import rospy
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    if len(msg.position) == 14:
        q_left_actual = msg.position[0:7]
        q_right_actual = msg.position[7:14]
        # 使用实际关节位置更新Placo状态

rospy.Subscriber("/motor_state/arm_motor_state_actual", JointState, joint_state_callback)
```

### 3. 夹爪控制（如果使用Robotiq夹爪）
```python
from robot_controller.msg import Robotiq2FGripper_robot_output

gripper_pub = rospy.Publisher("Robotiq2FGripperRobotOutput", Robotiq2FGripper_robot_output, queue_size=10)

cmd = Robotiq2FGripper_robot_output()
cmd.rACT = 1  # 激活
cmd.rGTO = 1  # 执行
cmd.rATR = 0  # 禁用自动释放
cmd.rPR = 128  # 位置（0-255）
cmd.rSP = 255  # 速度
cmd.rFR = 50   # 力（安全值）

gripper_pub.publish(cmd)
```

---

## 七、注意事项

1. **关节顺序**：确保XRoboToolkit项目中的关节顺序与QJ机器人一致
2. **单位**：关节位置和速度使用弧度（rad）和弧度/秒（rad/s）
3. **数据验证**：发布前检查关节角度是否在合理范围内
4. **频率**：建议控制频率100Hz（与现有teleop_arm.cpp一致）
5. **坐标系**：确保XRoboToolkit的坐标系与QJ机器人坐标系匹配
