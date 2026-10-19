# XRoboToolkit



XenseConsole 用户手册：[`XenseConsole-用户手册.pdf`](XenseConsole-用户手册.pdf




# 环境要求
组件	要求
系统Ubuntu 22.04
Python 3.10。Humble 的 rclpy 和千觉 SDK 都是 cpython-310
ROS ROS 2 Humble（source /opt/ros/humble/setup.bash）。用到 rclpy、sensor_msgs、std_msgs、geometry_msgs，录包用 ros2 bag
Python 包  numpy、placo（带 placo_utils）、meshcat、tyro、opencv-python；力传感器要 pyserial；开相机要 pyrealsense2
PICO  /opt/apps/roboticsservice 里的 PC Service，以及同架构的 libPXREARobotSDK.so，用来编 xrobotoolkit_sdk
机器人侧  同一 ROS 2 域里要有 /motor/q2w_upper_motor_joint_state；本进程发布 /throw/arm_joint_command 和 /motor/tt_upper_motor_csp_control_data
夹爪（默认开） third_party/xense（CycloneDDS）。左 192.168.99.30，右 192.168.98.3
六纬力传感     /dev/ttyUSB1、/dev/ttyUSB2，需要 usbserial

大部分情况下，只需要做如下步骤
1.安装XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb
2.安装缺少的Python包依赖
3.安装编译好的xrobotoolkit_sdk

具体情况具体分析



#   硬件迁移

如果机器人不是2m,参考如下，是的话代码可以套用，外设如相机，夹爪，力传感器需要自行接入遥操作中，后续会写
机器人本身
*一句话：** XR 输入、增量位姿映射、夹爪目标计算、IK 求解、线程框架已经是通用的。移植时通常**不重写遥操算法**，而是为新机器人补齐三层：

- 机器人 URDF 和关节命名
- 机器人硬件接口
- 机器人专属的 teleop controller

### 代码结构

路径均相对 `src/XRoboToolkit-Teleop-Sample-Python/`。

#### 通用遥操作逻辑

文件：`xrobotoolkit_teleop/common/base_teleop_controller.py`

与机器人型号无关的能力：

- XR 控制器位姿读取
- 手柄坐标系到机器人世界坐标系的变换
- 增量位姿计算
- 末端位姿或位置目标更新
- Placo IK 求解
- 夹爪目标计算
- Motion tracker 支持

适配新机时**通常不改这里**。

#### 通用硬件运行框架

文件：`xrobotoolkit_teleop/common/base_hardware_teleop_controller.py`

封装硬件 teleop 执行框架：

- 控制线程、IK 线程、日志线程、相机线程
- rosbag 录制、启停和清理

适配时一般**继承该基类**，实现它要求的接口方法。

#### 机器人专属适配层（移植重点）

示例：

- `xrobotoolkit_teleop/hardware/qj2_teleop_controller.py`
- `xrobotoolkit_teleop/hardware/piper_dual_teleop_controller.py`
- `xrobotoolkit_teleop/hardware/interface/qj2.py`
- 本仓：`arms_2w_hardware_teleop_controller.py`、`arms_2w_ros_bridge.py`

这一层负责：

- 初始化机器人通信接口
- 从真机读关节状态
- 把 Placo 求出的目标关节发给机器人
- 夹爪映射
- 底盘、腰部、相机等附加能力

### 移植六步

#### 第 1 步：准备新机器人的 URDF

Placo IK 依赖 URDF，须确认：

- 关节链完整
- 左右臂关节名正确
- 末端执行器 link 名正确
- 关节顺序与真机一致
- 关节方向定义合理

URDF 不对，后面的 IK、映射、控制都会错。

#### 第 2 步：定义 manipulator config

参考 `qj2_teleop_controller.py` 的 `DEFAULT_MANIPULATOR_CONFIG`（本仓见 `scripts/hardware/teleop_arms_2w_hardware.py`）。

每条臂至少定义：

- `link_name`：末端 URDF link
- `pose_source`：通常 `left_controller` / `right_controller`
- `control_trigger`：激活该臂的按键（如 grip）
- `gripper_config`：夹爪配置（可选）

```python
DEFAULT_MANIPULATOR_CONFIG = {
    "right_arm": {
        "link_name": "right_ee_link",
        "pose_source": "right_controller",
        "control_trigger": "right_grip",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "right_trigger",
            "joint_names": ["right_gripper_joint"],
            "open_pos": [0.0],
            "close_pos": [0.04],
        },
    },
    "left_arm": {
        "link_name": "left_ee_link",
        "pose_source": "left_controller",
        "control_trigger": "left_grip",
        "gripper_config": {
            "type": "parallel",
            "gripper_trigger": "left_trigger",
            "joint_names": ["left_gripper_joint"],
            "open_pos": [0.0],
            "close_pos": [0.04],
        },
    },
}
```

#### 第 3 步：实现硬件接口

在 `xrobotoolkit_teleop/hardware/interface/` 下新增，例如 `my_robot.py`。

把 ROS / ROS2 / SDK / CAN / 串口包装成统一调用，至少支持：

- 读当前关节状态
- 发目标关节命令
- 有夹爪则再发夹爪命令

双臂建议理清：

- `get_left_arm_state()` / `get_right_arm_state()`
- `publish_arm_control()`
- `set_gripper_position()`

参考：`interface/qj2.py`；本仓 CSP 桥：`arms_2w_ros_bridge.py`。

#### 第 4 步：新增 teleop controller

在 `xrobotoolkit_teleop/hardware/` 下新增，例如 `my_robot_teleop_controller.py`，继承 `HardwareTeleopController`。

最少实现：

- `_robot_setup()`
- `_initialize_camera()`
- `_update_robot_state()`
- `_send_command()`
- `_get_robot_state_for_logging()`
- `_get_camera_frame_for_logging()`
- `_shutdown_robot()`

其中最关键的三个：

**`_robot_setup()`** — 初始化 ROS/SDK、建立连接、等待第一帧状态。

**`_update_robot_state()`** — 把真机关节角写入 Placo：

```python
self.placo_robot.state.q[self.placo_arm_joint_slice["left_arm"]] = left_qpos
self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]] = right_qpos
```

**`_send_command()`** — 把 IK 目标关节发到机器人：

```python
q_target = self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]].copy()
# 再按本机接口格式封装发送
```

#### 第 5 步：建立关节映射

移植最容易出错的一步。在 `_placo_setup()` 之后建立每条臂在 Placo 向量中的切片，例如：

```python
self.placo_arm_joint_slice["right_arm"] = slice(
    self.placo_robot.get_joint_offset("right_joint1"),
    self.placo_robot.get_joint_offset("right_joint7") + 1,
)
```

必须确认：

- URDF 关节名与代码一致
- 关节顺序与真实驱动顺序一致
- 每个关节正负号与真机一致

映射错时，IK「看起来能解」，真机却乱动。

#### 第 6 步：增加启动脚本

在 `scripts/hardware/` 下增加入口，参考 `teleop_qj2_hardware.py` / 本仓 `teleop_arms_2w_hardware.py`：

```python
import tyro
from xrobotoolkit_teleop.hardware.my_robot_teleop_controller import MyRobotTeleopController

def main():
    controller = MyRobotTeleopController()
    controller.run()

if __name__ == "__main__":
    tyro.cli(main)
```

### 推荐开发顺序

不要一次接齐所有功能：

1. **先打通状态读取** — 左右臂关节、夹爪状态、时间戳，并正确写入 `placo_robot.state.q`
2. **再打通命令下发** — 先不接 XR，手工发固定关节，确认顺序 / 单位 / 方向 / 频率
3. **再接入 XR 遥操** — 末端是否跟手、左右是否串、姿态方向、`scale_factor`
4. **最后加夹爪、相机、底盘、腰部** — 见第 外设接入方式

### 常见坑位

| 问题 | 说明 |
|------|------|
| URDF 顺序 ≠ 驱动顺序 | `joint1..N`、position 数组、下发数组三者必须一致 |
| 单位不一致 | 关节 rad/deg；夹爪 mm/m/归一化；底盘 m/s 等 |
| 关节方向相反 | 在接口层做符号映射，不要硬拧 IK 权重 |
| 末端 `link_name` 配错 | 手不跟、姿态怪，常是 link 填错 |
| 控制频率不匹配 | 脚本默认约 100 Hz，驱动可能只适合 20–50 Hz → 抖、延迟、报错 |
| 夹爪范围映射错 | 框架按 `open_pos`–`close_pos`；真实接口可能是 0–255 / 角度 / 百分比，在硬件层转换 |

### 可复用 vs 需重写

**通常直接复用：**

- `BaseTeleopController`、`HardwareTeleopController`
- XR 输入、位姿增量、Placo IK 主循环、平行夹爪目标计算

**通常需为新机重写/适配：**

- `hardware/interface/xxx.py`
- `hardware/xxx_teleop_controller.py`
- 启动脚本、`DEFAULT_MANIPULATOR_CONFIG`
- 关节名与切片映射

### 最小迁移清单

1. 准备可用 URDF  
2. 确认末端 link 名称  
3. 确认每个关节的名称、顺序、方向、单位  
4. 写硬件接口类  
5. 写 `xxx_teleop_controller.py`  
6. 实现 `_update_robot_state()` 和 `_send_command()`  
7. 测试固定关节命令  
8. 测试 XR 跟随  
9. 调整 `scale_factor` 和控制频率  
10. 最后补齐夹爪、相机、底盘  

### 建议参考的现有实现

| 类型 | 参考 |
|------|------|
| QJ2 双臂一体 | `hardware/qj2_teleop_controller.py` |
| Piper 双臂独立 | `hardware/piper_dual_teleop_controller.py` |
| UR 双臂 | `hardware/dual_arm_ur_controller.py` |
| 本仓 arms_2w（ROS CSP） | `hardware/arms_2w_hardware_teleop_controller.py` |

**总结：** 移植本质是做「机器人适配层」，不是重写遥操算法。先保证 URDF、关节映射、状态读取、命令下发正确，再让现有 XR + IK 框架接管。

---


# 六纬力传感器

Force.cpp 是一个独立的 ROS 2 力传感器节点，不跑遥操作。
打开左右两个串口（默认 /dev/ttyUSB1、/dev/ttyUSB2）
配置传感器并连续读六维力（Fx Fy Fz Tx Ty Tz）
可选滤波（默认 10 Hz 低通）
发布 WrenchStamped：
左：force/left_arm、/wrench_filtered_left
右：force/right_arm、/wrench_filtered_right
另把右臂原始镜像到 force（兼容旧订阅）
作用概括： 把腕部六维力从串口读进 ROS，供别的节点看或录制

六纬力传感器只负责在在遥操作过程中记录六纬力数据，可以根据Force.cpp将六纬力传感器接入遥操作
可能需要修改USB拓扑   回退 tty  话题名，最好双臂的六纬力传感器分别接入。

# D435i相机

只需要采集图像。直接识别相机，将D435i相机的SDK接入遥操作，遥操作过程调用相机的SDK进行图像采集
换了相机，但是还是D435i相机这个型号，重新识别相机序列号，接入即可，底层用的都是相同的SDK
不是这个型号的话，需要获得相机的SDK，然后接入遥操作中

# xence夹爪
~/XRoboToolkit/third_party/xense
遥操作不重新实现夹爪协议，只 import 官方包，接入数据流。
官方 SDK 负责硬件通信，遥操作层只做映射与调度。将SDK接入遥操作，与相机相同，遥操作过程调用SDK进行控制和数据采集。
xence是网口通信，左 192.168.99.30，右 192.168.98.3，在同一台机器人上需要网段不同，具体修改方法参考
XenseConsole-用户手册.pdf里的网口设置

# DH夹爪
和xenxe夹爪相同，将控制DH夹爪的gripper文件接入遥操作中即可



# 跑仿真验证（不接机器人）：按 说明.txt 的仿真步骤启动 PICO 服务和 scripts/simulation/teleop_qj2m_placo.py
说明  pico可以通信
需注意，电脑，pico,机器人需要在同一局域网

#真机前检查（先不要启动遥操作）：
  确认 ROS_DOMAIN_ID 和机器人一致
 ros2 topic echo /motor/q2w_upper_motor_joint_state --once 能收到数据
  确认没有其他节点在发布 /throw/arm_joint_command
  
   启动环境
ROS_DOMAIN_ID — 逻辑隔离域
DDS 用 Domain ID 把网络划成互不相通的房间。两边必须是同一个数字（常见默认 0）。

主控在 30，本机在 0 → 彼此发现不到，话题对不上。
部署文档里要求：机器人若用了非默认域，本机启动前要 export ROS_DOMAIN_ID=<同一个值>。

RMW_IMPLEMENTATION — 中间件实现
ROS 2 的 RMW 可以选择不同 DDS 实现，例如：
rmw_fastrtps_cpp（Fast DDS）
rmw_cyclonedds_cpp（Cyclone DDS）
不同实现的发现/序列化不互通。一边 FastDDS、一边 Cyclone，即使 Domain ID 相同，通常也收不到对方话题。真机两侧要选同一种。

ROS_LOCALHOST_ONLY — 是否只在本机通信
1：只走本机回环，不参与跨机器 DDS。
0 或不设：允许局域网组播/发现，才能和电机主控互通。
本机若设成 ROS_LOCALHOST_ONLY=1，本机进程之间能通信，但电机主控在另一台机器上时，本机永远收不到关节状态、也发不出臂指令。


换机器人时，这三项不要沿用上一台机的值，也不要写进 Python 代码当默认值。它们描述的是「本机遥操作进程」和「新机器人电机主控」能不能进同一个 DDS 网络。URDF、关节映射、话题名可以按机型改；这三项必须以新主控实际在用的配置为准，再让本机对齐。





# 设置启动脚本
最后写一个本机的启动脚本，包括 PICO 服务、环境变量（PYTHONPATH、LD_LIBRARY_PATH）和 scripts/hardware/teleop_qj2m_hardware.py 的启动命令。
一个用与启动pico sevice 服务
一个用于启动遥操作
方便后续数据采集


# 手柄按键（默认）

| 左 / 右 Grip | 按住激活对应臂跟踪；松开保持当前关节 |
| 左 / 右 Trigger | 夹爪开合（0 开 → 1 闭，） |
| B | 开始 / 停止数据记录（力 / 相机 / 触觉需对应驱动已开） |


遥操作步骤
1.机器人上电
2.运行启动脚本
3.连接pico,进入XRoboToolkit这个软件，若机器人，电脑，pico处于同一局域网，会弹出ip,连接这个ip,然后勾选这个页面左半部分除swtich button这个选项，
然后点击 Reconnect 出现 绿色 Working 说明连接成功，接下可以进行遥操作数据采集
4.握持键用于控制手臂移动，握住握持键，手臂跟随运动，松开握持键，手臂不跟随运动
    扳机键用于控制夹爪的闭合
  B键用于数据的录取，按一次，开始记录数据，再按一次，停止记录数据。
    数据应该在:~/XRoboToolkit/XRoboToolkit-Teleop-Sample-Python/logs文件夹下


























