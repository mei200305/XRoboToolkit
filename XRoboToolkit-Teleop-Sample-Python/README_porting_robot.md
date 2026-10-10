# Teleoperation Porting Guide

## 目的

这份文档说明如何将当前仓库中的遥操作算法移植到其他机器人上。

核心结论只有一句话：

本项目里的 XR 输入处理、增量位姿映射、夹爪目标计算、IK 求解、线程框架，本身已经是通用的。移植时通常不需要重写遥操作算法，而是需要为新机器人补齐以下三层：

- 机器人 URDF 和关节命名
- 机器人硬件接口
- 机器人专属的 teleop controller

## 代码结构

移植前，建议先理解当前工程的分层。

### 1. 通用遥操作逻辑

文件: `xrobotoolkit_teleop/common/base_teleop_controller.py`

这里包含了大部分与机器人型号无关的能力：

- XR 控制器位姿读取
- 手柄坐标系到机器人世界坐标系的变换
- 增量位姿计算
- 末端位姿或位置目标更新
- Placo IK 求解
- 夹爪目标计算
- Motion tracker 支持

如果你要适配新的机器人，通常不需要改这里。

### 2. 通用硬件运行框架

文件: `xrobotoolkit_teleop/common/base_hardware_teleop_controller.py`

这里封装了硬件 teleop 的执行框架：

- 控制线程
- IK 线程
- 日志线程
- 相机线程
- rosbag 录制
- 启停和清理逻辑

适配新机器人时，一般只需要继承这个基类并实现它要求的几个接口方法。

### 3. 机器人专属适配层

示例文件：

- `xrobotoolkit_teleop/hardware/qj2_teleop_controller.py`
- `xrobotoolkit_teleop/hardware/piper_dual_teleop_controller.py`
- `xrobotoolkit_teleop/hardware/interface/qj2.py`

这一层才是移植工作的重点。它主要负责：

- 初始化机器人通信接口
- 从真实机器人读取关节状态
- 把 Placo 求出的目标关节发给机器人
- 做夹爪映射
- 处理底盘、腰部、相机等附加能力

## 移植时你真正需要做什么

可以把整个迁移过程理解为下面 6 步。

### 第 1 步：准备新机器人的 URDF

这是最关键的一步。

Placo 的 IK 依赖 URDF，因此你必须先确认：

- 关节链是完整的
- 左右臂关节名称正确
- 末端执行器 link 名称正确
- 关节顺序和真实机器人一致
- 关节方向定义合理

如果 URDF 本身就不对，后面的 IK、映射、控制都会出问题。

### 第 2 步：定义 manipulator config

参考 `qj2_teleop_controller.py` 里的 `DEFAULT_MANIPULATOR_CONFIG`。

每个手臂至少要定义：

- `link_name`: 末端执行器对应的 URDF link
- `pose_source`: XR 输入源，通常是 `left_controller` 或 `right_controller`
- `control_trigger`: 哪个按键用于激活该手臂
- `gripper_config`: 夹爪配置

一个典型配置大致像这样：

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

### 第 3 步：实现新机器人的硬件接口

建议在 `xrobotoolkit_teleop/hardware/interface/` 下新增一个接口文件，例如：

```text
xrobotoolkit_teleop/hardware/interface/my_robot.py
```

这个接口层的职责是把你的 ROS、ROS2、SDK、CAN 或串口协议包装成统一调用方式。至少需要支持：

- 读取当前关节状态
- 发送目标关节命令
- 如果有夹爪，再发送夹爪命令

如果你的机器人是双臂，建议尽量在这里就把左右臂状态组织清楚，例如：

- `get_left_arm_state()`
- `get_right_arm_state()`
- `publish_arm_control()`
- `set_gripper_position()`

QJ2 的例子可以参考：

- `xrobotoolkit_teleop/hardware/interface/qj2.py`

### 第 4 步：新增 teleop controller

建议在 `xrobotoolkit_teleop/hardware/` 下新增一个文件，例如：

```text
xrobotoolkit_teleop/hardware/my_robot_teleop_controller.py
```

并继承 `HardwareTeleopController`。

最少需要实现这些方法：

- `_robot_setup()`
- `_initialize_camera()`
- `_update_robot_state()`
- `_send_command()`
- `_get_robot_state_for_logging()`
- `_get_camera_frame_for_logging()`
- `_shutdown_robot()`

其中最关键的是下面三个。

#### `_robot_setup()`

负责：

- 初始化 ROS 节点或 SDK
- 建立 Publisher / Subscriber 或设备连接
- 等待机器人第一帧状态到来

#### `_update_robot_state()`

负责把真实机器人当前关节角写入 Placo：

```python
self.placo_robot.state.q[self.placo_arm_joint_slice["left_arm"]] = left_qpos
self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]] = right_qpos
```

#### `_send_command()`

负责把 IK 求出的目标关节发到机器人：

```python
q_target = self.placo_robot.state.q[self.placo_arm_joint_slice["right_arm"]].copy()
```

然后再根据你的机器人接口格式进行封装和发送。

### 第 5 步：建立关节映射

这一步是移植最容易出错的地方。

你需要在 `_placo_setup()` 之后建立每条手臂在 Placo 向量中的切片，例如：

```python
self.placo_arm_joint_slice["right_arm"] = slice(
    self.placo_robot.get_joint_offset("right_joint1"),
    self.placo_robot.get_joint_offset("right_joint7") + 1,
)
```

这里必须确认三件事：

- URDF 里的关节名和代码里一致
- 关节顺序和真实驱动顺序一致
- 每个关节方向正负号和真实机器人一致

如果这里映射错了，表面上 IK 看起来能解，但实际机器人会运动异常。

### 第 6 步：增加启动脚本

最后在 `scripts/hardware/` 下增加一个入口脚本，例如：

```text
scripts/hardware/teleop_my_robot_hardware.py
```

结构可以直接参考 `scripts/hardware/teleop_qj2_hardware.py`。

示意如下：

```python
import tyro

from xrobotoolkit_teleop.hardware.my_robot_teleop_controller import MyRobotTeleopController


def main():
    controller = MyRobotTeleopController()
    controller.run()


if __name__ == "__main__":
    tyro.cli(main)
```

## 推荐的开发顺序

不要一开始就把所有功能一起接上。建议按下面顺序逐步打通。

### 1. 先打通状态读取

先确认你能稳定拿到：

- 左右臂关节位置
- 夹爪状态
- 时间戳

并且能正确更新到 `self.placo_robot.state.q`。

### 2. 再打通命令下发

不要先接 XR，先手工发送一组固定关节命令，确认：

- 关节顺序正确
- 单位正确
- 方向正确
- 控制频率合理

### 3. 再接入 XR 遥操

这时通用算法层就能直接复用，你主要关注：

- 末端是否跟手
- 左右手有没有串
- 姿态方向是否一致
- 比例系数是否合适

### 4. 最后加夹爪、相机、底盘、腰部

这些都属于附加能力，放到最后调试更稳妥。

## 常见坑位

下面这些问题在迁移时最常见。

### 1. URDF 关节顺序和真实机器人驱动顺序不一致

这是最常见的问题之一。一定要明确：

- URDF 中 joint1 到 jointN 的顺序
- 驱动接口里 position 数组的顺序
- 下发命令时数组的顺序

三者必须一致。

### 2. 单位不一致

注意确认：

- 关节角是弧度还是角度
- 夹爪开合是毫米、米还是归一化值
- 底盘速度是 m/s 还是别的单位

### 3. 关节方向相反

IK 在 Placo 中解出来的方向不一定和你的实际机器人驱动方向一致。如果发现“朝相反方向动”，通常要在接口层做符号映射。

### 4. 末端 link 配错

`link_name` 如果配错，IK 目标会追踪到错误的 link，表现通常是：

- 手不跟手柄
- 姿态很奇怪
- 看起来像是“求解错了”

实际上常常只是末端 link 填错了。

### 5. 控制频率不匹配

例如脚本里默认的 `control_rate_hz` 是 100 Hz，但你的机器人驱动可能只适合 20 Hz 或 50 Hz。频率不匹配时容易出现：

- 抖动
- 延迟
- 跟随不稳定
- 驱动报错

### 6. 夹爪范围映射不正确

框架中的夹爪目标通常按 `open_pos` 到 `close_pos` 计算，但真实夹爪接口可能需要：

- 0 到 255
- 角度值
- 行程长度
- 百分比

这一层建议统一在硬件接口里转换。

## 哪些代码可以直接复用

通常下面这些都可以直接复用：

- `BaseTeleopController`
- `HardwareTeleopController`
- XR 输入处理逻辑
- 位姿增量计算逻辑
- Placo IK 主循环
- 平行夹爪目标计算逻辑

通常需要为新机器人重写或适配的是：

- `hardware/interface/xxx.py`
- `hardware/xxx_teleop_controller.py`
- 启动脚本
- `DEFAULT_MANIPULATOR_CONFIG`
- 关节名和切片映射

## 最小迁移清单

如果你想尽快开始，可以按这个最小清单来做。

1. 准备可用 URDF
2. 确认末端 link 名称
3. 确认每个关节的名称、顺序、方向、单位
4. 写硬件接口类
5. 写 `xxx_teleop_controller.py`
6. 实现 `_update_robot_state()` 和 `_send_command()`
7. 测试固定关节命令
8. 测试 XR 跟随
9. 调整 `scale_factor` 和控制频率
10. 最后补齐夹爪、相机、底盘能力

## 建议参考的现有实现

如果你的目标机器人和下面某类结构类似，可以优先参考对应实现。

- QJ2 双臂一体控制: `xrobotoolkit_teleop/hardware/qj2_teleop_controller.py`
- Piper 双臂独立控制: `xrobotoolkit_teleop/hardware/piper_dual_teleop_controller.py`
- UR 双臂控制: `xrobotoolkit_teleop/hardware/dual_arm_ur_controller.py`

## 一句话总结

把这个遥操算法移植到新机器人上，本质上是在做“机器人适配层”，不是重写遥操算法本身。

先保证 URDF、关节映射、状态读取、命令下发是正确的，再让现有的 XR + IK 框架接管控制，这样会最快、最稳。
