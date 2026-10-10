# QJ2M 腕部六维力传感器接入遥操作

本文说明左右腕六维力传感器如何挂进 QJ2M 真机遥操作：串口读数 → 滤波 → 对齐到 30 Hz 采集帧 → 写入 rosbag。

力数据**不参与** IK / 关节命令闭环，只做同步采集与发布。

---

## 1. 一句话流程

```
腕部传感器 (CH340 串口, 115200)
  → WristForceSensors 后台线程解析 30B 包 @100 Hz
  → 原始 Fx..Tz + 一阶低通滤波
  → _on_force_sample 写入 _force_history
  → D435i 帧时钟 / 30 Hz 定时器取最近样本
  → 发布 WrenchStamped，B 键 rosbag 一并录下
```

对照实现：

| 角色 | 路径 |
|------|------|
| 串口协议与驱动（Python，遥操作在用） | `xrobotoolkit_teleop/hardware/interface/force_sensor.py` |
| 接入控制器、发布与录包 | `xrobotoolkit_teleop/hardware/qj2m_teleop_controller.py` |
| 启动入口（打印话题） | `scripts/hardware/teleop_qj2m_hardware.py` |
| 独立 C++ ROS2 节点（协议同源，可单独跑） | 仓库根目录 `Force.cpp` |

---

## 2. 硬件与串口约定

左右臂各一颗腕部六维力，经 USB 转串口（CH340）接到 Jetson。

| 臂 | USB 拓扑（优先） | 回退 tty | 话题前缀 |
|----|------------------|----------|----------|
| 左 | `1-4.4.4` 或 `1-4.4.3` | `/dev/ttyUSB1` | `/force/left_arm` |
| 右 | `1-4.3.2` | `/dev/ttyUSB2` | `/force/right_arm` |

查找逻辑（`find_arm_port`）：

1. 按固定 USB 口在 `/sys/bus/usb/devices/<usb>:1.0` 下找 `ttyUSB*`。
2. 找不到再回退到上表的 tty 名，且**不会**占用对侧臂已经占用的口（避免重插后左右对调）。

若端口全找不到，日志会提示加载串口驱动（每 boot 一次），例如：

```bash
sudo modprobe usbserial
sudo insmod /home/ubuntu/ch341-5.15.199-tegra.ko
```

波特率 **115200**。上电配置与连续采样命令（与 `Force.cpp` 一致）：

```
AT+SMPF=100      # 采样 100 Hz
AT+DCPCU=MVPV    # 单位
AT+DCPM=?
AT+GSD           # 开始连续输出
```

数据包：**30 字节**，头 `0xAA 0x55`，偏移 6 起 6 个 little-endian `float`：

`Fx, Fy, Fz, Tx, Ty, Tz`

有效范围粗校验：力 ±600，力矩 ±20；超界丢弃。超过 **2 s** 无包则断开重连。

---

## 3. 在遥操作里何时启动

入口：`python scripts/hardware/teleop_qj2m_hardware.py` → `QJ2MTeleopController`。

在 `_robot_setup` 末尾、夹爪与 D435i 之后调用：

```python
self._setup_xense_grippers()
self._start_d435_publisher()
self._start_force_sensor()
```

`_start_force_sensor` 创建 `WristForceSensors`，并注册回调：

```python
self._force_sensor.start(self._on_force_sample)
```

每个臂一条 daemon 线程（`left-arm-force` / `right-arm-force`）。关机时 `_shutdown_robot` 里 `stop()`。

没有单独的 CLI 开关：真机控制器起来就会尝试接传感器；找不到口则周期性重试，**不阻塞**遥操主循环。

---

## 4. 滤波与缓存

- 采样标称 **100 Hz**。
- 一阶低通：截止 **10 Hz**（`FORCE_CUTOFF_HZ`），与 `Force.cpp` 默认 `lowpass` 一致。
- 每收到一包立刻调用 `on_sample(arm_name, raw, filtered)`。
- 控制器侧：

```python
def _on_force_sample(self, arm_name, raw, filtered):
    self._force_history[arm_name].append(time.time(), (tuple(raw), tuple(filtered)))
```

`_force_history` 每臂 `_SampleHistory(maxlen=128)`，用主机 `time.time()` 打戳，供后续按相机帧时间取最近邻。

读串口时用 `ser.read(max(1, ser.in_waiting))`，避免固定大块读把多包挤成一坨、时间戳挤成 50 ms 突发，影响与 30 Hz 对齐。

---

## 5. 发布话题与 frame

消息类型：`geometry_msgs/WrenchStamped`。数值发布前用 `round_half_away` 取整，对齐 C++ 的 `std::llround`。

| 话题 | 内容 | frame_id |
|------|------|----------|
| `/force/left_arm` | 左臂原始 | `l_arm_force_link` |
| `/wrench_filtered_left` | 左臂滤波 | `l_arm_force_link` |
| `/force/right_arm` | 右臂原始 | `r_arm_force_link` |
| `/wrench_filtered_right` | 右臂滤波 | `r_arm_force_link` |
| `/force` | 右臂原始镜像（兼容旧订阅） | `force_link` |

Publisher 在 `_ensure_dataset_publishers` 里与 VLA 臂状态、Xense、D435i 一起创建。

---

## 6. 与 30 Hz 采集帧对齐（rosbag）

力传感器本身约 100 Hz；数据集发布按 **D435i 彩色帧 ~30 Hz**（无相机时用同频率定时器）。

每帧 `_publish_vla_frame(t_capture, ...)`：

1. 用 `t_capture`（曝光/主机时间）作统一 `header.stamp`。
2. 对每臂 `self._force_history[arm].nearest(t_capture, VLA_MAX_SAMPLE_AGE_S)`，`VLA_MAX_SAMPLE_AGE_S = 0.1`。
3. 超龄则该帧跳过该臂力话题；否则同 stamp 发布 raw + filtered（右臂再 mirror 一份 `/force`）。

B 键启停 rosbag 时，话题列表 `_get_rosbag_topics()` 包含 `force_record_topics()`，因此六维力与关节、夹爪、触觉、相机在同一 bag、同一时间轴。

力**不**写入 `_get_robot_state_for_logging()` 的 JSON 日志，只走 ROS 话题 / bag。

---

## 7. 与控制环的关系

```
PICO → Placo IK → /throw/arm_joint_command     （控制）
腕力 → history → 30 Hz WrenchStamped / bag      （感知采集）
```

当前实现：**力不反馈到遥操作增益、阻抗或安全停机**。若以后要做力控或接触检测，应从 `_force_history` 或订阅上述话题接入，而不是再开一套串口。

---

## 8. 与独立节点 `Force.cpp` 的关系

`Force.cpp` 是同协议的独立 ROS2 节点（`force_node`）：双串口、同样 AT 命令、低通/滑动平均可选、话题命名一致（左 `force/left_arm`，右带 `/force` 镜像）。

遥操作路径已把协议迁到 Python `WristForceSensors`，**不必**再并行跑 `Force.cpp`，否则会抢串口。调试串口或非遥操场景可单独编译运行 C++ 节点。

---

## 9. 排障速查

| 现象 | 排查 |
|------|------|
| `[Force] wrist sensor port not found` | USB 口、`ch341` 驱动、`ls /dev/ttyUSB*`、`ls /sys/bus/usb/devices/1-4.*` |
| 左右力对调 | 看 USB 拓扑是否仍是 1-4.4.* / 1-4.3.2；tty 号会变，以 USB 口为准 |
| `no data for 2s, reconnecting` | 传感器未进连续模式或掉线；重插后等配置命令跑完 |
| bag 里没有力 | 确认 B 键录包、传感器已 `connected`、样本年龄 ≤ 0.1 s |
| 与相机时间对不齐 | 确认用 `in_waiting` 读串口，而非大块阻塞读 |

---

## 10. 关键常量速查

见 `force_sensor.py`：

- `FORCE_SAMPLE_HZ = 100`
- `FORCE_CUTOFF_HZ = 10.0`
- `FORCE_STALL_TIMEOUT_S = 2.0`
- `_PACKET_LEN = 30`
