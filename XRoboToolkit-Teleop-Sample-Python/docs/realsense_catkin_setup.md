# RealSense 相机编译、安装与启动说明

本文档用于当前机器上的 ROS1 工作空间：

```bash
/home/catkin_ws
```

当前工作空间中已经放入了 RealSense ROS1 legacy 驱动源码：

```bash
/home/catkin_ws/src/realsense-ros-ros1-legacy
```

其中主要 ROS 包为：

- `realsense2_camera`：RealSense 相机 ROS 驱动节点
- `realsense2_description`：RealSense 相机模型、URDF、RViz 配置

当前三相机启动文件为：

```bash
/home/catkin_ws/src/realsense-ros-ros1-legacy/realsense2_camera/launch/three_realsense.launch
```

里面配置了三台相机：

| 位置 | ROS namespace | serial_no |
| --- | --- | --- |
| 左手相机 | `left_camera` | `247122071929` |
| 右手相机 | `right_camera` | `317622070449` |
| 头部相机 | `head_camera` | `243622070764` |

当前 launch 中三台相机均启用了彩色图像，关闭了深度、加速度计和陀螺仪：

```xml
<arg name="enable_depth" value="false"/>
<arg name="enable_accel" value="false"/>
<arg name="enable_gyro"  value="false"/>
```

如果后续需要深度图，需要把对应参数改为 `true`，详见本文“启用深度图”部分。

## 1. 前置条件

推荐环境：

- Ubuntu 20.04
- ROS Noetic
- RealSense USB3.0 接口
- 已安装 `catkin_tools`
- 已安装 Intel librealsense2
- 当前用户有 USB 设备访问权限

先确认 ROS 是否可用：

```bash
source /opt/ros/noetic/setup.bash
rosversion -d
```

正常应输出：

```bash
noetic
```

如果提示 `/opt/ros/noetic/setup.bash` 不存在，说明 ROS Noetic 没有安装或路径不对，需要先安装 ROS。

## 2. 安装系统依赖

### 2.1 安装 ROS 和 catkin 工具

如果系统尚未安装 ROS Noetic，先安装 ROS。若已经安装，可跳过本小节。

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  python3-catkin-tools \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  build-essential
```

初始化 `rosdep`：

```bash
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 提示已经初始化过，可忽略。

### 2.2 安装 RealSense SDK

RealSense ROS 驱动依赖 `librealsense2`。优先使用 apt 安装：

```bash
sudo apt update
sudo apt install -y \
  librealsense2-dkms \
  librealsense2-utils \
  librealsense2-dev \
  librealsense2-udev-rules
```

安装完成后重新插拔 RealSense 相机，或者执行：

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

检查 RealSense SDK 是否识别相机：

```bash
rs-enumerate-devices
```

如果能看到设备型号、序列号和 USB 类型，说明 SDK 侧识别正常。

建议确认 USB 连接为 USB3：

```bash
rs-enumerate-devices | grep -i "usb type"
```

如果显示 USB2，彩色和深度流可能卡顿、掉帧或无法同时启动，应更换 USB3 口、线材或 USB hub。

### 2.3 安装 ROS 包依赖

进入工作空间并安装依赖：

```bash
cd /home/catkin_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

如果 `rosdep` 报某个包找不到，先确认 ROS 版本是 Noetic，再确认网络和 apt 源是否正常。

RealSense 常用依赖也可以手动补装：

```bash
sudo apt install -y \
  ros-noetic-ddynamic-reconfigure \
  ros-noetic-rgbd-launch \
  ros-noetic-image-transport \
  ros-noetic-cv-bridge \
  ros-noetic-nodelet \
  ros-noetic-diagnostic-updater \
  ros-noetic-tf
```

## 3. 编译 catkin_ws

当前工作空间是 catkin 工作空间，建议使用 `catkin build`。

### 3.1 首次编译

```bash
cd /home/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build
```

编译成功后，加载工作空间环境：

```bash
source /home/catkin_ws/devel/setup.bash
```

检查 ROS 是否能找到 RealSense 包：

```bash
rospack find realsense2_camera
```

正常应输出类似：

```bash
/home/catkin_ws/src/realsense-ros-ros1-legacy/realsense2_camera
```

### 3.2 只重新编译 RealSense 包

如果只改了 RealSense launch 或驱动源码，可以只编译相关包：

```bash
cd /home/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build realsense2_camera realsense2_description
source /home/catkin_ws/devel/setup.bash
```

### 3.3 清理后重新编译

如果遇到奇怪的 CMake 缓存、依赖变更、包找不到等问题，可以清理指定包：

```bash
cd /home/catkin_ws
source /opt/ros/noetic/setup.bash
catkin clean realsense2_camera realsense2_description
catkin build realsense2_camera realsense2_description
source /home/catkin_ws/devel/setup.bash
```

如果整个工作空间都需要重编译：

```bash
cd /home/catkin_ws
source /opt/ros/noetic/setup.bash
catkin clean
catkin build
source /home/catkin_ws/devel/setup.bash
```

## 4. 配置 Shell 环境

每次打开新终端后，至少需要加载 ROS 和工作空间：

```bash
source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash
```

为了减少重复操作，可以写入 `~/.bashrc`：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

注意：如果你同时使用 conda，启动 ROS 前建议退出 conda 环境，避免 Python 路径污染：

```bash
conda deactivate
```

当前 `/home/catkin_ws/start_robot.sh` 中已经有类似处理：

```bash
eval "$(conda shell.bash hook)"
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## 5. 启动单台 RealSense 相机

先插好相机，确认设备存在：

```bash
rs-enumerate-devices
```

启动单台相机：

```bash
source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash
roslaunch realsense2_camera rs_camera.launch
```

如果机器上接了多台 RealSense，建议指定序列号，避免 ROS 启错设备：

```bash
roslaunch realsense2_camera rs_camera.launch \
  serial_no:=247122071929 \
  camera:=left_camera \
  tf_prefix:=left_camera \
  color_width:=640 \
  color_height:=480 \
  color_fps:=30 \
  enable_depth:=false \
  enable_accel:=false \
  enable_gyro:=false
```

启动后新开终端检查 topic：

```bash
source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash
rostopic list | grep left_camera
```

常见彩色图像 topic：

```bash
/left_camera/color/camera_info
/left_camera/color/image_raw
/left_camera/color/image_raw/compressed
```

查看图像：

```bash
rqt_image_view
```

然后在下拉框里选择：

```bash
/left_camera/color/image_raw
```

## 6. 启动三台 RealSense 相机

当前已经有三相机 launch：

```bash
/home/catkin_ws/src/realsense-ros-ros1-legacy/realsense2_camera/launch/three_realsense.launch
```

直接启动：

```bash
source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash
roslaunch realsense2_camera three_realsense.launch
```

该 launch 会启动：

- `left_camera`
- `right_camera`
- `head_camera`
- RViz，加载 `realsense2_camera/rviz/3_camera.rviz`

启动后检查 topic：

```bash
rostopic list | grep camera
```

重点确认以下 topic 是否存在：

```bash
/left_camera/color/image_raw
/right_camera/color/image_raw
/head_camera/color/image_raw
```

也可以分别查看频率：

```bash
rostopic hz /left_camera/color/image_raw
rostopic hz /right_camera/color/image_raw
rostopic hz /head_camera/color/image_raw
```

正常情况下应接近 launch 中配置的 `30 Hz`。如果明显低很多，通常是 USB 带宽、线材、hub 或 CPU 压力问题。

## 7. 启动夹爪和三相机

当前工作空间根目录有一键启动脚本：

```bash
/home/catkin_ws/start_robot.sh
```

脚本内容会依次启动：

1. Robotiq 双夹爪
2. 三台 RealSense 相机
3. RViz

运行：

```bash
cd /home/catkin_ws
chmod +x start_robot.sh
./start_robot.sh
```

停止：

```bash
Ctrl+C
```

脚本里已经设置了退出时关闭后台 ROS 进程：

```bash
trap "echo '正在关闭所有节点...'; kill 0" SIGINT SIGTERM
```

如果你只想启动相机，不启动夹爪，请使用：

```bash
roslaunch realsense2_camera three_realsense.launch
```

## 8. 启用深度图

当前三相机 launch 中深度图是关闭的：

```xml
<arg name="enable_depth" value="false"/>
```

如需启用深度，将对应相机改为：

```xml
<arg name="enable_depth" value="true"/>
```

建议同时开启深度和彩色对齐：

```xml
<arg name="enable_depth" value="true"/>
<arg name="align_depth" value="true"/>
```

启用后常见 topic：

```bash
/left_camera/depth/image_rect_raw
/left_camera/aligned_depth_to_color/image_raw
/left_camera/depth/camera_info
```

查看深度图：

```bash
rqt_image_view /left_camera/depth/image_rect_raw
```

如果三台相机都开启深度，USB 带宽压力会明显增加。建议先用一台相机验证，再逐台打开。

## 9. 修改相机序列号

如果更换了 RealSense 相机，需要更新 launch 中的 `serial_no`。

查看当前连接设备序列号：

```bash
rs-enumerate-devices | grep -i "serial"
```

编辑文件：

```bash
vim /home/catkin_ws/src/realsense-ros-ros1-legacy/realsense2_camera/launch/three_realsense.launch
```

修改对应位置：

```xml
<arg name="serial_no" value="247122071929" />
```

修改 launch 文件后通常不需要重新编译，重新 `roslaunch` 即可。

## 10. 常用检查命令

检查 ROS master：

```bash
roscore
```

检查包路径：

```bash
rospack find realsense2_camera
```

检查相机设备：

```bash
rs-enumerate-devices
```

检查节点：

```bash
rosnode list
```

检查 topic：

```bash
rostopic list
```

查看某路图像：

```bash
rqt_image_view /left_camera/color/image_raw
```

查看图像频率：

```bash
rostopic hz /left_camera/color/image_raw
```

查看相机内参：

```bash
rostopic echo /left_camera/color/camera_info -n 1
```

查看 TF：

```bash
rosrun tf view_frames
```

## 11. 常见问题

### 11.1 `source /home/catkin_ws/devel/setup.bash` 找不到

说明工作空间还没有完整编译成功，或者 devel 空间不完整。

处理：

```bash
cd /home/catkin_ws
source /opt/ros/noetic/setup.bash
catkin build
source /home/catkin_ws/devel/setup.bash
```

如果仍然没有 `devel/setup.bash`，查看构建日志：

```bash
catkin build realsense2_camera --verbose
```

### 11.2 `rospack find realsense2_camera` 找不到包

通常是没有 source 工作空间：

```bash
source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash
rospack profile
rospack find realsense2_camera
```

也检查源码是否存在：

```bash
ls /home/catkin_ws/src/realsense-ros-ros1-legacy/realsense2_camera/package.xml
```

### 11.3 `rs-enumerate-devices` 看不到相机

按顺序检查：

1. 重新插拔 USB
2. 换 USB3 口
3. 换 USB3 数据线
4. 不要接无供电 USB hub
5. 重新加载 udev 规则

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

再检查：

```bash
rs-enumerate-devices
```

### 11.4 ROS 能启动但没有图像

检查 topic 是否存在：

```bash
rostopic list | grep image_raw
```

检查图像频率：

```bash
rostopic hz /left_camera/color/image_raw
```

如果 topic 存在但 `rqt_image_view` 没画面，确认选择的是 raw topic：

```bash
/left_camera/color/image_raw
```

不要优先选 compressed topic，先用 raw topic 排查。

### 11.5 多相机启动失败

常见原因：

- 三台相机序列号和真实设备不一致
- 多台相机接在同一个 USB 控制器上，带宽不够
- 某台相机被其他程序占用
- USB 线或 hub 不稳定

建议先单独启动每台相机：

```bash
roslaunch realsense2_camera rs_camera.launch serial_no:=247122071929 camera:=left_camera tf_prefix:=left_camera enable_depth:=false
roslaunch realsense2_camera rs_camera.launch serial_no:=317622070449 camera:=right_camera tf_prefix:=right_camera enable_depth:=false
roslaunch realsense2_camera rs_camera.launch serial_no:=243622070764 camera:=head_camera tf_prefix:=head_camera enable_depth:=false
```

确认三台都能单独启动后，再启动：

```bash
roslaunch realsense2_camera three_realsense.launch
```

### 11.6 图像频率低或卡顿

优先降低分辨率或帧率。当前 launch 是：

```xml
<arg name="color_width"  value="640"/>
<arg name="color_height" value="480"/>
<arg name="color_fps"    value="30"/>
```

可临时改为：

```xml
<arg name="color_width"  value="640"/>
<arg name="color_height" value="480"/>
<arg name="color_fps"    value="15"/>
```

如果开启了深度，也可以先关闭：

```xml
<arg name="enable_depth" value="false"/>
```

### 11.7 `librealsense2` 相关 CMake 报错

如果编译时报：

```bash
Could not find librealsense2
```

安装开发包：

```bash
sudo apt install -y librealsense2-dev
```

然后重新编译：

```bash
cd /home/catkin_ws
source /opt/ros/noetic/setup.bash
catkin clean realsense2_camera
catkin build realsense2_camera
```

## 12. 推荐启动流程

首次部署时按这个顺序走：

```bash
source /opt/ros/noetic/setup.bash
cd /home/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source /home/catkin_ws/devel/setup.bash
rs-enumerate-devices
roslaunch realsense2_camera three_realsense.launch
```

日常使用时：

```bash
source /opt/ros/noetic/setup.bash
source /home/catkin_ws/devel/setup.bash
roslaunch realsense2_camera three_realsense.launch
```

如果要同时启动夹爪和相机：

```bash
cd /home/catkin_ws
./start_robot.sh
```

## 13. 和 XRoboToolkit teleop 联动

RealSense 相机启动后，teleop 程序可以读取对应 ROS 图像 topic。

三路彩色图像建议使用：

```bash
/left_camera/color/image_raw
/right_camera/color/image_raw
/head_camera/color/image_raw
```

如果 teleop 程序需要压缩图像，可以检查是否有：

```bash
/left_camera/color/image_raw/compressed
/right_camera/color/image_raw/compressed
/head_camera/color/image_raw/compressed
```

启动顺序建议：

1. 启动 ROS 和相机
2. 确认 `rostopic hz` 有稳定图像
3. 再启动 XRoboToolkit teleop

示例：

```bash
cd /home/catkin_ws
./start_robot.sh
```

另开终端启动 teleop：

```bash
cd /home/XRoboToolkit-Teleop-Sample-Python
./run_teleop.sh
```

如果 teleop 中需要修改相机 topic，请优先使用上面列出的三个 namespace，避免和默认 `/camera/...` topic 混淆。
