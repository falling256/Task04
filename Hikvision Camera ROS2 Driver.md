# Hikvision Camera ROS2 Driver

基于海康威视MVS SDK的ROS2相机驱动包，提供稳定、易用的海康相机ROS2接口。

## 代码结构

```
hikvision_camera/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── camera.launch.py
├── src/
│   └── hikvision_camera_node.cpp
└── README.md
```

## 功能特性

-  **自动设备发现** - 支持自动、序列号、IP三种连接模式
-  **稳定图像采集** - 实时图像采集与发布，支持BGR8格式
-  **动态参数配置** - 实时调整曝光、增益、帧率等相机参数
-  **断线自动重连** - 完善的连接异常处理机制
-  **ROS2标准接口** - 符合ROS2最佳实践的消息和服务接口

## 快速开始

### 系统要求

- Ubuntu 22.04
- ROS2 Humble
- 海康威视工业相机（支持GigE或USB3.0接口）

### 安装依赖

```bash
# 安装ROS2基础依赖
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-std-msgs

# 安装可视化工具（可选）
sudo apt install ros-humble-rviz2 ros-humble-rqt-image-view
```

### 一键安装所有依赖

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

### 编译

```bash
cd ~/ros2_ws
colcon build --packages-select hikvision_camera
source install/setup.bash
```

## 使用方法

### 基本使用

```bash
# 自动发现并连接第一个可用相机
ros2 launch hikvision_camera camera.launch.py

# 或者直接运行节点
ros2 run hikvision_camera hikvision_camera_node
```

### 高级连接模式

```bash
# 按序列号连接特定相机
ros2 launch hikvision_camera camera.launch.py mode:='by_serial' serial:='相机序列号'

# 按IP地址连接相机
ros2 launch hikvision_camera camera.launch.py mode:='by_ip' ip:='192.168.1.100'
```

### 查看图像

```bash
# 在RViz2中查看
rviz2
# 添加Image显示，选择 /camera/image_raw 话题
```

## 参数配置

### 连接参数

| 参数                 | 类型   | 默认值                | 说明                                 |
| -------------------- | ------ | --------------------- | ------------------------------------ |
| `mode`               | string | `"auto"`              | 连接模式: `auto`/`by_serial`/`by_ip` |
| `serial`             | string | `""`                  | 相机序列号（by_serial模式使用）      |
| `ip`                 | string | `""`                  | 相机IP地址（by_ip模式使用）          |
| `topic`              | string | `"/camera/image_raw"` | 图像发布话题                         |
| `timeout_ms`         | int    | `1000`                | 采集超时时间(ms)                     |
| `reconnect_delay_ms` | int    | `2000`                | 重连延迟时间(ms)                     |

### 相机参数

| 参数            | 类型   | 默认值    | 说明         |
| --------------- | ------ | --------- | ------------ |
| `exposure_time` | double | `10000.0` | 曝光时间(μs) |
| `auto_exposure` | bool   | `true`    | 自动曝光开关 |
| `gain`          | double | `0.0`     | 增益值(dB)   |
| `auto_gain`     | bool   | `true`    | 自动增益开关 |
| `frame_rate`    | double | `30.0`    | 采集帧率(Hz) |
| `pixel_format`  | string | `"BGR8"`  | 图像像素格式 |

### 动态参数调整

```bash
# 设置曝光时间
ros2 param set /hikvision_camera exposure_time 20000.0

# 设置增益
ros2 param set /hikvision_camera gain 5.0

# 关闭自动曝光
ros2 param set /hikvision_camera auto_exposure false

# 查看当前参数
ros2 param get /hikvision_camera exposure_time
```

## 话题与服务

### 发布的话题

- `/camera/image_raw` (`sensor_msgs/msg/Image`) - 相机图像数据
- `/parameter_events` (`rcl_interfaces/msg/ParameterEvent`) - 参数事件
- `/rosout` (`rcl_interfaces/msg/Log`) - 日志输出

### 服务

- `/hikvision_camera/describe_parameters` - 参数描述服务
- `/hikvision_camera/get_parameters` - 获取参数服务
- `/hikvision_camera/set_parameters` - 设置参数服务
- 其他标准ROS2参数服务

## 故障排除

### 遇到的问题

**1. 相机连接失败**
```bash
# 检查相机设备
ros2 run hikvision_camera hikvision_camera_node --ros-args -p mode:="auto"
```

**2. RViz2显示问题**
```bash
# 如果遇到Qt库冲突，使用修复版本
alias rviz2='LD_LIBRARY_PATH="/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu" /opt/ros/humble/lib/rviz2/rviz2'
```

**3. 图像显示黑屏**
- 检查Fixed Frame设置为 `camera`
- 确认相机镜头盖已打开
- 调整曝光和增益参数

**4. 参数设置失败**
- 先关闭自动模式：`ros2 param set /hikvision_camera auto_exposure false`
- 然后设置具体参数值

