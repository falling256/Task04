# Hikvision Camera ROS2 Driver

## 代码结构

```
hikvision_camera/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── auto_image.rviz
├── launch/
│   └── simple_auto.launch.py
├── src/
│   └── hikvision_camera_node.cpp
└── README.md
```

## 功能特性

- **稳定图像采集** - 实时图像采集与发布，支持多种图像格式
- **动态参数配置** - 实时调整曝光、增益、帧率、图像格式等相机参数
- **断线自动重连** - 完善的连接异常处理和健康检查机制
- **ROS2标准接口** - 符合ROS2最佳实践的消息和参数接口
- **格式转换支持** - 支持BGR8、MONO8等图像格式

## 快速开始

### 安装依赖

```zsh
# 安装ROS2基础依赖
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-std-msgs

```

### 一键安装所有依赖

```zsh
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

### 编译

```zsh
cd ~/ros2_ws
colcon build --packages-select hikvision_camera
source install/setup.zsh
```

## 使用方法

### 基本使用

```zsh
# 自动发现并连接第一个可用相机
ros2 launch hikvision_camera simple_auto.launch.py

# 在RViz2中查看图像（使用预置配置）
rviz2 -d install/hikvision_camera/share/hikvision_camera/config/auto_image.rviz
```

### 实时图像查看

```zsh

# 使用RViz2查看
rviz2
# 然后添加Image显示，选择 /image_raw 话题，Fixed Frame设为camera
```

## 参数配置

### 连接参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mode` | string | `"auto"` | 连接模式: `auto`/`by_serial`/`by_ip` |
| `serial` | string | `""` | 相机序列号（by_serial模式使用） |
| `ip` | string | `""` | 相机IP地址（by_ip模式使用） |
| `topic` | string | `"/image_raw"` | 图像发布话题 |
| `timeout_ms` | int | `1000` | 采集超时时间(ms) |
| `reconnect_delay_ms` | int | `2000` | 重连延迟时间(ms) |

### 相机控制参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `exposure_time` | double | `10000.0` | 曝光时间(μs)，范围: 10.0-1000000.0 |
| `auto_exposure` | bool | `true` | 自动曝光开关 |
| `gain` | double | `0.0` | 增益值(dB)，范围: 0.0-30.0 |
| `auto_gain` | bool | `true` | 自动增益开关 |
| `frame_rate` | double | `30.0` | 采集帧率(Hz)，受相机硬件限制 |
| `pixel_format` | string | `"BGR8"` | 图像像素格式: `BGR8`/`RGB8`/`MONO8`/`MONO16` |

### 动态参数调整示例

```zsh
# 设置曝光时间（先关闭自动曝光）
ros2 param set /hikvision_camera auto_exposure false
ros2 param set /hikvision_camera exposure_time 20000.0

# 设置增益（先关闭自动增益）
ros2 param set /hikvision_camera auto_gain false
ros2 param set /hikvision_camera gain 5.0

# 设置图像格式
ros2 param set /hikvision_camera pixel_format 'MONO8'

# 设置帧率
ros2 param set /hikvision_camera frame_rate 25.0

# 查看当前参数
ros2 param get /hikvision_camera exposure_time
ros2 param get /hikvision_camera pixel_format
```

### 预设配置模式

```zsh
# 高质量模式（图像质量优先）
ros2 param set /hikvision_camera pixel_format 'BGR8'
ros2 param set /hikvision_camera frame_rate 15.0
ros2 param set /hikvision_camera auto_exposure true

# 高速模式（性能优先）
ros2 param set /hikvision_camera pixel_format 'MONO8'
ros2 param set /hikvision_camera frame_rate 30.0
ros2 param set /hikvision_camera auto_exposure false
ros2 param set /hikvision_camera exposure_time 5000.0

# 低光模式
ros2 param set /hikvision_camera pixel_format 'MONO8'
ros2 param set /hikvision_camera frame_rate 10.0
ros2 param set /hikvision_camera auto_exposure false
ros2 param set /hikvision_camera exposure_time 50000.0
ros2 param set /hikvision_camera gain 20.0
```

## 话题与服务

### 发布的话题

- `/image_raw` (`sensor_msgs/msg/Image`) - 相机图像数据流
- `/parameter_events` (`rcl_interfaces/msg/ParameterEvent`) - 参数变更事件
- `/rosout` (`rcl_interfaces/msg/Log`) - 系统日志输出

### 参数服务

- `/hikvision_camera/describe_parameters` - 参数描述服务
- `/hikvision_camera/get_parameters` - 获取参数服务  
- `/hikvision_camera/set_parameters` - 设置参数服务
- `/hikvision_camera/list_parameters` - 参数列表服务

## 故障排除

### 常见问题

**1. 图像显示黑屏**
- 调整曝光参数：`ros2 param set /hikvision_camera exposure_time 50000.0`
- 调整增益参数：`ros2 param set /hikvision_camera gain 15.0`

**2. 帧率达不到设定值**
- 这是相机硬件限制，大多数相机在最高分辨率下帧率有限
- 尝试使用MONO8格式：`ros2 param set /hikvision_camera pixel_format 'MONO8'`
- 减少曝光时间：`ros2 param set /hikvision_camera exposure_time 5000.0`

**3. 参数设置失败**
- 确保先关闭自动模式：
  ```zsh
  ros2 param set /hikvision_camera auto_exposure false
  ros2 param set /hikvision_camera auto_gain false
  ```
- 然后设置具体参数值


### 诊断命令

```zsh
# 检查节点状态
ros2 node list
ros2 node info /hikvision_camera

# 检查话题数据
ros2 topic list
ros2 topic hz /image_raw
ros2 topic echo /image_raw --once | head -20

# 监控参数变化
ros2 param monitor /hikvision_camera
```

