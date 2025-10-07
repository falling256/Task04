cmake_minimum_required(VERSION 3.8)
project(hikvision_camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add flags for warnings in GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------
# Find ROS dependencies
# ----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ----------------------------
# SDK include & lib dirs
# ----------------------------
# HCNetSDK (网络摄像头 SDK)
set(HCNetsdk_INCLUDE_DIR /home/shulin/hikvision_sdk/CH-HCNetSDKV6.1.9.48_build20230410_linux64/include)
set(HCNetsdk_LIB_DIR /home/shulin/hikvision_sdk/CH-HCNetSDKV6.1.9.48_build20230410_linux64/lib)

# MVS SDK (工业相机 SDK)
set(MVS_INCLUDE_DIR /opt/MVS/include)
set(MVS_LIB_DIR /opt/MVS/lib/64)

# Include dirs
include_directories(
  ${HCNetsdk_INCLUDE_DIR}
  ${MVS_INCLUDE_DIR}
)

link_directories(
  ${HCNetsdk_LIB_DIR}
  ${MVS_LIB_DIR}
)

# ----------------------------
# 查找OpenCV
# ----------------------------
find_package(OpenCV REQUIRED)

# ----------------------------
# 主相机节点 (使用MVS SDK)
# ----------------------------
add_executable(hikvision_camera_node src/hikvision_camera_node.cpp)

ament_target_dependencies(hikvision_camera_node
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(hikvision_camera_node
  MvCameraControl
  ${OpenCV_LIBS}
  pthread
)

# ----------------------------
# 帧率监控节点 (新增 - 用于验收演示)
# ----------------------------
add_executable(frame_rate_monitor src/frame_rate_monitor.cpp)

ament_target_dependencies(frame_rate_monitor
  rclcpp
  sensor_msgs
  std_msgs
)

# 帧率监控节点不需要链接MVS SDK库，只使用ROS2标准库

# ----------------------------
# 设备发现节点 (可选)
# ----------------------------
# add_executable(device_discovery_node src/device_discovery_node.cpp)
# ament_target_dependencies(device_discovery_node rclcpp)
# target_link_libraries(device_discovery_node MvCameraControl pthread)

# ----------------------------
# Install executables
# ----------------------------
install(TARGETS
  hikvision_camera_node
  frame_rate_monitor  # 新增
  # device_discovery_node
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------
# Install launch files
# ----------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------
# Install config files (新增这一行)
# ----------------------------
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------
# Export dependencies
# ----------------------------
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
)

# ----------------------------
# Lint & Test
# ----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
