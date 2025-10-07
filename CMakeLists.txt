cmake_minimum_required(VERSION 3.8)
project(hikvision_camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)  # 确保这一行存在
find_package(sensor_msgs REQUIRED)


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

find_package(OpenCV REQUIRED)

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


install(TARGETS
  hikvision_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  std_msgs       
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
