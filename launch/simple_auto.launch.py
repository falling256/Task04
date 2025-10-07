from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    # 获取完整的 ROS2 环境
    ros_env = dict(os.environ)
    
    # 为 RViz2 创建安全环境
    safe_ld_path = ':'.join([
        '/opt/ros/humble/opt/rviz_ogre_vendor/lib',
        '/opt/ros/humble/lib',
        '/opt/ros/humble/lib/x86_64-linux-gnu', 
        '/usr/lib/x86_64-linux-gnu'
    ])
    
    rviz_env = ros_env.copy()
    rviz_env['LD_LIBRARY_PATH'] = safe_ld_path
    
    # 海康相机节点
    camera_node = Node(
        package='hikvision_camera',
        executable='hikvision_camera_node',
        name='hikvision_camera',
        parameters=[{
            'mode': 'auto',
            'topic': '/camera/image_raw',
            'timeout_ms': 1000,
            'reconnect_delay_ms': 2000,
            'exposure_time': 10000.0,
            'auto_exposure': True,
            'gain': 0.0,
            'auto_gain': True,
            'frame_rate': 30.0,
            'pixel_format': 'BGR8',
            'save_path': './camera_captures',
            'auto_capture_on_param_change': True
        }],
        output='screen',
        emulate_tty=True
    )
    
    # 帧率监控节点
    frame_monitor_node = Node(
        package='hikvision_camera',
        executable='frame_rate_monitor',
        name='frame_rate_monitor',
        parameters=[{
            'target_frame_rate': 30.0
        }],
        output='screen',
        emulate_tty=True
    )
    
    # 使用您保存的配置文件
    rviz_node = ExecuteProcess(
        cmd=['/opt/ros/humble/bin/rviz2', '-d', '/home/shulin/ros2_ws/src/hikvision_camera/config/auto_image.rviz'],
        output='screen',
        env=rviz_env,
        name='rviz2'
    )
    
    # 延迟启动 RViz2
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    return LaunchDescription([
        camera_node,
        frame_monitor_node,
        delayed_rviz
    ])
