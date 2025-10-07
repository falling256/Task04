'''
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    # 获取完整的 ROS2 环境
    ros_env = dict(os.environ)  # 复制当前环境
    
    # 修复库路径 - 移除 MVS SDK，保留 ROS2
    safe_ld_path = ':'.join([
        '/opt/ros/humble/opt/rviz_ogre_vendor/lib',
        '/opt/ros/humble/lib',
        '/opt/ros/humble/lib/x86_64-linux-gnu', 
        '/usr/lib/x86_64-linux-gnu'
    ])
    
    # 更新环境
    rviz_env = ros_env.copy()
    rviz_env['LD_LIBRARY_PATH'] = safe_ld_path
    rviz_env['QT_QPA_PLATFORM'] = 'xcb'
    
    # 确保必要的 ROS2 环境变量存在
    essential_ros_vars = [
        'AMENT_PREFIX_PATH', 'ROS_DISTRO', 'ROS_VERSION',
        'ROS_LOCALHOST_ONLY', 'PYTHONPATH', 'ROS_PYTHON_VERSION'
    ]
    
    for var in essential_ros_vars:
        if var not in rviz_env and var in os.environ:
            rviz_env[var] = os.environ[var]

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
    
    # RViz2 节点 - 使用完整的 ROS2 环境
    rviz_node = ExecuteProcess(
        cmd=['/opt/ros/humble/bin/rviz2'],
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
    rviz_node = ExecuteProcess(
    cmd=['rviz2', '-d', PathJoinSubstitution([
        FindPackageShare('hikvision_camera'), 
        'config', 
        'camera_display.rviz'  # 你的配置文件名
    ])],
    output='screen',
    env=safe_env,  # 使用你之前设置的安全环境变量
    name='rviz2'
)
'''
