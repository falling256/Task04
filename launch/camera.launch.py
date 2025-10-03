from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hikvision_camera',
            executable='hikvision_camera_node',
            name='hikvision_camera',
            parameters=[{
                # 连接模式：auto(自动选择第一个), by_serial(按序列号), by_ip(按IP)
                'mode': 'auto',
                
                # 当mode为by_serial时使用
                'serial': '',
                
                # 当mode为by_ip时使用  
                'ip': '',
                
                # 图像话题
                'topic': '/camera/image_raw',
                
                # 超时和重连设置
                'timeout_ms': 1000,
                'reconnect_delay_ms': 2000,
                
                # 相机参数 - 这些会自动应用到相机
                'exposure_time': 10000.0,
                'auto_exposure': True,
                'gain': 0.0,
                'auto_gain': True,
                'frame_rate': 30.0,
                'pixel_format': 'BGR8'
            }],
            output='screen',
            emulate_tty=True
        ),
    ])