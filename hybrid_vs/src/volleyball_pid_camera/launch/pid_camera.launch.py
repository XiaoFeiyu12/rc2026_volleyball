import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 设置 OpenVINO 运行时库路径
    openvino_lib_path = '/home/rqxx/.local/lib/python3.10/site-packages/openvino/libs'
    current_ld = os.environ.get('LD_LIBRARY_PATH', '')
    os.environ['LD_LIBRARY_PATH'] = f'{openvino_lib_path}:{current_ld}' if current_ld else openvino_lib_path

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw',
                              description='USB camera image topic'),
        DeclareLaunchArgument('debug', default_value='true',
                              description='Enable debug image publishing'),

        # usb_cam 驱动节点（默认发布到 image_raw，重映射到 /usb_cam/image_raw）
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            remappings=[('image_raw', '/usb_cam/image_raw')],
            parameters=[{
                'video_device': '/dev/video1',
            }],
        ),

        # PID 像素偏移检测节点
        Node(
            package='volleyball_pid_camera',
            executable='pid_camera_node',
            name='pid_camera_node',
            output='screen',
            parameters=[{
                'usb_cam_topic': LaunchConfiguration('camera_topic'),
                'confidence_threshold': 0.7,
                'NMS_threshold': 0.5,
                'debug': LaunchConfiguration('debug'),
            }],
            emulate_tty=True,
        ),
    ])
