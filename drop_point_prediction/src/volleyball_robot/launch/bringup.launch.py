import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command


sys.path.append(os.path.join(get_package_share_directory("volleyball_robot"), 'launch'))
from get_params import *

# 相机参数文件路径（realsense + usb_camera 共用）
camera_params_path = os.path.join(
    get_package_share_directory("volleyball_robot"),
    "config",
    "camera_params.yaml"
)

def get_realsense_camera_launch():
    # realsense相机启动
    with open(camera_params_path, "r") as f:
        realsense_params = yaml.safe_load(f)

    launch_args = {}
    for k, v in  realsense_params['realsense'].items():
        if k == 'json_file_path':
            if v and str(v).strip():
                json_path = str(v)
            if not os.path.isabs(json_path):
                json_path = os.path.join(
                    get_package_share_directory('volleyball_robot'),
                    json_path
                )
            launch_args[k] = json_path
        else:
            launch_args[k] = str(v)

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py"
            )
        ),
        launch_arguments = launch_args.items()
    )
    return realsense_launch

def generate_launch_description():
    ld = LaunchDescription()
    
    # ============================================================
    # Realsense 相机
    # ============================================================
    if is_to_launch_params['realsense_camera']:
        ld.add_action(get_realsense_camera_launch())

    # ============================================================
    # 机器人描述（URDF + joint_state_publisher + robot_state_publisher）
    # ============================================================
    if is_to_launch_params['robot_description']:
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            arguments=[urdf_model_path],
        )
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{'robot_description': get_robot_description(robot_description_params, urdf_model_path),
                            'publish_frequency': 1000.0}],
        )
        ld.add_action(joint_state_publisher)
        ld.add_action(robot_state_publisher)

    # ============================================================
    # 模拟里程计（无下位机时使用，发布静态 odom → base_link）
    # ============================================================
    if is_to_launch_params.get('mock_odom', False):
        mock_odom_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mock_odom_to_base_link',
            output='screen',
            arguments=['0', '0', '0', '1.5707963267', '0', '0', 'odom', 'base_link'],
        )
        ld.add_action(mock_odom_node)

    # ============================================================
    # 串口驱动（odom → base_link TF + 下位机通信）
    # ============================================================
    if is_to_launch_params['serial_driver_node']:
        serial_driver_node = Node(
            package='volleyball_serial_driver',
            executable='volleyball_serial_driver_node',
            name='serial_driver_node',
            output='screen',
            parameters=[node_params],
            ros_arguments=["--ros-args", "--log-level", "serial_driver_node:=" + info_level_params['serial_driver_node']],
        )
        ld.add_action(serial_driver_node)

    # ============================================================
    # 检测节点（YOLOv11n）—— 延迟 0.5s 等待相机初始化
    # ============================================================
    if is_to_launch_params['detector_node']:
        detector_node = Node(
            executable = "volleyball_detect",
            package= "volleyball_detect",
            output = "screen",
            name = "detector_node",
            parameters = [node_params],
            ros_arguments=["--ros-args", "--log-level", "detector_node:=" + info_level_params["detector_node"]]
        )
        ld.add_action(TimerAction(period=0.5, actions=[detector_node]))
        
    # ============================================================
    # 追踪节点（EKF 滤波）—— 延迟 1.0s
    # ============================================================
    if is_to_launch_params['tracker_node']:
        tracker_node = Node(
            package='volleyball_track',
            executable='volleyball_track',
            output='screen',
            name='tracker_node',
            parameters=[node_params],
            ros_arguments=["--ros-args", "--log-level", "tracker_node:=" + info_level_params['tracker_node']],
        )
        ld.add_action(TimerAction(period=1.0, actions=[tracker_node]))

    # ============================================================
    # 预测节点—— 延迟 1.5s
    # ============================================================
    if is_to_launch_params['predict_node']:
        predict_node = Node(
            package='volleyball_predict',
            executable='volleyball_predict',
            output='screen',
            name='predict_node',
            parameters=[node_params],
            ros_arguments=["--ros-args", "--log-level", "predict_node:=" + info_level_params['predict_node']],
        )
        ld.add_action(TimerAction(period=1.5, actions=[predict_node]))

    # ============================================================
    # 规划节点—— 延迟 2.0s
    # ============================================================
    if is_to_launch_params['planner_node']:
        planner_node = Node(
            package='volleyball_plan',
            executable='volleyball_plan',
            output='screen',
            name='planner_node',
            parameters=[node_params],
            ros_arguments=["--ros-args", "--log-level", "planner_node:=" + info_level_params['planner_node']],
        )
        ld.add_action(TimerAction(period=2.0, actions=[planner_node]))

    # ============================================================
    # Foxglove 桥接
    # ============================================================
    if is_to_launch_params['foxglove_node']:
        foxglove_node = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': is_to_launch_params['foxglove_port'],
                'send_buffer_limit': 10000000,
                'use_compression': True,
            }],
        )
        ld.add_action(foxglove_node)

    # ============================================================
    # RViz2
    # ============================================================
    if is_to_launch_params['rviz']:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('volleyball_robot'),
                'rviz',
                'volleyball_robot.rviz'
            )],
        )
        ld.add_action(rviz_node)

    return ld
