import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.substitutions import Command

def get_robot_description(robot_description_params, urdf_model_path):
    camera_config = robot_description_params['camera']
    delta_arm_config = robot_description_params['delta_arm']
    strike_point_config = robot_description_params['strike_point']

    # 读取 URDF 内容作为 robot_description 参数
    robot_description = Command(['xacro ', urdf_model_path,
                                 " delta_arm_xyz:=", delta_arm_config["xyz"],
                                 " delta_arm_rpy:=", delta_arm_config['rpy'],
                                 " cam_xyz:=", camera_config['xyz'],
                                 " cam_rpy:=", camera_config['rpy'],
                                 " strike_point_xyz:=", strike_point_config['xyz'],
                                 " strike_point_rpy:=", strike_point_config['rpy']
                                 ])
    return robot_description

#-----------------------------------------------------------------------------------
urdf_model_path = os.path.join(
    get_package_share_directory('volleyball_robot_description'),
    'urdf',
    'volleyball_robot_description.urdf.xacro'
)
launch_params_path = os.path.join(
    get_package_share_directory("volleyball_robot"),
    'config',
    'launch_params.yaml'
)
node_params = os.path.join(
    get_package_share_directory("volleyball_robot"),
    'config',
    'node_params.yaml'
)


with open(launch_params_path, 'r') as launch_params_f :
    launch_params = yaml.safe_load(launch_params_f)

is_to_launch_params = launch_params['is_to_launch']
info_level_params = launch_params['info_level']
robot_description_params = launch_params['robot_description']
