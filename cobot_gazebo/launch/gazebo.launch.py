import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from pathlib import Path

def generate_launch_description():
    pkg_name = 'cobot_description'
    urdf_path = PathJoinSubstitution([FindPackageShare(pkg_name), 'urdf', 'main.xacro'])

    model_arg = DeclareLaunchArgument(name = "model", default_value=urdf_path,description="Absolute path to robot urdf file.")

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration("model")]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output='screen'
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[str(Path(get_package_share_directory(pkg_name)).parent.resolve())]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output='screen'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ])
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'cobot']
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        set_gazebo_model_path,
        gazebo_launch,
        spawn_entity
    ])
