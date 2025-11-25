from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():  

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    cobot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cobot_controller", "--controller-manager", "/controller_manager"],
    )


    return LaunchDescription([  
        joint_state_broadcaster,
        cobot_controller
    ])