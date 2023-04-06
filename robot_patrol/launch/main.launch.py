from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    service_node = Node(
        package="robot_patrol",
        executable="direction_service_node",
    )
    patrol_node = Node(
        package="robot_patrol",
        executable="patrolv2_node"
    )
    ld.add_action(service_node)
    ld.add_action(patrol_node)
    return ld