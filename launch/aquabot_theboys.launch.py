from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sensors_node = Node(
        package="aquabot_theboys",
        executable="sensors",
        name="sensors",
    )

    cmd_motors_node = Node(
        package="aquabot_theboys",
        executable="cmd_motors",
        name="cmd_motors"
    )

    filters_node = Node(
        package="aquabot_theboys",
        executable="filters",
        name="filters"
    )

    ld.add_action(sensors_node)
    ld.add_action(cmd_motors_node)
    ld.add_action(filters_node)

    return ld