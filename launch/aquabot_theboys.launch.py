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

    hub_node = Node(
        package="aquabot_theboys",
        executable="hub",
        name="hub"
    )

    object_detection_node = Node(
        package="aquabot_theboys",
        executable="object_detection_node",
        name="object_detection_node"
    )

    ld.add_action(sensors_node)
    ld.add_action(cmd_motors_node)
    ld.add_action(filters_node)
    ld.add_action(hub_node)
    ld.add_action(object_detection_node)

    return ld