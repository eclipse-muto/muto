from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    talker_config = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "talker.yaml",
        )
    )
    node_talker = Node(
        name="talker",
        package="muto_talker",
        executable="muto_talker",
        output="screen",
        parameters=[talker_config],
    )

    node_listener = Node(
        name="listener",
        package="muto_listener",
        executable="muto_listener",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(node_talker)
    ld.add_action(node_listener)
    return ld
