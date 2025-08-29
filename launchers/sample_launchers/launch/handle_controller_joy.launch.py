import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "joy"
    NODE_NAME = "joy_node"

    ROS_PARAM_CONFIG = (
        osp.join(get_package_share_directory("sample_launchers"), "config", "handle_controller_joy.yaml"),
    )
    handle_controller_joy = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_control/handle_controller",
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("joy", "/aiformula_control/handle_controller/joy"),
        ],
    )

    return LaunchDescription([
        handle_controller_joy,
    ])
