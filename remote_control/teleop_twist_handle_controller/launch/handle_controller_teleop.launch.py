import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "teleop_twist_handle_controller"
    NODE_NAME = "teleop_node"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    launch_args = (
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logger level (debug, info, warn, error, fatal)",
        ),
    )

    ROS_PARAM_CONFIG = (
        osp.join(PACKAGE_DIR, "config", PACKAGE_NAME + ".yaml"),
    )
    handle_controller_teleop = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_control/handle_controller",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration('log_level')],
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("sub_joy", "/aiformula_control/handle_controller/joy"),
            ("pub_cmd_vel", "/aiformula_control/handle_controller/cmd_vel"),
            ("pub_cmd_vel_coasting", "/aiformula_control/handle_controller/cmd_vel/coasting"),
            ("pub_twist_mux_lock", "/aiformula_control/twist_mux/gamepad/lock"),
        ],
    )

    return LaunchDescription([
        *launch_args,
        handle_controller_teleop,
    ])
