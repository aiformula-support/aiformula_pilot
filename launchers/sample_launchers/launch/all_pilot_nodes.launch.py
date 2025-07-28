import os.path as osp
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Command input from handle controller --- #
    handle_controller_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(
                get_package_share_directory("sample_launchers"),
                "launch/handle_controller_joy.launch.py",
            ),
        ),
    )
    # --- Output velocity and angular velocity from handle controller --- #
    handle_controller_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(
                get_package_share_directory("teleop_twist_handle_controller"),
                "launch/handle_controller_teleop.launch.py",
            ),
        ),
    )
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(
                get_package_share_directory("sample_launchers"),
                "launch/twist_mux.launch.py",
            ),
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_runtime_monitor": "false",
        }.items(),
    )

    gui_webserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(
                get_package_share_directory("gui_webserver"),
                "launch/gui_webserver.launch.py",
            ),
        ),
    )

    return LaunchDescription(
        [
            handle_controller_joy,
            handle_controller_teleop,
            twist_mux,
            gui_webserver,
        ]
    )
