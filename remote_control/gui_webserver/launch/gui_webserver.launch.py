import os.path as osp
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "gui_webserver"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    WEB_SERVER_SHELL = (
        osp.join(PACKAGE_DIR, "shellscripts", "start_ros_server.sh"),
    )

    rosbridge_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            osp.join(
                get_package_share_directory("gui_webserver"),
                "launch/rosbridge_websocket.launch.py",
            ),
        ),
    )

    return LaunchDescription([
            ExecuteProcess(cmd=["bash", WEB_SERVER_SHELL], shell=True),
            rosbridge_server,
    ])
