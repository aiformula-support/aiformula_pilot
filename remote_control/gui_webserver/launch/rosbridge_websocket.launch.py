from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('port', default_value='9090', description='Port number'),
        DeclareLaunchArgument('address', default_value='', description='Address'),
        DeclareLaunchArgument('ssl', default_value='false', description='Use SSL'),
        DeclareLaunchArgument('certfile', default_value='', description='SSL certificate file'),
        DeclareLaunchArgument('keyfile', default_value='', description='SSL key file'),
        DeclareLaunchArgument('retry_startup_delay', default_value='5.0', description='Retry startup delay'),
        DeclareLaunchArgument('fragment_timeout', default_value='600', description='Fragment timeout'),
        DeclareLaunchArgument('delay_between_messages', default_value='0', description='Delay between messages'),
        DeclareLaunchArgument('max_message_size', default_value='10000000', description='Max message size'),
        DeclareLaunchArgument('unregister_timeout', default_value='10.0', description='Unregister timeout'),
        DeclareLaunchArgument('use_compression', default_value='false', description='Use compression'),
        DeclareLaunchArgument('call_services_in_new_thread', default_value='false', description='Call services in new thread'),
        DeclareLaunchArgument('send_action_goals_in_new_thread', default_value='false', description='Send action goals in new thread'),
        DeclareLaunchArgument('topics_glob', default_value='', description='Topics glob pattern'),
        DeclareLaunchArgument('services_glob', default_value='', description='Services glob pattern'),
        DeclareLaunchArgument('params_glob', default_value='', description='Params glob pattern'),
        DeclareLaunchArgument('bson_only_mode', default_value='false', description='BSON only mode'),
        DeclareLaunchArgument('binary_encoder', default_value='default', condition=UnlessCondition(LaunchConfiguration('bson_only_mode')), description='Binary encoder'),
    ]

    ssl_group = GroupAction([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[
                {'certfile': LaunchConfiguration('certfile')},
                {'keyfile': LaunchConfiguration('keyfile')},
                {'port': LaunchConfiguration('port')},
                {'address': LaunchConfiguration('address')},
                {'retry_startup_delay': LaunchConfiguration('retry_startup_delay')},
                {'fragment_timeout': LaunchConfiguration('fragment_timeout')},
                {'delay_between_messages': LaunchConfiguration('delay_between_messages')},
                {'max_message_size': LaunchConfiguration('max_message_size')},
                {'unregister_timeout': LaunchConfiguration('unregister_timeout')},
                {'use_compression': LaunchConfiguration('use_compression')},
                {'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread')},
                {'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread')},
                {'topics_glob': LaunchConfiguration('topics_glob')},
                {'services_glob': LaunchConfiguration('services_glob')},
                {'params_glob': LaunchConfiguration('params_glob')},
            ],
            condition=IfCondition(LaunchConfiguration('ssl')),
        ),
    ])

    non_ssl_group = GroupAction([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
                {'address': LaunchConfiguration('address')},
                {'retry_startup_delay': LaunchConfiguration('retry_startup_delay')},
                {'fragment_timeout': LaunchConfiguration('fragment_timeout')},
                {'delay_between_messages': LaunchConfiguration('delay_between_messages')},
                {'max_message_size': LaunchConfiguration('max_message_size')},
                {'unregister_timeout': LaunchConfiguration('unregister_timeout')},
                {'use_compression': LaunchConfiguration('use_compression')},
                {'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread')},
                {'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread')},
                {'topics_glob': LaunchConfiguration('topics_glob')},
                {'services_glob': LaunchConfiguration('services_glob')},
                {'params_glob': LaunchConfiguration('params_glob')},
                {'bson_only_mode': LaunchConfiguration('bson_only_mode')},
            ],
            remappings=[
                ('compressed_img', "/aiformula_visualization/zed/left_image/compressed"),
                ('twist_mux/cmd_vel', "/aiformula_control/twist_mux/cmd_vel"),
                ('vectornav/gnss', "/aiformula_sensing/vectornav/gnss"),
                ('twist_mux_gamepad/lock', "/aiformula_control/twist_mux/gamepad/lock"),
                ('handle_controller/joy', "/aiformula_control/handle_controller/joy"),
                ('odom', "/aiformula_sensing/gyro_odometry_publisher/odom"),
            ],
            condition=UnlessCondition(LaunchConfiguration('ssl')),
        ),
    ])

    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        parameters=[
            {'topics_glob': LaunchConfiguration('topics_glob')},
            {'services_glob': LaunchConfiguration('services_glob')},
            {'params_glob': LaunchConfiguration('params_glob')},
        ],
    )

    return LaunchDescription([
        *launch_args,
        ssl_group,
        non_ssl_group,
        rosapi_node,
    ])
