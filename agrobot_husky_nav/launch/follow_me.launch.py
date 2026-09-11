"""Follow-me: ZED (marcador ArUco) + MID360 (segurança) -> cmd_vel. Não inicia sozinho."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
        [FindPackageShare('agrobot_husky_nav'), 'config', 'follow_me.yaml'])),
    DeclareLaunchArgument('target_distance', default_value='1.5'),
    DeclareLaunchArgument('max_linear', default_value='0.4'),
]


def generate_launch_description():
    return LaunchDescription(ARGS + [Node(
        package='agrobot_husky_nav', executable='follow_me', name='follow_me',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[LaunchConfiguration('params'),
                    {'target_distance': LaunchConfiguration('target_distance'),
                     'max_linear': LaunchConfiguration('max_linear')}],
    )])
