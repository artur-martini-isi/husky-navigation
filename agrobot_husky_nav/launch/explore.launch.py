"""Exploração autônoma por fronteiras. Não inicia sozinho: precisa de start.

Requer `indoor.launch.py` (scan) e `slam.launch.py` (mapa e frame `map`) no ar.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
        [FindPackageShare('agrobot_husky_nav'), 'config', 'explore.yaml'])),
    DeclareLaunchArgument('max_linear', default_value='0.3'),
]


def generate_launch_description():
    return LaunchDescription(ARGS + [Node(
        package='agrobot_husky_nav', executable='explore', name='explore',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[LaunchConfiguration('params'),
                    {'max_linear': LaunchConfiguration('max_linear')}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )])
