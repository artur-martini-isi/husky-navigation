"""Navegação até marcadores clicados no mapa.

Requer `indoor.launch.py` (scan do MID360) e `slam.launch.py` (mapa e frame `map`) no ar.
Ao contrário da exploração, este nó começa a andar assim que recebe um marcador
(`auto_start`), porque o comando partiu de uma pessoa.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
        [FindPackageShare('agrobot_husky_nav'), 'config', 'goto_point.yaml'])),
    DeclareLaunchArgument('max_linear', default_value='0.3'),
    DeclareLaunchArgument('goal_mode', default_value='append'),
]


def generate_launch_description():
    return LaunchDescription(ARGS + [Node(
        package='agrobot_husky_nav', executable='goto_point', name='goto_point',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[LaunchConfiguration('params'),
                    {'max_linear': LaunchConfiguration('max_linear'),
                     'goal_mode': LaunchConfiguration('goal_mode')}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )])
