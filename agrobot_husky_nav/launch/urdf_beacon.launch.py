"""Repete a descrição do robô num tópico próprio, para o painel 3D do Foxglove.

O URDF é publicado uma única vez e retido; assinante volátil nunca o recebe. Ver a explicação
completa no cabeçalho de `urdf_beacon.py`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('period', default_value='5.0'),
    DeclareLaunchArgument('output_topic', default_value='robot_description_foxglove'),
]


def generate_launch_description():
    return LaunchDescription(ARGS + [Node(
        package='agrobot_husky_nav', executable='urdf_beacon', name='urdf_beacon',
        namespace=LaunchConfiguration('namespace'), output='screen',
        parameters=[{'period': LaunchConfiguration('period'),
                     'output_topic': LaunchConfiguration('output_topic')}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )])
