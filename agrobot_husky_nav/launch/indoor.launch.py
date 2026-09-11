"""Navegação indoor: MID360 -> mapa local voxelizado (+ scan 2D opcional para a guarda de obstáculos)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
        [FindPackageShare('agrobot_husky_nav'), 'config', 'indoor.yaml'])),
    DeclareLaunchArgument('livox_topic', default_value='/livox/lidar'),
    # mesma montagem do nav.launch.py: MID360 invertido e girado 90 graus
    DeclareLaunchArgument('livox_xyz', default_value='-0.168 0.0 0.45'),
    DeclareLaunchArgument('livox_rpy', default_value='3.14159 0.0 -1.5708'),
    DeclareLaunchArgument('scan', default_value='true',
                          description='também publica o scan 2D (pointcloud_to_laserscan)'),
]


def launch_setup(context):
    ns = LaunchConfiguration('namespace')
    params = LaunchConfiguration('params')
    x, y, z = LaunchConfiguration('livox_xyz').perform(context).split()
    r, p, yw = LaunchConfiguration('livox_rpy').perform(context).split()
    nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher', name='livox_static_tf',
             namespace=ns, output='screen',
             arguments=['--x', x, '--y', y, '--z', z, '--roll', r, '--pitch', p, '--yaw', yw,
                        '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        Node(package='agrobot_husky_nav', executable='voxel_local_map', name='voxel_local_map',
             namespace=ns, output='screen',
             parameters=[params, {'input_topic': LaunchConfiguration('livox_topic'),
                                  'livox_xyz': [float(x), float(y), float(z)],
                                  'livox_rpy': [float(r), float(p), float(yw)]}]),
    ]
    if LaunchConfiguration('scan').perform(context).lower() in ('true', '1'):
        nodes.append(Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='livox_to_scan', namespace=ns, output='screen', parameters=[params],
            remappings=[('cloud_in', LaunchConfiguration('livox_topic')), ('scan', 'scan'),
                        ('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    return nodes


def generate_launch_description():
    return LaunchDescription(ARGS + [OpaqueFunction(function=launch_setup)])
