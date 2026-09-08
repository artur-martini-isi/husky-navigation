"""Lightweight navigation stack for the Husky A300.

  static TF base_link -> livox_frame   (MID360 mount, adjust livox_xyz / livox_rpy)
  pointcloud_to_laserscan              /livox/lidar -> <ns>/scan
  gps_waypoint_follower                Fixposition + scan + joy -> <ns>/cmd_vel
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
        [FindPackageShare('agrobot_husky_nav'), 'config', 'nav_params.yaml'])),
    DeclareLaunchArgument('mission_file', default_value=''),
    DeclareLaunchArgument('autostart', default_value='false'),
    DeclareLaunchArgument('pose_source', default_value='fusion'),   # fusion | dual_gnss
    DeclareLaunchArgument('max_linear', default_value='0.5'),        # m/s, field tests start at 0.3
    DeclareLaunchArgument('obstacle_stop_distance', default_value='1.2'),  # m; scale up with speed
    DeclareLaunchArgument('obstacle_slow_distance', default_value='3.0'),  # m
    DeclareLaunchArgument('livox_topic', default_value='/livox/lidar'),
    # MID360 pose relative to base_link: "x y z" (m) and "roll pitch yaw" (rad).
    # Mounted UPSIDE DOWN under the Fixposition and turned 90 deg (2026-09-08): robot front = lidar -y.
    # roll=pi, yaw=-pi/2 maps lidar (x,y,z) -> base_link (-y,-x,-z). ~0.61 m above ground.
    DeclareLaunchArgument('livox_xyz', default_value='-0.168 0.0 0.45'),
    DeclareLaunchArgument('livox_rpy', default_value='3.14159 0.0 -1.5708'),
]


def launch_setup(context):
    ns = LaunchConfiguration('namespace')
    params = LaunchConfiguration('params')
    x, y, z = LaunchConfiguration('livox_xyz').perform(context).split()
    r, p, yw = LaunchConfiguration('livox_rpy').perform(context).split()

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='livox_static_tf',
        namespace=ns, output='screen',
        arguments=['--x', x, '--y', y, '--z', z, '--roll', r, '--pitch', p, '--yaw', yw,
                   '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    pc2scan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='livox_to_scan', namespace=ns, output='screen',
        parameters=[params],
        remappings=[('cloud_in', LaunchConfiguration('livox_topic')), ('scan', 'scan'),
                    ('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    follower = Node(
        package='agrobot_husky_nav', executable='gps_waypoint_follower',
        name='gps_waypoint_follower', namespace=ns, output='screen',
        parameters=[params, {'mission_file': LaunchConfiguration('mission_file'),
                             'autostart': LaunchConfiguration('autostart'),
                             'pose_source': LaunchConfiguration('pose_source'),
                             'max_linear': LaunchConfiguration('max_linear'),
                             'obstacle_stop_distance': LaunchConfiguration('obstacle_stop_distance'),
                             'obstacle_slow_distance': LaunchConfiguration('obstacle_slow_distance')}],
    )
    return [static_tf, pc2scan, follower]


def generate_launch_description():
    return LaunchDescription(ARGS + [OpaqueFunction(function=launch_setup)])
