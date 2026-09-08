"""Everything for field operation: NTRIP corrections over ROS + navigation stack (mission idle)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('mission_file', default_value=''),
    DeclareLaunchArgument('ntrip', default_value='true'),
]


def generate_launch_description():
    share = FindPackageShare('agrobot_husky_nav')
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'nav.launch.py'])),
        launch_arguments={'namespace': LaunchConfiguration('namespace'),
                          'mission_file': LaunchConfiguration('mission_file')}.items())
    ntrip = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'ntrip.launch.py'])),
        launch_arguments={'namespace': LaunchConfiguration('namespace')}.items(),
        condition=__import__('launch.conditions', fromlist=['IfCondition']).IfCondition(LaunchConfiguration('ntrip')))
    return LaunchDescription(ARGS + [nav, ntrip])
