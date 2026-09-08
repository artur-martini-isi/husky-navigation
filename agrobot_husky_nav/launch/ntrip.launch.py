"""NTRIP corrections over ROS 2 for the Fixposition.

  ntrip_client (LORD-MicroStrain) -> <ns>/sensors/ins_0/rtcm (rtcm_msgs/Message)
  fixposition_driver forwards every RTCM message to the sensor over its TCP stream.
  The sensor's own NavSatFix (<ns>/sensors/ins_0/gps_0/fix) is fed back to the caster as GGA.

Credentials live OUTSIDE the package: /home/robot/ntrip_ibge.yaml (chmod 600), shaped like
config/ntrip_params.example.yaml.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('params_file', default_value='/home/robot/ntrip_ibge.yaml'),
    DeclareLaunchArgument('debug', default_value='false'),
]


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    ntrip = Node(
        package='ntrip_client', executable='ntrip_ros.py', name='ntrip_client',
        namespace=ns, output='screen',
        parameters=[LaunchConfiguration('params_file'), {
            'debug': LaunchConfiguration('debug'),
            'rtcm_message_package': 'rtcm_msgs',
            'rtcm_frame_id': 'ins_0_link',
            'reconnect_attempt_max': 100,
            'reconnect_attempt_wait_seconds': 5,
            'rtcm_timeout_seconds': 10,
        }],
        remappings=[('rtcm', 'sensors/ins_0/rtcm'),
                    ('fix', 'sensors/ins_0/gps_0/fix'),
                    ('nmea', 'ntrip_client/nmea_unused')],
    )
    return LaunchDescription(ARGS + [ntrip])
