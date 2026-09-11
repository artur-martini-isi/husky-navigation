"""Mapeamento global 2D (slam_toolbox) a partir do scan do MID360.

Publica o frame `map`, a transformada `map -> odom` e o `map` (OccupancyGrid) do prédio.
Precisa do `scan`, ou seja, do `indoor.launch.py` (ou `nav.launch.py`) no ar.

    ros2 launch agrobot_husky_nav slam.launch.py
    ~/save_map.sh meu_mapa          # grava .pgm/.yaml e o grafo de poses

O `slam_toolbox` é um nó de **ciclo de vida**: sobe em `unconfigured` e não assina nada até ser
levado a `active`. Este launch faz as duas transições sozinho (mesmo padrão do launch oficial).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
        [FindPackageShare('agrobot_husky_nav'), 'config', 'slam.yaml'])),
    DeclareLaunchArgument('mode', default_value='mapping',
                          description='mapping = constrói o mapa; localization = usa um mapa salvo'),
    DeclareLaunchArgument('map_file', default_value='',
                          description='mapa serializado para o modo localization (sem extensão)'),
    DeclareLaunchArgument('autostart', default_value='true',
                          description='leva o nó a active automaticamente'),
]


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')

    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        namespace=ns, output='screen',
        parameters=[LaunchConfiguration('params'),
                    {'mode': LaunchConfiguration('mode'),
                     'map_file_name': LaunchConfiguration('map_file'),
                     'use_lifecycle_manager': False}],
        # o robô tem TF com namespace; sem estes remaps o slam não enxerga nada
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('/map', 'map'), ('/map_metadata', 'map_metadata')],
    )
    configurar = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                          transition_id=Transition.TRANSITION_CONFIGURE),
        condition=IfCondition(autostart))
    ativar = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription(ARGS + [slam, ativar, configurar])
