"""Par estéreo da ZED 2i (sem SDK). Opcionalmente sobe o stereo_image_proc para retificar."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('namespace', default_value='a300_00096'),
    DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
        [FindPackageShare('agrobot_husky_nav'), 'config', 'zed_stereo.yaml'])),
    DeclareLaunchArgument('resolution', default_value='HD'),
    DeclareLaunchArgument('fps', default_value='30'),
    DeclareLaunchArgument('rectify', default_value='false',
                          description='sobe o stereo_image_proc (retificação + disparidade)'),
    # TF do suporte da câmera. Desligado por padrão: MEDIR a montagem antes de ligar,
    # senão publica uma transformada errada. xyz em metros, rpy em radianos, base_link -> zed_left_camera_optical_frame.
    DeclareLaunchArgument('publish_tf', default_value='false'),
    DeclareLaunchArgument('zed_xyz', default_value='0.0 0.0 0.6'),
    DeclareLaunchArgument('zed_rpy', default_value='-1.5708 0.0 -1.5708'),
]


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    camera = Node(
        package='agrobot_husky_nav', executable='zed_stereo', name='zed_stereo',
        namespace=ns, output='screen',
        parameters=[LaunchConfiguration('params'),
                    {'resolution': LaunchConfiguration('resolution'),
                     'fps': LaunchConfiguration('fps')}],
    )
    # stereo_image_proc espera left/ e right/ dentro do seu namespace
    rectify = GroupAction(
        condition=IfCondition(LaunchConfiguration('rectify')),
        actions=[
            PushRosNamespace(ns),
            PushRosNamespace('zed'),
            Node(package='stereo_image_proc', executable='disparity_node', name='disparity_node',
                 output='screen', parameters=[{'approximate_sync': True}]),
            Node(package='stereo_image_proc', executable='point_cloud_node', name='point_cloud_node',
                 output='screen', parameters=[{'approximate_sync': True}]),
        ])
    # o frame óptico segue a convenção de visão (z para frente, x para a direita, y para baixo):
    # por isso o rpy padrão gira o eixo do robô para o da câmera.
    tf = Node(
        condition=IfCondition(LaunchConfiguration('publish_tf')),
        package='tf2_ros', executable='static_transform_publisher', name='zed_static_tf',
        namespace=ns, output='screen',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'zed_left_camera_optical_frame'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    return LaunchDescription(ARGS + [camera, rectify, tf])
