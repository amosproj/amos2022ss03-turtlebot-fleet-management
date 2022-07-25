from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # configurations
    mqtt_config = os.path.join(
        get_package_share_directory('mqtt_bridge'),
        'config',
        'amos_params.yaml'
    )

    kobuki_share_dir = get_package_share_directory('kobuki_auto_docking')

    kobuki_node_config = os.path.join(
        kobuki_share_dir,
        'config',
        'kobuki_node_params.yaml'
    )

    with open(kobuki_node_config, 'r') as f:
        kobuki_node_params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    kobuki_docking_config = os.path.join(
        kobuki_share_dir,
        'config',
        'auto_docking.yaml'
    )

    with open(kobuki_docking_config, 'r') as f:
        kobuki_docking_params = yaml.safe_load(f)['kobuki_auto_docking']['ros__parameters']

    # nodes
    worker_node = Node(
        package="worker_node",
        executable="worker",
    )

    mqtt_node = Node(
        package="mqtt_bridge",
        executable="mqtt_bridge_node",
        parameters=[mqtt_config],
        arguments=['--ros-args', '--log-level', 'error'],
    )

    sick_node = Node(
        package="sick_lidar_localization",
        executable="sick_lidar_localization",
        parameters=[],
        arguments=['./src/sick_lidar_localization/launch/sick_lidar_localization.launch', '--ros-args', '--log-level','error'],
    )

    kobuki_node = ComposableNode(
        package='kobuki_node',
        plugin='kobuki_node::KobukiRos',
        name='kobuki_node',
        parameters=[kobuki_node_params],
    )

    kobuki_auto_docking_node = ComposableNode(
        package='kobuki_auto_docking',
        plugin='kobuki_auto_docking::AutoDockingROS',
        name='kobuki_auto_docking',
        parameters=[kobuki_docking_params]
    )

    kobuki_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='mobile_base_container',
        namespace='mobile_base',
        composable_node_descriptions=[
            kobuki_node,
            kobuki_auto_docking_node
        ],
        output='both',
    )

    return LaunchDescription([mqtt_node, sick_node, kobuki_container, worker_node])
