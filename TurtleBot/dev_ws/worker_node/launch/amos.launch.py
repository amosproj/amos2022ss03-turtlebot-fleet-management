from launch import LaunchDescription
from launch_ros.actions import Node
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

    kobuki_config = os.path.join(
        get_package_share_directory('turtlebot2_ros2'),
        'config/kobuki',
        'kobuki_node_params.yaml'
        )

    with open(kobuki_config, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']


    # nodes
    worker_node = Node(
        package="worker_node",
        executable="worker",
    )

    mqtt_node = Node(
        package="mqtt_bridge",
        executable="mqtt_bridge_node",
        parameters = [mqtt_config]
    )

    sick = Node(
        package="sick_lidar_localization",
        executable="sick_lidar_localization",
        parameters = [],
        arguments=['./src/sick_lidar_localization/launch/sick_lidar_localization.launch']
    )

    kobuki_node = Node(
        package="kobuki_node",
        executable="kobuki_ros_node",
        namespace='mobile_base',
        parameters=[params],
        remappings=[
            ('odom', '/odom'),
            ('joint_states', '/joint_states')
        ],
    )


    ld = LaunchDescription()
    ld.add_action(mqtt_node)
    ld.add_action(kobuki_node)
    ld.add_action(sick)
    #ld.add_action(worker_node)
    return ld
