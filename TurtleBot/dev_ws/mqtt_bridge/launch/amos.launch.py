import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('mqtt_bridge'),
        'config',
        'amos_params.yaml'
        )
    node=Node(
        package = 'mqtt_bridge',
        name = 'mqtt_bridge_node',
        executable = 'mqtt_bridge_node',
        parameters = [config]
    )
    ld.add_action(node)
    return ld