import sys

from sick_lidar_localization.srv import LocSetMapSrv, LocLoadMapToCacheSrv
from kobuki_ros_interfaces.action import AutoDocking

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


# service client for setting the map in SICK LiDARLoc
class MinimalMapClientSet(Node):

    def __init__(self):
        super().__init__('minimal_map_client_set')
        self.cli = self.create_client(LocSetMapSrv, 'LocSetMap')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('map set service not available, waiting again...')
        self.req = LocSetMapSrv.Request()
        self.future = None

    def send_request(self, path):
        self.req.mappath = str(path)
        self.future = self.cli.call_async(self.req)


# service client for loading the newly set map into SICK LiDARLoc
class MinimalMapClientLoad(Node):

    def __init__(self):
        super().__init__('minimal_map_client_load')
        self.cli = self.create_client(LocLoadMapToCacheSrv, 'LocLoadMapToCache')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('map load service not available, waiting again...')
        self.req = LocLoadMapToCacheSrv.Request()
        self.future = None

    def send_request(self, path):
        self.req.mappath = str(path)
        self.future = self.cli.call_async(self.req)


# action client for starting auto docking of kobuki robot
class DockingClient(Node):

    def __init__(self):
        super().__init__('docking_client')
        self._action_client = ActionClient(self, AutoDocking, 'auto_docking_action')

    def send_request(self):
        goal_msg = AutoDocking.Goal()
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
