import sys

from sick_lidar_localization.srv import LocSetMapSrv, LocLoadMapToCacheSrv
from kobuki_ros_interfaces.action import AutoDocking

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


class MinimalMapClientSet(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(LocSetMapSrv, 'LocSetMap')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LocSetMapSrv.Request()

    def send_request(self, path):
        self.req.mappath = str(path)
        self.future = self.cli.call_async(self.req)


class MinimalMapClientLoad(Node):

    def __init__(self):
        super().__init__('minimal_client_set_async')
        self.cli = self.create_client(LocLoadMapToCacheSrv, 'LocLoadMapToCache')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LocLoadMapToCacheSrv.Request()

    def send_request(self, path):
        self.req.mappath = str(path)
        self.future = self.cli.call_async(self.req)


class DockingClient(Node):

    def __init__(self):
        super().__init__('docking_client')
        self._action_client = ActionClient(self, AutoDocking, 'auto_docking_action')

    def send_request(self):
        goal_msg = AutoDocking.Goal()
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
