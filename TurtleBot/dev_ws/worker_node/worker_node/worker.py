import rclpy
import time
import math
import base64
from .map_client import MinimalMapClientSet, MinimalMapClientLoad, DockingClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from kobuki_ros_interfaces.msg import MotorPower
from geometry_msgs.msg import PoseWithCovariance, Twist
from sensor_msgs.msg import BatteryState as ROSBatteryState
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sick_lidar_localization.msg import *
from vda5050_msgs.msg import Action, AGVPosition, BatteryState, Connection, InstantActions, Order, OrderInformation, Visualization


class Worker(Node):
    # subscriber callbacks
    # MQTT
    def call_mqtt(self, msg):
        self.data = msg.data

    def call_action(self, msg):
        self.get_logger().info('Receiving: "%s"' % msg)
        for action in msg.instantActions:
            if action.actionType == "startCharging":
                self.get_logger().info("start charging")
                self.action_client.send_request()
                self.charging = True
            elif action.actionType == "stopCharging":
                self.charging = False
            elif action.actionType == "cancelOrder":
                self.get_logger().info('cancell Order')
                self.is_cancelled = True

    def call_order(self, msg):
        self.get_logger().info('Receiving: "%s"' % msg)
        order_id = int(msg.order_id)
        #if order_id not in self.finished_orders:
        self.orders[order_id] = msg.nodes

    def call_map(self, msg):
        #self.get_logger().info('Receiving: "%s"' % msg)
        data = base64.b64decode(msg.data).decode()
        with open('/opt/sick/SICKAppEngine/home/appdata/public/maps/current_room.vmap', 'w') as f:
            f.write(data)
        path = "current_room.vmap"
        #self.get_logger().info("opened map to write")
        self.map_client_set.send_request(path)
        while rclpy.ok():
            rclpy.spin_once(self.map_client_set)
            if self.map_client_set.future.done():
                self.get_logger().info('set map')
                break

        self.map_client_load.send_request(path)
        while rclpy.ok():
            rclpy.spin_once(self.map_client_load)
            if self.map_client_load.future.done():
                self.get_logger().info('loaded Map')
                break

    # Sick
    def call_location(self, msg):
        # self.get_logger().info('Receiving "%s"' %msg)
        self.position_initialized = True if msg.loc_status > 30 else False
        self.localization_score = (100.0 - msg.loc_status) / 100
        # x, y in m, heading in deg
        self.pose = [float(msg.x / 1000), float(msg.y / 1000), float(msg.heading / 1000)]

    def call_linemeasurement(self, msg):
        self.num_lanes = msg.num_lanes
        self.lanes = msg.lanes

    # Kuboki
    def call_batterystate(self, msg):
        self.battery_charge = msg.percentage
        self.battery_voltage = msg.voltage

    def __init__(self):
        super().__init__('worker')

        self.get_logger().info('worker started')

        # vars

        # Turtlebot
        self.serial_number = "1"

        # MQTT
        self.data = ""
        self.map_client_set = MinimalMapClientSet()
        self.map_client_load = MinimalMapClientLoad()

        # Sick
        self.position_initialized = False
        self.localization_score = 0.0
        self.pose = [0.0, 0.0, 0.0]
        self.num_lanes = 0
        self.lanes = []

        self.driving = False
        self.orders = {}
        self.finished_orders = []
        self.current_order = ""
        self.last_node_id = ""
        self.last_node_sequence_id = 0
        self.is_cancelled = False

        # Kuboki
        self.battery_charge = 0.0
        self.battery_voltage = 0.0
        self.charging = False
        self.action_client = DockingClient()

        # Subscribe to
        # - MQTT
        self.mqtt_sub = self.create_subscription(String, "/back", self.call_mqtt, 10)
        self.action_sub = self.create_subscription(Order, "/order", self.call_order, 10)
        self.order_sub = self.create_subscription(Order, "/instantAction", self.call_action, 10)
        self.map_sub = self.create_subscription(String, "/map", self.call_map, 10)
        # - Sick location estimation and virtual line measurement
        self.location_sub = self.create_subscription(LocalizationControllerResultMessage0502, "/localizationcontroller/out/localizationcontroller_result_message_0502", self.call_location, 10)
        self.line_sub = self.create_subscription(LineMeasurementMessage0403, "/localizationcontroller/out/line_measurement_message_0403", self.call_linemeasurement, 10)
        # - Kuboki
        self.battery_sub = self.create_subscription(ROSBatteryState, "/mobile_base/sensors/battery_state", self.call_batterystate, 10)

        # Publish to
        # - MQTT
        self.pub_mqtt = self.create_publisher(String, "/echo", 10)
        self.pub_state = self.create_publisher(OrderInformation, "/state", 10)
        self.pub_visualisation = self.create_publisher(Visualization, "/visualization", 10)
        self.pub_connection = self.create_publisher(Connection, "/connection", 10)
        # say hello
        msg_connection = Connection()
        msg_connection.serial_number = self.serial_number
        msg_connection.connection_state = "ONLINE"
        self.pub_connection.publish(msg_connection)

        # - Kuboki
        self.pub_power = self.create_publisher(MotorPower, "/mobile_base/commands/motor_power", 10)
        self.pub_vel = self.create_publisher(Twist, "/mobile_base/commands/velocity", 10)
        # switch on motor power
        msg_power = MotorPower()
        msg_power.state = 1
        self.pub_power.publish(msg_power)
        # self.get_logger().info('Publishing: "%s"' %msg_power)

        # demo move
        # msg_twist = Twist()
        # msg_twist.linear.x = 0.1
        # msg_twist.angular.z = 0.8
        # self.pub_vel.publish(msg_twist)

        # create independent timers
        timer_period = 1
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(timer_period, self.publisher_callback, self.group1)
        self.timer2 = self.create_timer(timer_period, self.controller_callback, self.group2)
        self.i = 0

    def deinit(self):
        msg_connection = Connection()
        msg_connection.serial_number = self.serial_number
        msg_connection.connection_state = "OFFLINE"
        self.pub_connection.publish(msg_connection)
        # self.get_logger().info('Publishing: "%s"' %msg_connection)

    # timer callbacks
    # publisher
    def publisher_callback(self):
        # publish battery state and position to FMS
        msg_agvpos = AGVPosition()
        msg_agvpos.position_initialized = self.position_initialized
        msg_agvpos.localization_score = self.localization_score
        # x, y in m, thea in [-pi ... pi]
        msg_agvpos.x = self.pose[0]
        msg_agvpos.y = self.pose[1]
        msg_agvpos.theta = self.pose[2] / 1000 * 3.14

        msg_batterystate = BatteryState()
        msg_batterystate.battery_charge = self.battery_charge
        msg_batterystate.battery_voltage = self.battery_voltage
        msg_batterystate.charging = self.charging

        msg_state = OrderInformation()
        msg_state.serial_number = self.serial_number
        msg_state.order_id = str(self.current_order)
        msg_state.last_node_id = str(self.last_node_id)
        msg_state.last_node_sequence_id = self.last_node_sequence_id
        msg_state.driving = self.driving
        msg_state.agv_position = msg_agvpos
        msg_state.battery_state = msg_batterystate
        self.pub_state.publish(msg_state)
        # self.get_logger().info('Publishing: "%s"' %msg_state)

        # optional
        # msg_vis = Visualization()
        # msg_vis.serial_number = self.serial_number
        # msg_vis.header_id = self.i
        # msg_vis.agv_position = msg_agvpos
        # self.pub_visualisation.publish(msg_vis)
        # self.get_logger().info('Publishing: "%s"' %msg_vis)
        self.i += 1

    # helpers for controller
    def normalize_angle(self, angle):
        """
        Gets angle within [-180,180] and returns an angle within [0, 360]
        """
        if angle < 0:
            return 360 - abs(angle)
        return angle

    # rotate turtlebot to given angle
    def rotate_self(self, angle, linear_speed=0.0):
        self.get_logger().info('Rotate')
        msg_twist = Twist()
        angular_speed = 0.5
        pose = self.normalize_angle(self.pose[2])
        # rotate in negative direction
        if ((angle - pose) + 360) % 360 > 180:
            angular_speed *= -1
        # add acceptable margin
        lower_angle = int((angle - 3) % 360)
        upper_angle = int((angle + 3) % 360)
        angle_list = range(lower_angle, upper_angle + 1)
        if lower_angle > upper_angle:
            angle_list = list(range(lower_angle, 360)) + list(range(0, upper_angle + 1))
        # print out debug info
        self.get_logger().info('enter loop turn')
        self.get_logger().info('pose "%f"' % pose)
        self.get_logger().info('angle "%f"' % angle)
        self.get_logger().info('list lower "%d"' % angle_list[0])
        self.get_logger().info('list_upper "%d"' % angle_list[-1])
        # turn until target angle reached
        while int(pose) not in angle_list:
            pose = self.normalize_angle(self.pose[2])
            # self.get_logger().info('currently "%d"' %pose)
            msg_twist.linear.x = linear_speed
            msg_twist.angular.z = angular_speed
            self.pub_vel.publish(msg_twist)
            time.sleep(0.2)
        self.get_logger().info('pose "%d"' % pose)
        self.get_logger().info('Leave Rotate')
        #msg_twist.angular.z = 0.0
        #self.pub_vel.publish(msg_twist)

    # follow line
    def follow_line(self, goal):
        self.get_logger().info('Follow Line')
        msg_twist = Twist()
        last_lane = False
        self.get_logger().info('enter loop line')
        while True:
            # calculate x and y d from goal
            dx = abs(self.pose[0] - goal.node_position.x)
            dy = abs(self.pose[1] - goal.node_position.y)
            # check if goal reached
            if dx < 0.2 and dy < 0.2:
                break
            # print debug info
            # self.get_logger().info('px = "%f"' %self.pose[0])
            # self.get_logger().info('py = "%f"' %self.pose[1])
            # self.get_logger().info('dx = "%f"' %dx)
            # self.get_logger().info('dy = "%f"' %dy)
            linear_speed = 0.3
            msg_twist.linear.x = linear_speed
            self.pub_vel.publish(msg_twist)
            # use virtual line sensor
            # check if robot was on line in last iteration
            if self.num_lanes == 0:
                last_lane = False
            # use virtual line information if available and not too close to intersection
            if self.num_lanes != 0 and (dx > 0.5 or dy > 0.5):
                self.get_logger().info('lanes "%d"' % self.num_lanes)
                # get distance to nearest lane
                d_lane = min(self.lanes, key=abs)
                # right margin
                if d_lane > 37 and last_lane:
                    self.get_logger().info('turn left')
                    self.rotate_self((self.pose[2] + 3) % 360, 0.2)
                # left margin
                elif d_lane < -37 and last_lane:
                    self.get_logger().info('turn right')
                    self.rotate_self((self.pose[2] - 3) % 360, 0.2)
                last_lane = True
            time.sleep(0.3)
        self.get_logger().info('Leave Follow Line')
        #msg_twist.linear.x = 0.0
        #self.pub_vel.publish(msg_twist)

    def drive_to_node(self, node):
        self.get_logger().info('Drive to node %s' % node.node_id)
        # TODO make it work with arbitrary coord systems
        pose = self.normalize_angle(self.pose[2])
        # compute driving direction
        dx = node.node_position.x - self.pose[0]
        dy = node.node_position.y - self.pose[1]
        direction = self.normalize_angle(math.degrees(math.atan2(dy, dx)))
        # print out debug info
        self.get_logger().info('direction "%d"' % direction)
        self.get_logger().info('dist_x "%s"' % dx)
        self.get_logger().info('dist_y "%s"' % dy)
        # When the rotation angle is small (< 30) the turtlebot can keep driving while rotating
        linear_speed = 0.0
        if abs((pose - direction + 180) % 360 - 180) < 30:
            linear_speed = 0.2
            self.get_logger().info('Small curve, keep driving and no full stop')
        # rotate in driving direction
        self.driving = True
        self.rotate_self(direction, linear_speed)
        # follow line until next node
        self.follow_line(node)
        self.driving = False
        self.get_logger().info('Reached node %s' % node.node_id)
        self.last_node_id = int(node.node_id)
        self.last_node_sequence_id = int(node.sequence_id)

    def controller_callback(self):
        # one-shot only
        self.timer2.cancel()
        while True:
            for order_id in list(self.orders.keys()):
                self.get_logger().info('Start order %d' % order_id)
                self.current_order = order_id
                node_counter = 0
                while True:
                    nodes = self.orders[order_id]
                    if node_counter >= len(nodes):
                        break
                    # TODO check if turtlebot is already on target node
                    if nodes[node_counter].released:
                        self.drive_to_node(nodes[node_counter])
                        node_counter += 1
                    if self.is_cancelled:
                        break
                # when one order is finished move it to finished_orders
                self.finished_orders.append(order_id)
                self.current_order = ""
                self.get_logger().info('Finished order %d' % order_id)
                del self.orders[order_id]

            # when all orders are finished, remove the finished orders
            #for order in self.finished_orders:
            #    del self.orders[order]


def main(args=None):
    rclpy.init(args=args)

    worker = Worker()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(worker)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        worker.deinit()


if __name__ == '__main__':
    main()
