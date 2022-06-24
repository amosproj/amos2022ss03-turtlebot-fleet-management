import rclpy
import time
import math
from .map_client import MinimalMapClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from kobuki_ros_interfaces.msg import MotorPower
from geometry_msgs.msg import PoseWithCovariance, Twist
from sensor_msgs.msg import BatteryState as ROSBatteryState
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sick_lidar_localization.msg import *
from vda5050_msgs.msg import AGVPosition, BatteryState, Connection, Order, OrderInformation, Visualization

class Worker(Node):
    # subscriber callbacks
    # MQTT
    def call_mqtt(self, msg):
        self.data = msg.data

    def call_order(self, msg):
        self.get_logger().info('Receiving: "%s"' % msg)
        self.orders_ids.append(msg.order_id)
        self.orders_nodes.append(msg.nodes)

    def call_map(self, msg):
        self.get_logger().info('Receiving: "%s"' % msg)
        with open('/opt/sick/SICKAppEngine/home/appdata/public/maps/current_room.vmap', 'w') as f:
            f.write(msg.data)
        path = "current_room.vmap"
        self.map_client.send_request(path)
        while rclpy.ok():
            rclpy.spin_once(self.map_client)
            if self.map_client.future.done():
                self.get_logger().info('Changed Map')
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
        # MQTT
        self.data = ""
        self.map_client = MinimalMapClient()
        #with open('/opt/sick/SICKAppEngine/home/appdata/public/maps/current_room.vmap', 'w') as f:
        #    f.write(msg.data)

        # Sick
        self.position_initialized = False
        self.localization_score = 0.0
        self.pose = [0.0, 0.0, 0.0]
        self.num_lanes = 0
        self.lanes = []

        self.driving = False
        self.orders_ids = []
        self.order_id = ""
        self.orders_nodes = []
        self.last_node_id = ""
        self.last_node_sequence_id = 0

        # Kuboki
        self.battery_charge = 0.0
        self.battery_voltage = 0.0

        # subscribe to
        # MQTT
        self.mqtt_sub = self.create_subscription(String, "/back", self.call_mqtt, 10)
        self.order_sub = self.create_subscription(Order, "/order", self.call_order, 10)
        self.map_sub = self.create_subscription(String, "/map", self.call_map, 10)
	# Sick location estimation and virtual line measurement
        self.location_sub = self.create_subscription(LocalizationControllerResultMessage0502, "/localizationcontroller/out/localizationcontroller_result_message_0502", self.call_location, 10)
        self.line_sub = self.create_subscription(LineMeasurementMessage0403, "/localizationcontroller/out/line_measurement_message_0403", self.call_linemeasurement, 10)
        # Kuboki
        self.battery_sub = self.create_subscription(ROSBatteryState, "/mobile_base/sensors/battery_state", self.call_batterystate, 10)

        # publish to
        # MQTT
        self.pub_mqtt = self.create_publisher(String, "/echo", 10)
        self.pub_state = self.create_publisher(OrderInformation, "/state", 10)
        self.pub_visualisation = self.create_publisher(Visualization, "/visualization", 10)
        self.pub_connection = self.create_publisher(Connection, "/connection", 10)

        # say hello
        msg_connection = Connection()
        msg_connection.connection_state = "ONLINE"
        self.pub_connection.publish(msg_connection)
        # cself.get_logger().info('Publishing: "%s"' %msg_connection)

        # Kuboki
        self.pub_power = self.create_publisher(MotorPower, "/mobile_base/commands/motor_power", 10)
        self.pub_vel = self.create_publisher(Twist, "/mobile_base/commands/velocity", 10)

        # switch on motor power
        msg_power = MotorPower()
        msg_power.state = 1
        self.pub_power.publish(msg_power)
        # self.get_logger().info('Publishing: "%s"' %msg_power)

        # demo move
        #msg_twist = Twist()
        #msg_twist.linear.x = 0.1
        #msg_twist.angular.z = 0.8
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

        msg_state = OrderInformation()
        msg_state.order_id = self.order_id
        msg_state.last_node_id = self.last_node_id
        msg_state.last_node_sequence_id = self.last_node_sequence_id
        msg_state.driving = self.driving
        msg_state.agv_position = msg_agvpos
        msg_state.battery_state = msg_batterystate
        self.pub_state.publish(msg_state)
        #self.get_logger().info('Publishing: "%s"' %msg_state)

        # optional
        # msg_vis = Visualization()
        # msg_vis.header_id = self.i
        # msg_vis.agv_position = msg_agvpos
        # self.pub_visualisation.publish(msg_vis)
        # self.get_logger().info('Publishing: "%s"' %msg_vis)
        self.i += 1


    # helpers for controller
    # map negative angles to positive ones
    def normalize_pose(self, pose):
        if pose < 0:
            return 360 - abs(pose)
        return pose

    # rotate turtlebot to given angle
    def rotate_self(self, angle, linear_speed=0.0):
        msg_twist = Twist()
        angular_speed = 0.4
        # pose normalization
        pose = self.normalize_pose(self.pose[2])
        angle = self.normalize_pose(angle)
        # rotate smaller direction
        if ((angle - pose) + 360) % 360 > 180:
            angular_speed *= -1
        # add acceptable margin
        lower_angle =  int(angle - 3)
        if lower_angle < 0:
            lower_angle = 360 + lower_angle
        upper_angle = int((angle + 3) % 360)
        angle_list = range(lower_angle, upper_angle + 1)
        if lower_angle > upper_angle:
            angle_list = list(range(lower_angle, 360)) + list(range(0, upper_angle + 1))
        # print out debug info
        self.get_logger().info('enter loop turn')
        self.get_logger().info('pose "%f"' %pose)
        self.get_logger().info('angle "%f"' %angle)
        self.get_logger().info('list lower "%d"' %angle_list[0])
        self.get_logger().info('list_upper "%d"' %angle_list[-1])
        # turn until target angle reached
        while(int(pose) not in angle_list):
            pose = self.normalize_pose(self.pose[2])
            # self.get_logger().info('currently "%d"' %pose)
            msg_twist.linear.x = linear_speed
            msg_twist.angular.z = angular_speed
            self.pub_vel.publish(msg_twist)
            time.sleep(0.2)
        self.get_logger().info('pose "%d"' %pose)
        self.get_logger().info('leave')
        #msg_twist.angular.z = 0.0
        #self.pub_vel.publish(msg_twist)

    # follow line
    def follow_line(self, goal):
        msg_twist = Twist()
        last_lane = False
        self.get_logger().info('enter loop line')
        while True:
            # calculate x and y d from goal
            dx = abs(self.pose[0] - goal.node_position.x)
            dy = abs(self.pose[1] - goal.node_position.y)
            # check if goal reached
            if(dx < 0.2 and dy < 0.2):
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
                self.get_logger().info('lane "%d"' %self.num_lanes)
                num = 0
                if self.num_lanes >= 1:
                    #num = [abs(l) for l in self.lanes].index(min([abs(l) for l in self.lanes]))
                    d_lane = min(self.lanes, key=abs)
                #self.get_logger().info('lane_num = "%d"' %num)
                # right margin
                if d_lane > 40 and last_lane == True:
                    self.get_logger().info('turn left')
                    self.rotate_self((self.pose[2] + 5) % 360, linear_speed)
                # left margin
                elif d_lane < -40 and last_lane == True:
                    self.get_logger().info('turn right')
                    self.rotate_self((self.pose[2] - 5) % 360, linear_speed)
                last_lane = True
            time.sleep(0.3)
        self.get_logger().info('leave')
        #msg_twist.linear.x = 0.0
        #self.pub_vel.publish(msg_twist)

    def controller_callback(self):
        # onshot only
        self.timer2.cancel()
        while True:
            finished_orders = []
            for order in self.orders_nodes or []:
                self.get_logger().info('Start order')
                for node in order or []:
                    self.get_logger().info('Go to node %s' %node.node_id)
                    # TODO make it work with arbitrary coord systems
                    pose = self.normalize_pose(self.pose[2])
                    # compute driving direction
                    dx = node.node_position.x - self.pose[0]
                    dy = node.node_position.y - self.pose[1]
                    direction = math.degrees(math.atan2(dy, dx))
                    # print out debug info
                    self.get_logger().info('d "%d"' %direction)
                    self.get_logger().info('dx "%s"' %dx)
                    self.get_logger().info('dy "%s"' %dy)
                    # rotate in driving direction
                    self.driving = True
                    self.get_logger().info('rotate')
                    linear_speed = 0.0
                    if abs((pose - self.normalize_pose(direction) + 180) % 360 - 180) < 30:
                        linear_speed = 0.2
                        self.get_logger().info('Small curve, keep driving and no full stop')
                    self.rotate_self(direction, linear_speed)
                    # follow line until next node
                    self.get_logger().info('Following line')
                    self.follow_line(node)
                    self.driving = False
                    #self.last_node_id = node.node_id
                    #self.last_node_sequence_id = node.last_node_sequence_id
                # when one order is finished move it to finished_orders
                finished_orders.append(order)
                self.get_logger().info('Finished order')
            # when all orders are finished, remove the finished orders
            for order in finished_orders:
                self.orders_nodes.remove(order)


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
