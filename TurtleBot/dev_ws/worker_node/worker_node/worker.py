import rclpy
import time
import math
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from kobuki_ros_interfaces.msg import MotorPower
from geometry_msgs.msg import PoseWithCovariance, Twist
from sensor_msgs.msg import BatteryState as ROSBatteryState
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sick_lidar_localization.msg import *
from vda5050_msgs.msg import AGVPosition, BatteryState, Order, OrderInformation, Visualization

class Worker(Node):
    # subscriber callbacks
    # MQTT
    def call_mqtt(self, msg):
        self.data = msg.data

    def call_order(self, msg):
        self.get_logger().info('Receiving: "%s"' % msg)
        self.orders_ids.append(msg.order_id)
        self.orders_nodes.append(msg.nodes)

    # Sick
    def call_location(self, msg):
        # self.get_logger().info('Receiving "%s"' %msg)
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
        self.order_ids = []
        self.orderUpdateId = 0

        # Sick
        self.pose = [0.0, 0.0, 0.0]
        self.num_lanes = 0
        self.lanes = []

        self.orders_ids = []
        self.orders_nodes = []

        # Kuboki
        self.battery_charge = 0.0
        self.battery_voltage = 0.0

        # subscribe to
        # MQTT
        self.mqtt_sub = self.create_subscription(String, "/back", self.call_mqtt, 10)
        self.order_sub = self.create_subscription(Order, "/order", self.call_order, 10)
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

        # Kuboki
        self.pub_power = self.create_publisher(MotorPower, "/mobile_base/commands/motor_power", 10)
        self.pub_vel = self.create_publisher(Twist, "/mobile_base/commands/velocity", 10)

        timer_period = 1
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(timer_period, self.publisher_callback, self.group1)
        self.timer2 = self.create_timer(timer_period, self.controller_callback, self.group2)
        self.i = 0

    # timer callbacks
    def publisher_callback(self):
        # switch on power for motor
        msg_power = MotorPower()
        msg_power.state = 1
        self.pub_power.publish(msg_power)
        # self.get_logger().info('Publishing: "%s"' %msg_power)

        # demo move
        # msg_twist = Twist()
        # msg_twist.linear.x = 0.1
        # msg_twist.angular.z = 0.8
        # self.pub_vel.publish(msg_twist)

        # publish position and battery to FMS
        msg_agvpos = AGVPosition()
        # x, y in m, thea in [-pi ... pi]
        msg_agvpos.x = self.pose[0]
        msg_agvpos.y = self.pose[1]
        msg_agvpos.theta = self.pose[2] / 1000 * 3.14

        msg_batterystate = BatteryState()
        msg_batterystate.battery_charge = self.battery_charge
        msg_batterystate.battery_voltage = self.battery_voltage

        msg_state = OrderInformation()
        msg_state.agv_position = msg_agvpos
        msg_state.battery_state = msg_batterystate
        # self.get_logger().info('Publishing: "%s"' %msg_state)

        # optional
        # msg_vis = Visualization()
        # msg_vis.header_id = self.i
        # msg_vis.agv_position = msg_agvpos
        # self.pub_visualisation.publish(msg_vis)
        # self.get_logger().info('Publishing: "%s"' %msg_vis)

        self.i += 1

    # maps negative poses to 0...+360
    def normalize_pose(self, pose):
        if pose < 0:
            return 360 - abs(self.pose[2])
        return pose

    # rotates by a given angle
    def rotate_self(self, angle):
        msg_twist = Twist()
        pose = self.normalize_pose(self.pose[2])

        # some slack
        lower_angle = ((angle + pose) % 360) - 5
        upper_angle = (((angle + pose) % 360) + 5) % 360

        self.get_logger().info('enter loop')
        self.get_logger().info('pose "%f"' %pose)
        self.get_logger().info('angle "%f"' %angle)
        self.get_logger().info('lower_angle "%f"' %lower_angle)
        self.get_logger().info('upper_angle "%f"' %upper_angle)

        # TODO check if turning in the other direction is faster
        # turn until target angle reached
        while(pose > upper_angle or pose < lower_angle):
            pose = self.normalize_pose(self.pose[2])
            self.get_logger().info('currently "%d"' %pose)
            msg_twist.angular.z = 0.2
            self.pub_vel.publish(msg_twist)
            time.sleep(0.1)

        self.get_logger().info('leave')

        msg_twist.angular.z = 0.0
        self.pub_vel.publish(msg_twist)

    # follows virtual line
    def follow_line(self, goal):
        msg_twist = Twist()

        # some slack
        lower_goal_x = goal.node_position.x * 0.7
        lower_goal_y = goal.node_position.y * 0.7
        upper_goal_x = goal.node_position.x * 1.3
        upper_goal_y = goal.node_position.y * 1.3

        self.get_logger().info('enter loop')

        while True:
            # calculate x and y d from goal
            dx = abs(self.pose[0] - goal.node_position.x)
            dy = abs(self.pose[1] - goal.node_position.y)

            # check if goal reached
            if(dx < 0.25 and dy < 0.25):
               break

            self.get_logger().info('px = "%f"' %self.pose[0])
            self.get_logger().info('py = "%f"' %self.pose[1])
            self.get_logger().info('dx = "%f"' %(upper_goal_x - self.pose[0]))
            self.get_logger().info('dy = "%f"' %(upper_goal_y - self.pose[1]))

            # drive straight
            msg_twist.linear.x = 0.2
            self.pub_vel.publish(msg_twist)

            # TODO use virtual line sensor
            # if self.lane_num != 0:
            #    if self.lanes[0] > 15:
            #        TODO relative turn
            #    elif self.lanes[0] < -15:
            #        ...

            time.sleep(0.3)

        self.get_logger().info('leave')

        msg_twist.linear.x = 0.0
        self.pub_vel.publish(msg_twist)

    def controller_callback(self):
        # onshot only
        self.timer2.cancel()

        while True:
            self.get_logger().info('Take new order')
            for order in self.orders_nodes or []:
                self.get_logger().info('Got to next node')
                for node in order or []:
                    # calculate distance to next node
                    # TODO make it work with arbitrary coord systems
                    dx = node.node_position.x - self.pose[0]
                    dy = node.node_position.y - self.pose[1]

                    self.get_logger().info('dx "%s"' %dx)
                    self.get_logger().info('dy "%s"' %dy)

                    # calculate direction of next node
                    pose = self.normalize_pose(self.pose[2])
                    direction = math.degrees(math.atan2(dy, dx))

                    self.get_logger().info('d "%d"' %direction)

                    # rectangular turn
                    #if direction < 135 and direction >= 45:
                        # turn right 90 deg
                        #self.get_logger().info('Rotating right')
                        #self.rotate_self(90)
                    #elif direction >= 135 and direction < 215:
                        # turn 180 deg
                        #self.get_logger().info('Turning around')
                        #self.rotate_self(180)
                    #elif direction >= 215 and direction < 305:
                        # turn left 270 deg
                        #self.get_logger().info('Rotating left')
                        #self.rotate_self(270)
                    #elif direction >= 305 or direction < 45:
                        # drive straight
                        #self.get_logger().info('Straight')

                    # turn robot to calculated direction
                    self.rotate_self(direction - pose)
                    self.get_logger().info('Following line')
                    self.follow_line(node)
                return


def main(args=None):
    rclpy.init(args=args)

    worker = Worker()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(worker)
    executor.spin()

if __name__ == '__main__':
    main()

