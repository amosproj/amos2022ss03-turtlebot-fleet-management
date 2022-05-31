import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sick_lidar_localization.msg import *
from vda5050_msgs.msg import Order
from vda5050_msgs.msg import AGVPosition


class Worker(Node):

    def call_mqtt(self, msg):
        self.data = msg.data

    def call_order(self, msg):
        self.get_logger().info('Order : "%d " arrived' % msg.orderId)
        self.nodes = msg.nodes
        self.edges = msg.edges
        self.orderID = msg.orderId
        self.orderupdateID = msg.orderUpdateId

    def call_location(self, msg):
        self.robot_pose = [msg.x, msg.y]

    def call_linemeasurement(self, msg):
        self.line_num = msg.num_lanes
        self.lines = msg.lanes

    def __init__(self):
        super().__init__('worker')

        self.data = ""
        self.robot_pose = []
        self.line_num = 0
        self.lines = []
        #self.vel_lin = Vector3()
        #self.vel_angle = Vector3()
        self.orderID = ""
        self.orderupdateID = ""
        self.orderUpdateId = 0
        self.nodes = []
        self.edges = []

        #subcribe to energy level
        self.mqtt_sub = self.create_subscription(String, "/back", self.call_mqtt, 10)
        self.order_sub = self.create_subscription(Order, "/Order", self.call_order, 10)
        self.location_sub = self.create_subscription(LocalizationControllerResultMessage0502, "/localizationcontroller/out/localizationcontroller_result_message_0502", self.call_location, 10)
        self.line_sub = self.create_subscription(LineMeasurementMessage0403, "/localizationcontroller/out/line_measurement_message_0403", self.call_linemeasurement, 10)

        #self.pub_vel = self.create_publisher(Twist, "/commands/velocity", 10)
        #publish motor power kobuki
        self.pub_agvpos = self.create_publisher(AGVPosition, "/AGVPosition", 10)
        self.pub_mqtt = self.create_publisher(String, "/echo", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        #msg_twist = Twist()
        #msg_twist.linear = self.vel_lin
        #msg_twist.angle = self.vel_angle
        #self.pub_vel.publish(msg_twist)
        msg_agvpos = AGVPosition()
        msg_agvpos.x = 42.0 #self.vel_lin(0)
        msg_agvpos.y = 10.0 #self.vel_lin(1)
        msg_agvpos.theta = 0.0
        self.pub_agvpos.publish(msg_agvpos) #agvPosition
        self.get_logger().info('Publishing: "%s"' % msg_agvpos)
        #msg = String()
        #msg.data = "Hallo: %d" %self.i
        #self.pub_mqtt.publish(msg) #define msg to be published
        self.i += 1

    def calculate_direction(self, pos: tuple, dest: tuple):
        """
        Calculates direction the robot needs to go, to reach the desired graph node.

        :param pos: tuple of numeric values representing the x- and y- coordinate of the current turtlebot position
        :param dest: tuple of numeric values representing the x- and y- coordinate of the destination
        :return: int in ]-180000;180000] representing the direction in which the destination lies in milli-degrees
        """
        dx = dest[0] - pos[0]
        dy = dest[1] - pos[1]
        direction = math.degrees(math.atan2(dy, dx)) * 1000
        # Direction in milli-degrees in order to fit the heading values coming from the SICK-LidarLoc software
        # TODO Find out what a heading of 0 deg means in the SICK-LidarLoc software
        # In this first implementation 0 deg would mean East.
        return direction

    def controller(self):
        #needs line measurement topic
        #check if robot is still on line
            #if not on line calculate new direction through calculate_direction()
        #publish topic for robot to move
        pass


def main(args=None):

    rclpy.init(args=args)

    worker = Worker()

    rclpy.spin(worker)


if __name__ == '__main__':
    main()


