import rclpy
from rclpy.node import Node
from kobuki_ros_interfaces.msg import MotorPower
from geometry_msgs.msg import PoseWithCovariance, Twist
from sensor_msgs.msg import BatteryState as ROSBatteryState
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sick_lidar_localization.msg import *
from vda5050_msgs.msg import AGVPosition, BatteryState, Order

class Worker(Node):
    # MQTT
    def call_mqtt(self, msg):
        self.data = msg.data

    def call_order(self, msg):
        self.get_logger().info('Receiving: "%s"' % msg)
        self.order_id = msg.order_id
        self.nodes = msg.nodes
        self.edges = msg.edges

    # Sick
    def call_location(self, msg):
        self.pose = [float(msg.x), float(msg.y), float(msg.heading)]

    def call_linemeasurement(self, msg):
        self.num_lanes = msg.num_lanes
        self.lanes = msg.lanes

    # Kuboki
    def call_batterystate(self, msg):
        self.battery_charge = msg.percentage
        self.battery_voltage = msg.voltage

    def __init__(self):
        super().__init__('worker')

        # vars
        # MQTT
        self.data = ""
        self.order_id = ""
        self.orderUpdateId = 0

        # Sick
        self.pose = [0.0, 0.0, 0.0]
        self.num_lanes = 0
        self.lanes = []

        self.orderID = ""
        self.orderUpdateId = 0
        self.nodes = []
        self.edges = []

        # Kuboki
        self.battery_charge = 0.0
        self.battery_voltage = 0.0

        # subscribe to
        # MQTT
        self.mqtt_sub = self.create_subscription(String, "/back", self.call_mqtt,10)
        self.order_sub = self.create_subscription(Order, "/Order", self.call_order,10)
	# Sick location estimation and virtual line measurement
        self.location_sub = self.create_subscription(LocalizationControllerResultMessage0502, "/localizationcontroller/out/localizationcontroller_result_message_0502", self.call_location, 10)
        self.line_sub = self.create_subscription(LineMeasurementMessage0403, "/localizationcontroller/out/line_measurement_message_0403", self.call_linemeasurement, 10)
        # Kuboki
        self.battery_sub = self.create_subscription(ROSBatteryState, "/mobile_base/sensors/battery_state", self.call_batterystate, 10)

        # publish to
        # MQTT
        self.pub_mqtt = self.create_publisher(String, "/echo", 10)
        self.pub_agvpos = self.create_publisher(AGVPosition, "/AGVPosition", 10)
        self.pub_batterystate = self.create_publisher(BatteryState, "/BatteryState", 10)
        # Kuboki
        self.pub_power = self.create_publisher(MotorPower, "/mobile_base/commands/motor_power", 10)
        self.pub_vel = self.create_publisher(Twist, "/mobile_base/commands/velocity", 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # switch on power for motor
        msg_power = MotorPower()
        msg_power.state = 1
        self.pub_power.publish(msg_power)

        # move to next node
        msg_twist = Twist()

        msg_twist.linear.x = 0.1
        msg_twist.angular.z = 0.8
        self.pub_vel.publish(msg_twist)

        # publish position to FMS
        msg_agvpos = AGVPosition()
        msg_agvpos.x = self.pose[0]
        msg_agvpos.y = self.pose[1]
        msg_agvpos.theta = self.pose[2]
        self.pub_agvpos.publish(msg_agvpos)
        self.get_logger().info('Publishing: "%s"' % msg_agvpos)

        msg_batterystate = BatteryState()
        msg_batterystate.battery_charge = self.battery_charge
        msg_batterystate.battery_voltage = self.battery_voltage
        self.pub_batterystate.publish(msg_batterystate)
        self.get_logger().info('Publishing: "%s"' % msg_batterystate)

        self.i += 1

    # TODOs
    def calculate_direction(self):
        # calculates direction the robot needs to go to to reach desired graph node, gets robot position, orientation, destination in graph
        pass

    def controller(self):
        # check if robot is still on line
            # if not on line calculate new direction through calculate_direction()
        # publish topic for robot to move
        pass

def main(args=None):

    rclpy.init(args=args)

    worker = Worker()

    rclpy.spin(worker)


if __name__ == '__main__':
    main()
