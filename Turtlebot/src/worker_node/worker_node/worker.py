import rclpy
from rclpy.node import Node

class WorkerSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

class WorkerPublisher(Node):
    
    def __init__(self):
        super().__init__('minimal_publisher')



#calculates direction the robot needs to go to to reach desired graph node, gets robot position, orientation, destination in graph
def calculate_direction(self):
    pass
        
def controller(self):
    #needs line measurement topic
    #check if robot is still on line
        #if not on line calculate new direction through calculate_direction()
    #publish topic for robot to move
    pass

def main(args=None):

    rclpy.init(args=args)

    work_subscriber = WorkSubscriber()

    work_publisher = WorkPublisher()

    

