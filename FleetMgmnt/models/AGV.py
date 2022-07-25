import threading
import time
from queue import Queue


class AGV:
    def __init__(self, graph, aid: int, color: str, x: float = None, y: float = None, heading: float = None):
        self.graph = graph
        self.aid = aid
        self.color = color
        self.x = x
        self.y = y
        self.heading = heading
        self.order = None
        self.battery_level = None
        self.charging_status = None
        self.charging_prepare = False
        self.velocity = None
        self.last_node_id = None
        self.driving_status = None
        self.connection_status = 'OFFLINE'
        self.pending_orders = Queue()
        self.lock = threading.Lock()

    def has_order(self):
        # Indicates if an AGV is currently executing an order
        return self.order is not None

    def update_position(self, x: float, y: float, heading: float = None):
        self.x = x
        self.y = y
        self.heading = heading
        self.lock.acquire()
        # if self.order is not None:
        #    while self.order.extension_required(self.x, self.y):
        #        self.order.try_extension(self.x, self.y)
        self.lock.release()

    def update_battery_level(self, battery: float):
        self.battery_level = battery

    def update_charging_status(self, charging_status: str):
        self.charging_status = charging_status

    def update_velocity(self, velocity: float):
        self.velocity = velocity

    def update_last_node_id(self, last_node_id: int):
        if last_node_id == '':
            return
        self.last_node_id = last_node_id
        self.lock.acquire()
        if self.order is not None:
            self.order.update_last_node(last_node_id, (self.x, self.y))

            while self.order.extension_required(self.x, self.y):
                self.order.try_extension(self.x, self.y)
        self.lock.release()

    def update_driving_status(self, driving_status: str):
        self.driving_status = driving_status

    def update_connection_status(self, connection_status: str):
        self.connection_status = connection_status
