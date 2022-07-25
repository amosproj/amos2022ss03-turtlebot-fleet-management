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

    def order_executor_thread(self):
        while True:
            if self.charging_status == "Charging":
                time.sleep(10)
                continue

            # print("AGV " + str(self.aid) + " order executor thread is online " + str(self) + ' ' + str(self.pending_orders))
            next_order = self.pending_orders.get()
            # print("AGV is now starting on new order")
            if next_order.status != 'CREATED':
                continue

            self.lock.acquire()
            # print("AGV has gotten lock")
            self.order = next_order
            self.order.agv = self
            # print("AGV " + str(self.aid) + " has a new order, executing now...")
            throttle = 0
            while self.order.extension_required(self.x, self.y):
                if self.order.try_extension(self.x, self.y):
                    throttle = 0
                else:
                    throttle += 1
                    if throttle > 20:
                        time.sleep(0.1)
                    if throttle > 100:
                        time.sleep(1)
                        print("Throttling active")
            self.lock.release()

            self.order.sem.acquire()
            # print("AGV " + str(self.aid) + " has finished order")
            time.sleep(1)

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
