import math
import threading
import time
from queue import Queue

import mqtt
import vda5050


class AGV:
    def __init__(self, graph, aid: int, color, x, y, heading, battery_level, charging_status, velocity, last_node_id,
                 driving_status, connection_status):
        self.graph = graph
        self.aid = aid
        self.order = None
        self.x = x
        self.y = y
        self.heading = heading
        self.battery_level = battery_level
        self.charging_status = charging_status
        self.velocity = velocity
        self.color = color
        self.last_node_id = last_node_id
        self.driving_status = driving_status
        self.connection_status = connection_status
        self.pending_orders = Queue()
        self.lock = threading.Lock()

    def order_executor_thread(self):
        while True:
            print("AGV " + str(self.aid) + " order executor thread is online " + str(self) + ' ' + str(self.pending_orders))
            next_order = self.pending_orders.get()
            print("AGV is now starting on new order")

            self.lock.acquire()
            print("AGV has gotten lock")
            self.order = next_order
            self.order.agv = self
            print("AGV " + str(self.aid) + " has a new order, executing now...")
            while self.order.extension_required(self.x, self.y):
                self.order.try_extension(self.x, self.y)
            self.lock.release()

            self.order.sem.acquire()
            print("AGV " + str(self.aid) + " has finished order")
            time.sleep(1)

    def has_order(self):
        # Indicates if an AGV is currently executing an order
        return self.order is not None

    def update_position(self, x, y, heading=None):
        self.x = x
        self.y = y
        self.heading = heading
        self.lock.acquire()
        if self.order is not None:
            while self.order.extension_required(self.x, self.y):
                self.order.try_extension(self.x, self.y)
        self.lock.release()

    def update_battery_level(self, battery, heading=None):
        self.battery_level = battery
        self.heading = heading

    def update_charging_status(self, charging_status, heading=None):
        self.charging_status = charging_status
        self.heading = heading

    def update_velocity(self, velocity, heading=None):
        self.velocity = velocity
        self.heading = heading

    def update_last_nodeid(self, last_node_id, heading=None):
        if last_node_id == '':
            return
        self.last_node_id = last_node_id
        self.heading = heading
        self.lock.acquire()
        if self.order is not None:
            self.order.update_last_node(last_node_id, (self.x, self.y))

            while self.order.extension_required(self.x, self.y):
                self.order.try_extension(self.x, self.y)
        self.lock.release()

    def update_driving_status(self, driving_status, heading=None):
        self.driving_status = driving_status
        self.heading = heading

    def update_connection_status(self, connection_status, heading=None):
        self.connection_status = connection_status
        self.heading = heading
