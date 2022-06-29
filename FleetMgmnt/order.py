import math
import threading
from enum import Enum

import turtlegraph
import main

RELEASE_DISTANCE = 100


class OrderStatus(Enum):
    CREATED = 0
    ASSIGNED = 1
    ACTIVE = 2
    WAITING = 3
    COMPLETED = 4


class OrderType(Enum):
    NORMAL = 0
    RELOCATION = 1


class Order:
    order_id_counter = 0
    order_id_lock = threading.Lock()

    def __init__(self, start: turtlegraph.Node, end: turtlegraph.Node):
        with Order.order_id_lock:
            self.order_id = Order.order_id_counter
            Order.order_id_counter += 1
        self.order_update_id = 0
        self.status = OrderStatus.CREATED
        self.start = start
        self.end = end
        self.base = list()
        self.horizon, _ = main.graph.get_shortest_route(start, end)

    def extension_required(self, x: float, y: float) -> bool:
        if len(self.base) == 0:
            return True
        last_node = self.base[-1]
        distance = math.dist((x, y), (last_node.x, last_node.y))
        return distance < RELEASE_DISTANCE

    def try_extension(self, x: float, y: float) -> bool:
        next_node = self.horizon[0]
        if not next_node.try_lock():
            return False
        self.base.append(next_node)
        self.horizon.remove(next_node)


