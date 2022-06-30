import math
import threading
from enum import Enum

import collavoid
from models import Node
import main
from models.AGV import AGV
from models.Node import SAFETY_BUFFER_NODE


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

    def __init__(self, start: Node.Node, end: Node.Node):
        with Order.order_id_lock:
            self.order_id = Order.order_id_counter
            Order.order_id_counter += 1
        self.order_update_id = 0
        self.status = OrderStatus.CREATED
        self.start = start
        self.end = end
        self.completed = list()
        self.base = list()
        self.horizon, _ = main.graph.get_shortest_route(start, end)

    def create_vda5050_message(self, agv: AGV):
        nodes = self.completed.copy()
        nodes.extend(self.base)
        return main.graph.create_vda5050_order(nodes, [], agv.aid)

    def update_last_node(self, nid: str, pos: (float, float)):
        last_node = main.graph.find_node_by_id(int(nid))
        if last_node is None or last_node in self.completed:
            return
        base_position = self.base.index(last_node)
        for i in range(base_position + 1):
            head = self.base[0]
            distance = math.dist((head.x, head.y), pos)
            if distance > SAFETY_BUFFER_NODE * 2:
                removed = self.base.pop()
                self.completed.append(removed)
            else:
                break
        # ToDo: Node Releasing

    def get_base_polygon(self):
        return collavoid.get_path_safety_buffer_polygon(self.base)

    def extension_required(self, x: float, y: float) -> bool:
        if len(self.horizon) == 0:
            return False
        next_node = self.horizon[0]
        distance = math.dist((x, y), (next_node.x, next_node.y))
        return distance < 1

    def try_extension(self, x: float, y: float) -> bool:
        if len(self.horizon) == 0:
            return False
        next_node = self.horizon[0]
        if not next_node.try_lock(self):
            return False
        self.base.append(next_node)
        self.horizon.remove(next_node)
        return True


