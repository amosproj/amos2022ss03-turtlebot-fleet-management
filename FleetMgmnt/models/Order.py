import math
import threading
from enum import Enum
from typing import List

import shapely.geometry

import collavoid
import mqtt
import vda5050
from models import Node
from models.AGV import AGV
from models.Node import SAFETY_BUFFER_NODE


order_id_counter = 0
order_id_lock = threading.Lock()


class OrderStatus(int, Enum):
    CREATED = 0
    ASSIGNED = 1
    ACTIVE = 2
    WAITING = 3
    COMPLETED = 4


class OrderType(Enum):
    NORMAL = 0
    RELOCATION = 1


class Order:

    def __init__(self, graph, start: Node.Node, end: Node.Node, order_type: OrderType = OrderType.NORMAL):
        global order_id_counter
        self.graph = graph
        with order_id_lock:
            self.order_id = order_id_counter
            order_id_counter += 1
        self.order_update_id = 0
        self.status = OrderStatus.CREATED
        self.order_type = order_type
        self.start = start
        self.end = end
        self.completed = list()
        self.base = list()
        self.horizon, _ = self.graph.get_shortest_route(start, end)
        self.sem = threading.Semaphore(0)
        self.agv = None
        graph.all_orders.append(self)

    def create_vda5050_message(self, agv: AGV):
        nodes = self.completed.copy()
        nodes.extend(self.base)
        # print(self.order_update_id)
        self.order_update_id += 1
        return self.graph.create_vda5050_order(nodes, [], str(agv.aid), self.order_id, self.order_update_id, self.horizon)
        # TODO move the complete vda5050 message creation to this method ?

    def update_last_node(self, nid: str, pos: (float, float)):
        print("Last node " + str(nid))

        last_node = self.graph.find_node_by_id(int(nid))
        if last_node is None or \
                last_node in self.completed or \
                self.status == OrderStatus.COMPLETED or \
                last_node not in self.base:
            return
        if last_node == self.end and math.dist((self.end.x, self.end.y), (self.agv.x, self.agv.y)) < 0.3:
            self.graph.lock.acquire()
            self.unlock_all()
            self.graph.lock.release()
            self.status = OrderStatus.COMPLETED
            self.sem.release()
            return

        base_position = self.base.index(last_node)
        for i in reversed(range(base_position + 1)):
            head = self.base[i]
            distance = math.dist((head.x, head.y), pos)
            if distance > 0.4:
                self.base.remove(head)
                print("Removing " + str(head.nid))
                self.completed.append(head)
            else:
                print("Not removing " + str(head.nid) + " because dist " + str(distance))
        self.graph.lock.acquire()
        self.unlock_all()
        self.lock_all()
        self.graph.lock.release()

    # COSP = Current Order Safety Polygon
    # AGV position + Base
    def get_cosp(self, virtual_ext = list()):
        base_copy = self.base.copy()
        base_copy.extend(virtual_ext)
        return collavoid.get_path_safety_buffer_polygon((self.agv.x, self.agv.y), base_copy)

    def unlock_all(self):
        for node in self.graph.nodes:
            node.release(self.order_id)

    def lock_all(self):
        for node in self.base:
            if not node.try_lock(self.order_id):
                raise Exception
        for node in self.graph.find_nodes_for_colocking(self.get_cosp()):
            if not node.try_lock(self.order_id):
                raise Exception

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
        next_nodes = self.graph.next_node_critical_path_membership(next_node, self.order_id)

        virtual_cosp = self.get_cosp(next_nodes)

        self.graph.lock.acquire()
        success = True
        for node in self.graph.find_nodes_for_colocking(virtual_cosp):
            if not node.try_lock(self.order_id):
                success = False
                break

        if success:
            self.base.append(next_node)
            if next_node in self.horizon:
                self.horizon.remove(next_node)

        self.unlock_all()
        self.lock_all()
        self.graph.lock.release()

        mqtt.client.publish(vda5050.get_mqtt_topic(str(self.agv.aid), vda5050.Topic.ORDER),
                                 self.create_vda5050_message(self.agv).json(), 2)

        return True

    def get_nodes_to_drive(self):
        # Should return all nodes that are not passed yet.
        all_nodes = self.base + self.horizon
        return list(set(all_nodes) - set(self.completed))

