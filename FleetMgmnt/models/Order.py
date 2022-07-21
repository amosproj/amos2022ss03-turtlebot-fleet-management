import math
import threading
from enum import Enum

from matplotlib import pyplot as plt

import collavoid
import mqtt
import vda5050
from models.AGV import AGV
from models.Node import Node

order_id_counter = 0
order_id_lock = threading.Lock()


class OrderStatus(str, Enum):
    CREATED = 'CREATED'
    ACTIVE = 'ACTIVE'
    COMPLETED = 'COMPLETED'
    CANCELLED = 'CANCELLED'


class OrderType(str, Enum):
    NORMAL = 'NORMAL'
    RELOCATION = 'RELOCATION'


class Order:

    def __init__(self, graph, start: Node, end: Node, order_type: OrderType = OrderType.NORMAL):
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
        self.lastCosp = start.buffer
        graph.all_orders.append(self)

    def create_vda5050_message(self, agv: AGV):
        nodes = self.completed.copy()
        nodes.extend(self.base)
        self.order_update_id += 1

        vda5050_nodes = []
        for seq_id, n in enumerate(nodes + self.horizon):
            vda5050_nodes.append(vda5050.Node(
                node_id=str(n.nid),
                sequence_id=seq_id,
                released=n not in self.horizon,
                actions=n.actions,
                node_position=vda5050.NodePosition(x=n.x, y=n.y, map_id='0')
            ))
            # print(n.actions)
        vda5050_order = vda5050.OrderMessage(
            headerid=0,
            timestamp='',
            version='',
            manufacturer='',  # All more general information, probably should not be set here
            serialnumber=str(agv.aid),
            order_id=str(self.order_id),
            order_update_id=self.order_update_id,
            nodes=vda5050_nodes,
            edges=[]
        )

        return vda5050_order

    def update_last_node(self, nid: str, pos: (float, float)):
        # print("Last node in order " + str(self.order_id) + " - " + str(nid))

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
            self.complete()
            return

        base_position = self.base.index(last_node)
        for i in reversed(range(base_position + 1)):
            head = self.base[i]
            distance = math.dist((head.x, head.y), pos)
            if distance > 0.4:
                self.base.remove(head)
                # print("Removing " + str(head.nid))
                self.completed.append(head)
            else:
                pass
                # print("Not removing " + str(head.nid) + " because dist " + str(distance))
        self.graph.lock.acquire()
        self.unlock_all()
        self.lock_all()
        self.graph.lock.release()

    # COSP = Current Order Safety Polygon
    # AGV position + Base
    def get_cosp(self, virtual_ext=list()):
        base_copy = self.base.copy()
        base_copy.extend(virtual_ext)
        if self.agv is not None:
            return collavoid.get_path_safety_buffer_polygon((self.agv.x, self.agv.y), base_copy)
        else:
            if len(base_copy) == 0:
                return collavoid.get_path_safety_buffer_polygon((None, None), base_copy)
            else:
                return collavoid.get_path_safety_buffer_polygon((base_copy[0].x, base_copy[0].y), base_copy)

    def unlock_all(self):
        for node in self.graph.nodes:
            node.release(self.order_id)

    def lock_all(self):
        for node in self.base:
            if not node.try_lock(self.order_id):
                raise Exception
        critical_nodes, _ = self.graph.order_critical_path_membership(self)
        in_crit_path = False
        for node in self.graph.find_nodes_for_colocking(self.get_cosp()):
            if node in critical_nodes:
                in_crit_path = True
            if not node.try_lock(self.order_id):
                print("Order " + str(self.order_id) + " tried to lock " + str(node.nid) + " but is locked by " + str(node.lock))
                raise Exception
        if in_crit_path:
            for node in critical_nodes:
                if not node.try_lock(self.order_id):
                    print(
                        "Order " + str(self.order_id) + " tried to lock crit " + str(node.nid) + " but is locked by " + str(
                            node.lock))
                    raise Exception

    def extension_required(self, x: float, y: float) -> bool:
        if self.status == OrderStatus.CREATED:
            self.status = OrderStatus.ACTIVE
        if len(self.horizon) == 0:
            return False
        next_node = self.horizon[0]
        distance = 2
        if x is not None:
            distance = math.dist((x, y), (next_node.x, next_node.y))
        return distance < 1.3

    def try_extension(self, x: float, y: float) -> bool:
        if len(self.horizon) == 0:
            return False

        critical_nodes, _ = self.graph.order_critical_path_membership(self)

        next_node = self.horizon[0]
        virtual_cosp = self.get_cosp([next_node])
        colocking_nodes = self.graph.find_nodes_for_colocking(virtual_cosp)

        nodes_part_of_crit_path = list(set(critical_nodes) & set(colocking_nodes))

        success = True
        if len(nodes_part_of_crit_path) == 0:
            self.graph.lock.acquire()
            for node in colocking_nodes:
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

        else:
            self.graph.lock.acquire()
            # We will now try to lock the entire critical path
            for node in critical_nodes:
                if not node.try_lock(self.order_id):
                    success = False
                    break
            if success:
                for node in self.horizon:
                    co_lock = self.graph.find_nodes_for_colocking(node.buffer)
                    stop = False
                    for co_lock_node in co_lock:
                        if co_lock_node.lock != self.order_id:
                            stop = True
                            break
                    if stop:
                        break

                    if node.lock == self.order_id:
                        self.base.append(node)
                        self.horizon.remove(node)
                    else:
                        break
            self.unlock_all()
            self.lock_all()
            self.graph.lock.release()

            # base_append = list()
            # print(set(critical_nodes) & set(self.horizon))
            # initial = True
            # for node in self.horizon:
            #     if initial or node in critical_nodes:
            #         print("Appending " + str(node) + " is  next")
            #         initial = False
            #         base_append.append(node)
            #     else:
            #         print("Node " + str(node) + " is not next")
            #         break
            # virtual_cosp = self.get_cosp(base_append)
            # colocking_nodes = self.graph.find_nodes_for_colocking(virtual_cosp)
            #
            #
            # for node in colocking_nodes:
            #     if not node.try_lock(self.order_id):
            #         success = False
            #         print("Extension error crit")
            #         break
            # if success:
            #     print("We are appending " + str(base_append))
            #     for node in base_append:
            #         self.base.append(node)
            #         if node in self.horizon:
            #             self.horizon.remove(node)


        # if success:
        if False:
            mqtt.client.publish(vda5050.get_mqtt_topic(str(self.agv.aid), vda5050.Topic.ORDER),
                                self.create_vda5050_message(self.agv).json(), 2)

        return success

    def get_nodes_to_drive(self):
        # Should return all nodes that are not passed yet.
        all_nodes = self.base + self.horizon
        return list(set(all_nodes) - set(self.completed))

    def serialize(self):
        output = {}
        output['id'] = self.order_id
        output['update_id'] = self.order_update_id
        if self.agv is None:
            output['agv'] = '?'
        else:
            output['agv'] = str(self.agv.aid)
        output['status'] = self.status
        output['type'] = self.order_type
        output['start'] = self.start.nid
        output['end'] = self.end.nid
        output['completed'] = Node.node_list_to_id_list(self.completed)
        output['base'] = Node.node_list_to_id_list(self.base)
        output['horizon'] = Node.node_list_to_id_list(self.horizon)
        output['cosp'] = []
        try:
            x_coords = self.get_cosp().exterior.xy[0].tolist()
            y_coords = self.get_cosp().exterior.xy[1].tolist()
            for i, x in enumerate(x_coords):
                output['cosp'].append({'x': x, 'y': y_coords[i]})
        except:
            output['cosp'] = []
        return output

    def complete(self):
        # This function completes the order. The next order assigned to this AGV will start.
        self.status = OrderStatus.COMPLETED
        self.sem.release()

    def cancel(self):
        # Order Cancellation
        # ToDo: Send MQTT message so the order actually stops
        self.unlock_all()
        self.status = OrderStatus.CANCELLED
        self.sem.release()

        if self.agv is not None:
            vda5050_order = vda5050.OrderMessage(
                headerid=0,
                timestamp='',
                version='',
                manufacturer='',  # All more general information, probably should not be set here
                serialnumber=str(self.agv.aid),
                order_id=str(self.order_id),
                order_update_id=self.order_update_id + 1,
                nodes=[],
                edges=[]
            )

        mqtt.client.publish(vda5050.get_mqtt_topic(str(self.agv.aid), vda5050.Topic.ORDER),
                            vda5050_order.json(), 2)

    def resend(self):
        # This will just resend the latest update in case of crash
        mqtt.client.publish(vda5050.get_mqtt_topic(str(self.agv.aid), vda5050.Topic.ORDER),
                            self.create_vda5050_message(self.agv).json(), 2)
