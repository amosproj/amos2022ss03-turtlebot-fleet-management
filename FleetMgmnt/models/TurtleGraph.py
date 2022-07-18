import io
import json
import math
import numpy as np
import time
import threading
from queue import Queue
from typing import List

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("Agg")
import matplotlib.style as mpls
mpls.use("fast")

import shapely.geometry

# TODO Fix import paths
import vmap_importer
import graph_search as gs
import vda5050
import collavoid

from models.Order import Order, OrderType, OrderStatus
from models.Edge import Edge
from models.Node import Node
from models.AGV import AGV



class Graph:
    def __init__(self):
        self.nodes = list()
        self.edges = list()
        self.agvs = list()
        self.node_id = 0
        self.edge_id = 0
        self.agv_id = 0
        self.graph_search = gs.GraphSearch(self)
        self.pending_orders = Queue()
        self.all_orders = list()
        self.completed_orders = list()
        self.lock = threading.Lock()
        self.image = self.create_image()

    def vmap_lines_to_graph(self, file: str):
        points, lines = vmap_importer.import_vmap(file)
        for i, point in enumerate(points):
            if point.name.isdigit():
                self.new_node(point.x, point.y)
            else:
                self.new_node(point.x, point.y, point.name)
        for line in lines:
            start = self.find_node_by_coords(line.start.x, line.start.y)
            end = self.find_node_by_coords(line.end.x, line.end.y)
            self.new_edge(
                start, end, math.dist(line.start.get_coords(), line.end.get_coords())
            )
            self.new_edge(
                end, start, math.dist(line.start.get_coords(), line.end.get_coords())
            )

    def append_new_order(self, start_node_id: str, end_node_id: str, agv_id: str = None):
        start = self.find_node_by_id(int(start_node_id))
        end = self.find_node_by_id(int(end_node_id))
        order = Order(self, start, end)
        if agv_id is None:
            order.agv = None
            self.pending_orders.put(order)
        elif 'AUTO' in agv_id:
            order.agv = self.get_agv_by_id(int(agv_id[-1]))
            self.pending_orders.put(order)
        else:
            # If an agv is already assigned to the order, set this field in order
            agv = self.get_agv_by_id(int(agv_id))
            order.agv = agv

            distance = 0
            if agv.x is not None:
                distance = math.dist((agv.x, agv.y), (order.start.x, order.start.y))
            if distance > 1:
                nearest = self.get_nearest_node_from_agv(agv)
                reloc_order = Order(self, nearest, order.start, OrderType.RELOCATION)
                agv.pending_orders.put(reloc_order)
                # print("Relocation order also created")
            agv.pending_orders.put(order)
            # print("Order assigned directly to agv")
        return "Success"

    def new_node(self, x: float, y: float, name: str = None):
        n_node = Node(self.node_id, x, y, name)
        self.node_id += 1
        self.nodes.append(n_node)
        return n_node

    def new_edge(self, start: Node, end: Node, length: float):
        n_edge = Edge(self.edge_id, start, end, length)
        self.edge_id += 1
        self.edges.append(n_edge)
        return n_edge

    def new_agv(self, serial: int, color: str, x=None, y=None, heading=None, agv_status=None, battery_level=None, charging_status=None, velocity=None, last_node_id=None, driving_status=None, connection_status=None):
        n_agv = AGV(self, serial, color, x, y, heading, battery_level, charging_status, velocity, last_node_id, driving_status, connection_status)
        self.agvs.append(n_agv)
        return n_agv

    def find_node_by_id(self, nid: int):
        for node in self.nodes:
            if node.nid == nid:
                return node
        raise Exception("Node not found, FATAL")

    def find_node_by_coords(self, x: float, y: float):
        for node in self.nodes:
            if node.x == x and node.y == y:
                return node
        raise Exception("Node not found, FATAL")

    def get_agv_by_id(self, aid: int):
        # print("Our agvs " + str(self.agvs))
        for agv in self.agvs:
            if agv.aid == aid:
                return agv
        raise Exception("AGV not found, FATAL")

    def get_order_by_id(self, order_id: int):
        for order in self.all_orders:
            if order.order_id == order_id:
                return order
        raise Exception("Order not found, FATAL")

    def get_free_agvs(self) -> List[AGV]:
        # Returns a list of all agvs, which are currently not executing an order
        free_agvs = []
        for agv in self.agvs:
            if agv.connection_status == 'ONLINE' and not agv.has_order():
                free_agvs.append(agv)
        return free_agvs

    def get_nearest_free_agv(self, node: Node) -> AGV:
        min_dist = math.inf
        nearest_agv = None
        for agv in self.get_free_agvs():
            dist = math.sqrt((agv.x - node.x)**2 + (agv.y - node.y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_agv = agv
        return nearest_agv

    def get_nearest_node_from_agv(self, agv: AGV) -> Node:
        min_dist = math.inf
        nearest_node = None
        # Is there a more efficient way than iterating over all the nodes?
        for node in self.nodes:
            dist = math.sqrt((agv.x - node.x)**2 + (agv.y - node.y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def find_nodes_for_colocking(self, polygon: shapely.geometry.Polygon) -> List[Node]:
        result = list()
        for node in self.nodes:
            if polygon.intersects(node.buffer):
                result.append(node)
        return result

    def next_node_critical_path_membership(self, node: Node, order: Order) -> List[Node]:
        # return [node]
        order_path_buffer = collavoid.get_path_safety_buffer_polygon((order.agv.x, order.agv.y),
                                                                     order.get_nodes_to_drive())
        critical_path_buffer = None

        for order2 in self.all_orders:
            if order2.status != OrderStatus.ACTIVE:
                # Calculate only critical paths with active orders
                continue
            if order2.order_id == order.order_id:
                # Order shouldn't have critical path with itself
                continue

            order2_path_buffer = collavoid.get_path_safety_buffer_polygon((order2.agv.x, order2.agv.y),
                                                                          order2.get_nodes_to_drive())

            intersection = order_path_buffer.intersection(order2_path_buffer)

            if node.buffer.intersects(intersection):
                if critical_path_buffer is None:
                    critical_path_buffer = intersection
                else:
                    critical_path_buffer = critical_path_buffer.union(intersection)

        if critical_path_buffer is None:
            return [node]

        critical_path = []
        for n in self.nodes:
            if n.buffer.intersects(critical_path_buffer):
                critical_path.append(n)

        return critical_path

    def bfs(self, start: Node):
        q = [start]
        visited = [start]
        while len(q) > 0:
            cur = q.pop()
            cur_edges = self.get_node_edges(cur)
            for edge in cur_edges:
                if edge.end not in visited:
                    q.append(edge.end)
                    visited.append(edge.end)
        return visited

    def is_strongly_connected(self):
        num_of_nodes = len(self.nodes)
        for node in self.nodes:
            if num_of_nodes != len(self.bfs(node)):
                return False
        return True

    def get_node_edges(self, node: Node):
        node_edges = list()
        for e in self.edges:
            if e.start == node:
                node_edges.append(e)
        return node_edges

    def get_stations(self):
        stations = list()
        for n in self.nodes:
            if n.name is not None:
                stations.append(n)
        return stations

    def get_agvs(self):
        agvs = list()
        for agv in self.agvs:
            if agv.aid is not None:
                agvs.append(agv)
        return agvs

    def get_active_orders(self):
        # return []
        orders = list()
        # Alternative: Iterate over agvs and get the orders, more efficient but probably higher error potential
        for order in self.all_orders:
            if order.status == OrderStatus.ACTIVE:
                orders.append(order)
        return orders

    def create_image(self):
        # drawing the edeges and saving fig to png takes most of the time
        fig1, ax1 = plt.subplots()
        plt_io = io.BytesIO()
        for edge in self.edges:
            ax1.plot(
                [edge.start.x, edge.end.x],
                [edge.start.y, edge.end.y],
                color="gray"
            )
            ax1.plot(
                edge.start.x, edge.start.y,
                marker='.',
                color="gray"
            )
            ax1.plot(
                edge.end.x, edge.end.y,
                marker='.',
                color="gray"
            )
        self.lock.acquire()
        for node in self.nodes:
            if node.name is not None:
                ax1.plot(
                    node.x, node.y,
                    marker='D',
                    color="red"
                )
                ax1.annotate(node.name + " (" + str(node.nid) + ")", (node.x, node.y))
            if node.lock != -1:
                ax1.plot(
                    node.x, node.y,
                    marker='.',
                    color="red"
                )
        self.lock.release()
        for agv in self.agvs:
            if agv.x is not None and agv.y is not None:
                ax1.plot(
                    agv.x, agv.y,
                    marker='s',
                    color=agv.color
                )
            if agv.order is not None:
                x, y = agv.order.get_cosp().exterior.xy
                ax1.plot(x, y, color=agv.color)
        for cur_order in self.get_active_orders():
            color = cur_order.agv.color
            # for edge in cur_order.edges:
            #     start = self.find_node_by_id(int(edge.startNodeId))
            #     end = self.find_node_by_id(int(edge.endNodeId))
            #     if edge.released:
            #         ax1.plot(
            #             [start.x, end.x],
            #             [start.y, end.y],
            #             color=color,
            #         )
            #     else:
            #         ax1.plot(
            #             [start.x, end.x],
            #             [start.y, end.y],
            #             color=color,
            #             linestyle="--",
            #             alpha=0.5
            #         )
        ax1.get_xaxis().set_visible(False)
        ax1.get_yaxis().set_visible(False)
        fig1.savefig(plt_io, format="png", dpi=300, bbox_inches='tight')
        plt.close(fig1)
        return plt_io

    def create_map_thread(self):
        while True:
            start = time.time()
            self.image = self.create_image()
            end = time.time()
            time.sleep(1)
            # print("Map rendered in " + str(end-start))

    def create_json(self):
        n = list()
        for node in self.nodes:
            n.append(json.loads(node.json()))
        orders = list()
        for order in self.all_orders:
            orders.append(order.serialize())
        return json.dumps({"nodes": n, "orders": orders}, indent=4)

    def get_shortest_route(self, start: Node, target: Node) -> (List[Node], List[Edge]):
        return self.graph_search.get_shortest_route(start, target)
