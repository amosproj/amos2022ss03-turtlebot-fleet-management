import io
import json
import math
from typing import List

import vmap_importer
import graph_search as gs
import vda5050
from agv import AGV
from matplotlib import pyplot as plt


class Node:
    def __init__(self, nid: int, x: float, y: float, name: str = None):
        self.nid = nid
        self.x = x
        self.y = y
        self.name = name

    def to_dict(self):
        return {k: v for k, v in self.__dict__.items() if v is not None}

    def json(self, pretty: bool = False) -> str:
        if pretty:
            return json.dumps(self.to_dict(), default=lambda o: o.to_dict(), indent=4)
        else:
            return json.dumps(self.to_dict(), default=lambda o: o.to_dict())


class Edge:
    def __init__(self, eid: int, start: Node, end: Node, length: float):
        self.eid = eid
        self.start = start
        self.end = end
        self.length = length

    def json(self):
        return json.dumps(
            {
                "eid": self.eid,
                "start": self.start.nid,
                "end": self.end.nid,
                "length": round(self.length * 100),
            }
        )


class Graph:
    def __init__(self):
        self.nodes = list()
        self.edges = list()
        self.agvs = list()
        self.node_id = 0
        self.edge_id = 0
        self.agv_id = 0
        self.graph_search = gs.GraphSearch(self)

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

    def new_agv(self, x=None, y=None, heading=None):
        n_agv = AGV(self.agv_id, x, y, heading)
        self.agv_id += 1
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
        for agv in self.agvs:
            if agv.aid == aid:
                return agv
        raise Exception("AGV not found, FATAL")

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

    def create_image(self):
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
        for node in self.nodes:
            if node.name is not None:
                ax1.plot(
                    node.x, node.y,
                    marker='D',
                    color="red"
                )
                ax1.annotate(node.name + " (" + str(node.nid) + ")", (node.x, node.y))
        for agv in self.agvs:
            if agv.x is not None and agv.y is not None:
                ax1.plot(
                    agv.x, agv.y,
                    marker='s',
                    color='blue'
                )
        fig1.savefig(plt_io, format="png", dpi=300)
        plt.close(fig1)
        return plt_io

    def create_json(self):
        n = list()
        for node in self.nodes:
            n.append(json.loads(node.json()))
        e = list()
        for edge in self.edges:
            e.append(json.loads(edge.json()))
        return json.dumps({"nodes": n, "edges": e}, indent=4)

    def get_shortest_route(self, start: Node, target: Node) -> (List[Node], List[Edge]):
        return self.graph_search.get_shortest_route(start, target)


def create_vda5050_order(nodes: List[Node], edges: List[Edge]) -> vda5050.OrderMessage:
    vda5050_nodes = []
    for seq_id, n in enumerate(nodes):
        vda5050_nodes.append(vda5050.Node(
            node_id=str(n.nid),
            sequence_id=seq_id,
            released=True,
            actions=[],
            node_position=vda5050.NodePosition(x=n.x, y=n.y, map_id='0')
        ))

    vda5050_edges = []
    for seq_id, e in enumerate(edges):
        vda5050_edges.append(vda5050.Edge(
            edge_id=str(e.eid),
            sequence_id=seq_id,
            released=True,
            start_node_id=str(e.start.nid),
            end_node_id=str(e.end.nid),
            actions=[],
            length=e.length
        ))

    order = vda5050.OrderMessage(
        headerid=0,
        timestamp='',
        version='',
        manufacturer='',
        serialnumber='',  # All more general information, probably should not be set here
        order_id='0',
        order_update_id=0,  # Also, can't be set here
        nodes=vda5050_nodes,
        edges=vda5050_edges
    )

    return order

