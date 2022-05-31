import io
import json
import math
from typing import List

import vmap_importer
from matplotlib import pyplot as plt


class Node:
    def __init__(self, nid: int, x: float, y: float):
        self.nid = nid
        self.x = x
        self.y = y

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
        self.node_id = 0
        self.edge_id = 0

    def vmap_lines_to_graph(self, file: str):
        points, lines = vmap_importer.import_vmap(file)
        for i, point in enumerate(points):
            self.new_node(point.x, point.y)
        for line in lines:
            start = self.find_node_by_coords(line.start.x, line.start.y)
            end = self.find_node_by_coords(line.end.x, line.end.y)
            self.new_edge(
                start, end, math.dist(line.start.get_coords(), line.end.get_coords())
            )
            self.new_edge(
                end, start, math.dist(line.start.get_coords(), line.end.get_coords())
            )

    def new_node(self, x: float, y: float):
        node = Node(self.node_id, x, y)
        self.node_id += 1
        self.nodes.append(node)
        return node

    def new_edge(self, start: Node, end: Node, length: float):
        n_edge = Edge(self.edge_id, start, end, length)
        self.edge_id += 1
        self.edges.append(n_edge)
        return n_edge

    def find_node_by_coords(self, x: float, y: float):
        for node in self.nodes:
            if node.x == x and node.y == y:
                return node
        raise Exception("Node not found, FATAL")

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

    def create_image(self):
        plt_io = io.BytesIO()
        for edge in self.edges:
            plt.plot(
                [edge.start.x, edge.end.x],
                [edge.start.y, edge.end.y],
                linestyle="dashed",
                marker="s",
            )
        for node in self.nodes:
            plt.annotate(str(node.nid), (node.x, node.y))
        plt.savefig(plt_io, format="png", dpi=300)
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
        pass
