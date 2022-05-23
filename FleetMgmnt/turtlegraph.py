import io
import math

from matplotlib import pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

import vmap_importer


class Node:
    def __init__(self, nid: int, x: float, y: float,  description: str = None):
        self.nid = nid
        self.x = x
        self.y = y
        self.description = description


class Edge:
    def __init__(self, eid: int, start: Node, end: Node, length: float):
        self.eid = eid
        self.start = start
        self.end = end


class Graph:
    def __init__(self):
        self.nodes = list()
        self.edges = list()
        self.node_id = 0
        self.edge_id = 0

    def vmap_lines_to_graph(self, file: str):
        points, lines = vmap_importer.import_vmap(file)
        print(len(points))
        print(len(lines))
        for i, point in enumerate(points):
            self.new_node(point.x, point.y, point.name)
        for line in lines:
            start = self.find_node_by_coords(line.start.x, line.start.y)
            end = self.find_node_by_coords(line.end.x, line.end.y)
            self.new_edge(start, end, math.dist(line.start.get_coords(), line.end.get_coords()))

    def new_node(self, x: float, y: float, description: str):
        node = Node(self.node_id, x, y, description)
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

    def is_strongly_connected(self):
        # Todo
        for node in self.nodes:
            edges_visited = [node]
        return False

    def get_node_edges(self, node: Node):
        node_edges = list()
        for e in self.edges:
            if e.start == node:
                node_edges.append(e)
        return node_edges

    def create_image(self):
        plt_io = io.BytesIO()
        for edge in self.edges:
            plt.plot([edge.start.x, edge.end.x], [edge.start.y, edge.end.y], linestyle='dashed', marker='s')
        for node in self.nodes:
            plt.annotate(node.description, (node.x, node.y))
        plt.savefig(plt_io, format='png', dpi=300)
        return plt_io

