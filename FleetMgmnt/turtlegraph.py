import typing
from typing import List


class Node:
    def __init__(self, nid: int, description: str = None):
        self.nid = nid
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

    def new_node(self):
        node = Node(self.node_id)
        self.node_id += 1
        self.nodes.append(node)
        return node

    def new_edge(self, start: Node, end: Node, length: float):
        n_edge = Edge(self.edge_id, start, end, length)
        self.edge_id += 1
        self.edges.append(n_edge)
        return n_edge

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






graph = Graph()

# Playground

node1 = graph.new_node()
node2 = graph.new_node()
node3 = graph.new_node()
node4 = graph.new_node()

edge1 = graph.new_edge(node1, node3, 15)
edge2 = graph.new_edge(node3, node4, 13)
edge3 = graph.new_edge(node4, node2, 2)
edge4 = graph.new_edge(node1, node2, 8)
edge5 = graph.new_edge(node1, node4, 87)
edge6 = graph.new_edge(node2, node3, 9)



print(graph.nodes)
print(graph.edges)
print(graph.get_node_edges(node1))

for edge in graph.get_node_edges(node1):
    print("Can reach " + str(edge.end.nid))

# graph = [node1, node2, node3, node4]
# visited = list()
# q = list()
# q.append(node1)
# visited.append(node1)
#
# while len(q) > 0:
#     cur = q.pop()
#     print(cur.nid)
#
#     for edge in cur.edges:
#         if edge.start == cur:
#             if edge.end in visited:
#                 continue
#             visited.append(edge.end)
#             q.append(edge.end)
#         else:
#             if edge.start in visited:
#                 continue
#             visited.append(edge.start)
#             q.append(edge.start)
#
#

