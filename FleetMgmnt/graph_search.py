import math

import turtlegraph
from astar import AStar  # https://github.com/jrialland/python-astar


class AStarSearch(AStar):

    def __init__(self, graph: turtlegraph.Graph):
        self.graph = graph

    def neighbors(self, node: turtlegraph.Node):
        """ For a given node, returns the list of its neighbors. """
        edges = self.graph.get_node_edges(node)
        neighbors = [e.end for e in edges]
        return neighbors

    def distance_between(self, n1: turtlegraph.Node, n2: turtlegraph.Node):
        """ Gives the real distance/cost between two adjacent nodes n1 and n2. """
        edges = self.graph.get_node_edges(n1)
        for e in edges:
            if e.end.nid == n2.nid:
                dist = e.length
                break
        return dist

    def heuristic_cost_estimate(self, current: turtlegraph.Node, goal: turtlegraph.Node):
        """ Computes the estimated distance/cost between a node and the goal. """
        return math.sqrt((current.x - goal.x) ** 2 + (current.y - goal.y) ** 2)

    def is_goal_reached(self, current: turtlegraph.Node, goal: turtlegraph.Node):
        """ This method shall return a truthy value when the goal is 'reached'. """
        return current.nid == goal.nid


class GraphSearch:

    def __init__(self, graph: turtlegraph.Graph):
        self.graph = graph
        self.a_star_search = AStarSearch(graph)
        self.shortest_routes = dict()  # Stores the shortest route for already calculated pairs of (start, end)

    def get_shortest_route(self, start: turtlegraph.Node, end: turtlegraph.Node):
        key = (start.nid, end.nid)
        if key not in self.shortest_routes.keys():
            self.shortest_routes[key] = self.calculate_shortest_route(start, end)
        return self.shortest_routes[key]

    def calculate_shortest_route(self, start: turtlegraph.Node, end: turtlegraph.Node):
        route_nodes = list(self.a_star_search.astar(start, end))  # List of nodes
        route_edges = list()  # List of edges
        for i, node in enumerate(route_nodes[:-1]):
            edges = self.graph.get_node_edges(node)
            for edge in edges:
                if edge.end.nid == route_nodes[i + 1].nid:
                    route_edges.append(edge)
                    continue
        return route_nodes, route_edges


if __name__ == '__main__':
    g = turtlegraph.Graph()
    g.vmap_lines_to_graph("demo.vmap")

    gs = GraphSearch(g)
    nodes, edges = gs.get_shortest_route(g.nodes[2], g.nodes[17])

    for edge in edges:
        print(edge.start.nid, '->', edge.end.nid)
