import math
from astar import AStar


class GraphSearch(AStar):
    """ Uses A* search in order to calculate the shortest paths from a start node to an end node. """

    def __init__(self, graph):
        self.graph = graph
        self.excluded_nodes = list()
        self.shortest_routes = dict()  # Stores the shortest route for already calculated pairs of (start, end)

    def get_shortest_route(self, start, end):
        key = (start.nid, end.nid)
        if key not in self.shortest_routes.keys():
            self.shortest_routes[key] = self.calculate_route(start, end)
        return self.shortest_routes[key]

    def get_alternative_route(self, start, end, excluded_nodes):
        return self.calculate_route(start, end, excluded_nodes)

    def calculate_route(self, start, end, excluded_nodes=None):
        if excluded_nodes is None:
            excluded_nodes = []
        self.excluded_nodes = excluded_nodes

        route = self.astar(start, end)
        if route is None:
            return [], []
        route_nodes = list(route)  # List of nodes
        route_edges = list()  # List of edges
        for i, node in enumerate(route_nodes[:-1]):
            edges = self.graph.get_node_edges(node)
            for edge in edges:
                if edge.end.nid == route_nodes[i + 1].nid:
                    route_edges.append(edge)
                    continue
        return route_nodes, route_edges

    def neighbors(self, node):
        """ [Override] For a given node, returns the list of its neighbors. """
        edges = self.graph.get_node_edges(node)
        neighbors = [e.end for e in edges if e.end not in self.excluded_nodes]
        return neighbors

    def distance_between(self, n1, n2):
        """ [Override] Gives the real distance/cost between two adjacent nodes n1 and n2. """
        edges = self.graph.get_node_edges(n1)
        for e in edges:
            if e.end.nid == n2.nid:
                return e.length
        return math.inf

    def heuristic_cost_estimate(self, current, goal):
        """ [Override] Computes the estimated distance/cost between a node and the goal. """
        return math.sqrt((current.x - goal.x) ** 2 + (current.y - goal.y) ** 2)

    def is_goal_reached(self, current, goal):
        """ [Override] This method shall return a truthy value when the goal is 'reached'. """
        return current.nid == goal.nid
