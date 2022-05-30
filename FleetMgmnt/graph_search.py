# https://github.com/jrialland/python-astar
from astar import AStar
import turtlegraph
import math

class GraphSearch(AStar):

    def __init__(self, graph):
        self.graph = graph

    def neighbors(self, node):
        ''' For a given node, returns the list of its neighbors. '''
        edges = self.graph.get_node_edges(node)
        neighbors = [e.end for e in edges]
        return neighbors
    
    def distance_between(self, n1, n2):
        ''' Gives the real distance/cost between two adjacent nodes n1 and n2. '''
        edges = self.graph.get_node_edges(n1)
        for e in edges:
            if e.end.nid == n2.nid:
                dist = e.length
                break
        return dist

    def heuristic_cost_estimate(self, current, goal):
        ''' Computes the estimated distance/cost between a node and the goal. '''
        return math.sqrt((current.x - goal.x)**2 + (current.y - goal.y)**2)

    def is_goal_reached(self, current, goal):
        ''' This method shall return a truthy value when the goal is 'reached'. '''
        return current.nid == goal.nid


if __name__ == '__main__':
    graph = turtlegraph.Graph()
    graph.vmap_lines_to_graph("Test1.vmap")

    start = graph.nodes[3]
    end = graph.nodes[0]
    found_path = list(GraphSearch(graph).astar(start, end))

    print('Start:', start.nid, '- End:', end.nid)
    for n in found_path:
        print(n.nid)