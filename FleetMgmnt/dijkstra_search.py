import turtlegraph
import math
import sys
import copy


class ShortestPath:
    """ First implementation of dijkstra algorithm for the search of a shortest path (Not used for now) """

    def __init__(self, graph: turtlegraph.Graph, source_node: turtlegraph.Node):
        self.graph = graph
        self.source_node = source_node
        self.visited_and_distance = [[None, sys.maxsize, [turtlegraph.Edge]]]
        self.unvisited_nodes = self.graph.nodes
        self.visited_nodes = []
        self.number_of_vertices = len(self.graph.nodes)

    def process_shortest_path(self):
        for i in range(self.number_of_vertices - 1):
            self.visited_and_distance.append([None, sys.maxsize, [turtlegraph.Edge]])

        self.visited_and_distance[0] = [self.source_node, 0, []]
        self.visited_nodes.append(self.source_node)
        neigh_path = self.neighbour_node_paths(self.visited_and_distance[0][0])

        for k in range(0, self.number_of_vertices-1):
            neigh_path1 = self.neighbour_node_paths(self.visited_and_distance[k][0])
            for i in range(0, len(neigh_path1)):
                for Nodes in self.visited_nodes:
                    if neigh_path1[i][0] == Nodes:
                        neigh_path1[i][0] = None
                        neigh_path1[i][1] = sys.maxsize

            for paths in neigh_path1:
                if paths[0] is not None:
                    his = [self.visited_and_distance[k][0]]
                    temp = [paths[0], self.visited_and_distance[k][1] + paths[1]]
                    neigh_path.append(temp)

            temp = self.find_short_path(neigh_path1)
            his = [self.visited_and_distance[k][0]]
            temp1 = [temp[0], self.visited_and_distance[k][1] + temp[1]]

            neigh_path_temp = copy.copy(neigh_path)
            bool_mvk = False

            while not bool_mvk:
                visit = self.cmp_find_short_path(neigh_path_temp, temp1)
                for nodes in self.visited_nodes:
                    if visit[0] == nodes:
                        neigh_path_temp.remove(visit)
                        bool_mvk = False
                        break
                    else:
                        bool_mvk = True

            visit = -1
            if len(neigh_path_temp) > 0:
                visit = self.cmp_find_short_path(neigh_path_temp, temp1)

            if visit != temp1:
                self.visited_and_distance[k+1] = [visit[0], visit[1]]
                neigh_path.append(temp1)
                neigh_path.remove(visit)
            else:
                self.visited_and_distance[k+1] = [visit[0], visit[1]]
                neigh_path.remove(visit)

            self.visited_nodes.append(visit[0])

            b_set = set(tuple(x) for x in neigh_path)
            neigh_path = [list(x) for x in b_set]

        return self.visited_and_distance

    def neighbour_node_paths(self, node: turtlegraph.Node):
        ret = []
        nodelis = self.attached_nodes(node)
        for node_obj in nodelis:
            ret.append([node_obj, distance_between_nodes(node, node_obj), self.find_edge_with_nodes(node, node_obj)])
        return ret

    def find_edge_with_nodes(self, node1: turtlegraph.Node, node2: turtlegraph.Node):
        for edg in self.graph.edges:
            if (edg.start == node1) and (edg.end == node2):
                found_edge = edg
        return found_edge

    def cmp_find_short_path(self, neigh_path_list, visted_path):
        if len(neigh_path_list) > 1:
            min_path = min(neigh_path_list, key=lambda x: x[1])
        elif len(neigh_path_list) == 0:
            min_path = [sys.maxsize,sys.maxsize]
        else:
            min_path = neigh_path_list[0]

        if min_path[1] < visted_path[1]:
            return min_path
        else:
            return visted_path

    def find_short_path(self, neigh_path_list):
        min_path = min(neigh_path_list, key=lambda x: x[1])
        return min_path

    def attached_nodes(self, node: turtlegraph.Node):
        edges = self.graph.get_node_edges(node)
        nodes = []
        for edge in edges:
            nodes.append(edge.end)
            nodes.append(edge.start)
        new_list = list(set(nodes))
        new_list.remove(node)
        return new_list

    def nearest_node_for_a_node(self, node: turtlegraph.Node, node_list: list[turtlegraph.Node]):
        distance_dic = {}
        distances = []
        for nodes in node_list:
            dis = distance_between_nodes(nodes, node)
            distance_dic[dis] = nodes
            distances.append(dis)

        distances.sort()
        node1 = key_for_value(distance_dic, distances[0])
        return node1, distances[0]


def distance_between_nodes(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y-node2.y)**2)


def key_for_value(d, key):
    """Return a key in `d` having a value of `value`."""
    for k, v in d.items():
        if k == key:
            return v


if __name__ == '__main__':
    graph = turtlegraph.Graph()
    graph.vmap_lines_to_graph("Test1.vmap")

    shortest_00 = ShortestPath(graph, graph.nodes[0])

    temp = shortest_00.process_shortest_path()
    for el in temp:
        print(el[0].nid, ' - ', el[1])












