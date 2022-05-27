import turtlegraph
import math
import sys


class ShortestPath:
    def __init__(self,graph: turtlegraph.Graph,sourcenode : turtlegraph.Node ):
        self.Graph = graph
        self.SourceNode = sourcenode
        self.visited_and_distance = [[None,sys.maxsize]]
        self.unvisited_nodes = self.Graph.nodes
        self.visitedNode = []
        self.number_of_vertices = len(self.Graph.nodes)
    def Process_ShortestPath(self):
        for i in range(self.number_of_vertices - 1):
            self.visited_and_distance.append([None, sys.maxsize])

        self.visited_and_distance[0] = [self.SourceNode,0]
        self.visitedNode.append(self.SourceNode)
        NeighPath = self.NeighbourNodePaths(self.visited_and_distance[0][0])
        visit = self.FindShortPath(NeighPath)
        NeighPath.remove(visit)
        self.visited_and_distance[1] = [visit[0], self.visited_and_distance[0][1]+visit[1]]
        self.visitedNode.append(visit[0])
        NeighPath1 = self.NeighbourNodePaths(self.visited_and_distance[1][0])

        for paths in NeighPath1:
            for node in self.visitedNode:
                if paths[0] == node:
                    NeighPath1.remove(paths)

        temp = self.FindShortPath(NeighPath1)
        temp1 = [temp[0], self.visited_and_distance[1][1]+temp[1]]

        visit = self.cmp_FindShortPath(NeighPath,temp1)
        if visit != temp1:
            self.visited_and_distance[2] = [visit[0], self.visited_and_distance[0][1] + visit[1]]
        else:
            self.visited_and_distance[2] = [visit[0], visit[1]]
        self.visitedNode.append(visit[0])

        return self.visited_and_distance

    def NeighbourNodePaths(self,Node: turtlegraph.Node):
        Ret = []
        Nodelis = self.Attched_Nodes(Node)
        for Node_obj in Nodelis:
            Ret.append([Node_obj,Distance_Between_Nodes(Node,Node_obj)])
        return Ret

    def cmp_FindShortPath(self,neighpathlist,vistedpath):
        minpath = min(neighpathlist, key=lambda x: x[1])

        if minpath[1] < vistedpath[1]:
            return minpath
        else:
            return vistedpath

    def FindShortPath(self,neighpathlist):
        minpath = min(neighpathlist, key=lambda x: x[1])
        return minpath



    def Attched_Nodes(self,Node : turtlegraph.Node):
        Edges = self.Graph.get_node_edges(Node)
        Nodes = []
        for edge in Edges:
            Nodes.append(edge.end)
            Nodes.append(edge.start)
        new_list = list(set(Nodes))
        new_list.remove(Node)
        return new_list

    def Nearest_Node_for_A_Node(self,Node: turtlegraph.Node,Nodelist: list[turtlegraph.Node]):
        Temp = []
        Distance_dic = {}
        Distances = []
        for nodes in Nodelist:
            dis = Distance_Between_Nodes(nodes, Node)
            Distance_dic[dis] = nodes
            Distances.append(dis)

        Distances.sort()
        Node1 = key_for_value(Distance_dic, Distances[0])
        return Node1,Distances[0]






def Distance_Between_Nodes(Node1,Node2):
    return math.sqrt((Node1.x - Node2.x)**2 + (Node1.y-Node2.y)**2)

def key_for_value(d, Key):
    """Return a key in `d` having a value of `value`."""
    for k, v in d.items():
        if k == Key:
            return v



graph = turtlegraph.Graph()
graph.vmap_lines_to_graph("Test1.vmap")

shortest_00 = ShortestPath(graph,graph.nodes[0])

temp = shortest_00.Process_ShortestPath()

print(temp[0][0].nid,temp[0][1])
print(temp[1][0].nid,temp[1][1])
print(temp[2][0].nid,temp[2][1])


print(temp)







