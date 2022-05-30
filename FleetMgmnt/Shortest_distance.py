import turtlegraph
import math
import sys
import copy


class ShortestPath:
    def __init__(self,graph: turtlegraph.Graph,sourcenode : turtlegraph.Node ):
        self.Graph = graph
        self.SourceNode = sourcenode
        self.visited_and_distance = [[None,sys.maxsize,[turtlegraph.Edge]]]
        self.unvisited_nodes = self.Graph.nodes
        self.visitedNode = []
        self.number_of_vertices = len(self.Graph.nodes)
    def Process_ShortestPath(self):
        for i in range(self.number_of_vertices - 1):
            self.visited_and_distance.append([None, sys.maxsize,[turtlegraph.Edge]])

        self.visited_and_distance[0] = [self.SourceNode,0,[]]
        self.visitedNode.append(self.SourceNode)
        NeighPath = self.NeighbourNodePaths(self.visited_and_distance[0][0])

        for k in range(0,self.number_of_vertices-1):

            NeighPath1 = self.NeighbourNodePaths(self.visited_and_distance[k][0])
            for i in range(0, len(NeighPath1)):
                for Nodes in self.visitedNode:
                    if NeighPath1[i][0] == Nodes:
                        NeighPath1[i][0] = None
                        NeighPath1[i][1] = sys.maxsize

            for paths in NeighPath1:
                if paths[0] != None:
                    His = []
                    His.append(self.visited_and_distance[k][0])
                    temp = [paths[0], self.visited_and_distance[k][1] + paths[1]]
                    NeighPath.append(temp)

            temp = self.FindShortPath(NeighPath1)
            His = [self.visited_and_distance[k][0]]
            temp1 = [temp[0], self.visited_and_distance[k][1] + temp[1]]

            NeighPath_temp = copy.copy(NeighPath)
            bool_mvk = False

            while bool_mvk == False:
                visit = self.cmp_FindShortPath(NeighPath_temp, temp1)
                for nodes in self.visitedNode:
                    if visit[0] == nodes:
                        NeighPath_temp.remove(visit)
                        bool_mvk = False
                        break
                    else:
                        bool_mvk = True

            if len(NeighPath_temp) > 0:
                visit = self.cmp_FindShortPath(NeighPath_temp, temp1)


            if visit != temp1:
                self.visited_and_distance[k+1] = [visit[0], visit[1]]
                NeighPath.append(temp1)
                NeighPath.remove(visit)
            else:
                self.visited_and_distance[k+1] = [visit[0], visit[1]]
                NeighPath.remove(visit)

            self.visitedNode.append(visit[0])

            b_set = set(tuple(x) for x in NeighPath)
            NeighPath = [list(x) for x in b_set]

        return self.visited_and_distance


    def NeighbourNodePaths(self,Node: turtlegraph.Node):
        Ret = []
        Nodelis = self.Attched_Nodes(Node)
        for Node_obj in Nodelis:
            Ret.append([Node_obj,Distance_Between_Nodes(Node,Node_obj),self.FindEdgewithNodes(Node,Node_obj)])
        return Ret

    def FindEdgewithNodes(self,Node1: turtlegraph.Node,Node2: turtlegraph.Node):
        for edg in self.Graph.edges:
            if (edg.start == Node1) and (edg.end == Node2) :
                Edge = edg
        return Edge

    def cmp_FindShortPath(self,neighpathlist,vistedpath):
        if len(neighpathlist) > 1:
            minpath = min(neighpathlist, key=lambda x: x[1])
        elif len(neighpathlist) == 0:
            minpath = [sys.maxsize,sys.maxsize]
        else:
            minpath = neighpathlist[0]

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












