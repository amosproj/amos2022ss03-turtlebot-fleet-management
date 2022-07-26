import random
import unittest

import matplotlib.pyplot as plt

import main  # Somehow needed to prevent circular imports
import vmap_importer
from models import TurtleGraph, Order


# ---- Some helper functions ---

def create_mock_order(graph, start, end, agv=True):
    order = Order.Order(graph, start, end)
    order.status = Order.OrderStatus.ACTIVE
    if agv:
        agv = graph.new_agv(0, 'red', start.x, start.y)
        order.agv = agv
    return order


def visualize_path(graph, path, marked=None):
    if marked is None:
        marked = []
    fig1, ax1 = plt.subplots()
    for edge in graph.edges:
        ax1.plot(
            [edge.start.x, edge.end.x], [edge.start.y, edge.end.y],
            color='gray'
        )
        ax1.plot(
            edge.start.x, edge.start.y,
            marker='.', color='gray'
        )
        ax1.plot(
            edge.end.x, edge.end.y,
            marker='.', color='gray'
        )
    for node in path:
        ax1.plot(
            node.x, node.y,
            marker='.', color='blue'
        )
    for node in marked:
        ax1.plot(
            node.x, node.y,
            marker='D', color='red'
        )
        ax1.annotate("(" + str(node.nid) + ")", (node.x, node.y))
    ax1.get_xaxis().set_visible(False)
    ax1.get_yaxis().set_visible(False)
    plt.show()


class TestVmapImporter(unittest.TestCase):
    def setUp(self):
        self.points, self.lines = vmap_importer.import_vmap("maps/demo.vmap")

    def test_vmap_importer(self):
        self.assertEqual(len(self.points), 15)
        self.assertEqual(len(self.lines), 16)


class TestTurtlegraph(unittest.TestCase):
    def setUp(self):
        self.graph = TurtleGraph.Graph()
        self.graph.vmap_lines_to_graph("maps/demo.vmap")

    def test_turtlegraph(self):
        self.assertEqual(len(self.graph.nodes), 15)
        self.assertEqual(len(self.graph.edges), 32)

    def test_turtlegraph_strongly_connected(self):
        self.assertEqual(self.graph.is_strongly_connected(), True)

    def test_turtlegraph_not_strongly_connected(self):
        self.graph.nodes.pop().start = None
        self.assertEqual(self.graph.is_strongly_connected(), False)


class TestGraphSearch(unittest.TestCase):
    def setUp(self):
        self.graph = TurtleGraph.Graph()
        self.graph.vmap_lines_to_graph("maps/demo.vmap")

    def test_path_to_same_node(self):
        start_node = self.graph.find_node_by_id(2)
        end_node = self.graph.find_node_by_id(2)
        nodes, edges = self.graph.get_shortest_route(start_node, end_node)
        self.assertEqual(len(nodes), 1)
        self.assertTrue(start_node in nodes)

    def test_shortest_path_nodes(self):
        start_node = self.graph.find_node_by_id(2)
        end_node = self.graph.find_node_by_id(14)
        nodes, edges = self.graph.get_shortest_route(start_node, end_node)
        self.assertEqual(len(nodes), 5)
        self.assertEqual(nodes[0], start_node)
        self.assertEqual(nodes[-1], end_node)

    def test_shortest_path_edges(self):
        start_node = self.graph.find_node_by_id(2)
        end_node = self.graph.find_node_by_id(14)
        nodes, edges = self.graph.get_shortest_route(start_node, end_node)
        self.assertEqual(len(edges), 4)
        self.assertEqual(edges[0].start, start_node)
        self.assertEqual(edges[-1].end, end_node)


class TestGraphSearch2(unittest.TestCase):
    def setUp(self):
        self.graph = TurtleGraph.Graph()
        self.graph.vmap_lines_to_graph("maps/room_04.150_curved.vmap")

    def test1(self):
        output = False
        self.assertTrue(self.graph.is_strongly_connected())
        samples = 100
        if output:
            print('-- Calculating the shortest route for', samples, 'random samples --')
        for _ in range(samples):
            start_node, end_node = random.choices(self.graph.nodes, k=2)
            if output:
                print(start_node.nid, '->', end_node.nid)
            nodes, edges = self.graph.get_shortest_route(start_node, end_node)
            self.assertTrue(len(nodes) > 0, 'No route found!')
            self.assertEqual(nodes[0], start_node)
            self.assertEqual(nodes[-1], end_node)


class TestAlternativeGraphSearch(unittest.TestCase):
    def setUp(self):
        self.graph = TurtleGraph.Graph()
        self.graph.vmap_lines_to_graph("maps/demo.vmap")

    def test_alternative_route_1(self):
        start_node = self.graph.find_node_by_id(2)
        end_node = self.graph.find_node_by_id(14)
        excluded = [self.graph.find_node_by_id(0), self.graph.find_node_by_id(4)]
        nodes, edges = self.graph.graph_search.get_alternative_route(start_node, end_node, excluded)
        self.assertNotEqual(len(nodes), 5)
        self.assertEqual(edges[0].start, start_node)
        self.assertEqual(edges[-1].end, end_node)
        self.assertEqual(len(nodes), 8)
        self.assertTrue(self.graph.find_node_by_id(11) in nodes)
        self.assertFalse(self.graph.find_node_by_id(0) in nodes)

    def test_alternative_route_2(self):
        start_node = self.graph.find_node_by_id(3)
        end_node = self.graph.find_node_by_id(10)
        excluded = [self.graph.find_node_by_id(5)]
        nodes, edges = self.graph.graph_search.get_alternative_route(start_node, end_node, excluded)
        self.assertTrue(len(nodes) > 4)
        self.assertEqual(edges[0].start, start_node)
        self.assertEqual(edges[-1].end, end_node)
        self.assertEqual(len(nodes), 9)
        self.assertTrue(self.graph.find_node_by_id(13) in nodes)
        self.assertFalse(self.graph.find_node_by_id(5) in nodes)

    def test_shortest_after_alternative(self):
        start_node = self.graph.find_node_by_id(3)
        end_node = self.graph.find_node_by_id(10)
        excluded = [self.graph.find_node_by_id(5)]
        nodes1, edges1 = self.graph.graph_search.get_alternative_route(start_node, end_node, excluded)
        start_node = self.graph.find_node_by_id(3)
        end_node = self.graph.find_node_by_id(10)
        nodes2, edges2 = self.graph.graph_search.get_shortest_route(start_node, end_node)
        self.assertTrue(len(nodes1) > len(nodes2))
        self.assertEqual(edges2[0].start, start_node)
        self.assertEqual(edges2[-1].end, end_node)
        self.assertTrue(self.graph.find_node_by_id(5) in nodes2)


class TestCriticalPath(unittest.TestCase):
    def setUp(self):
        self.graph = TurtleGraph.Graph()
        self.graph.vmap_lines_to_graph("maps/demo.vmap")
        self.nodes = {}
        # Create dict with nodes for easier access
        for n in self.graph.nodes:
            self.nodes[n.nid] = n
        # 5 -> 11 -> 9 -> 8
        self.order0 = create_mock_order(self.graph, self.nodes[5], self.nodes[8])
        # 2 -> 1 -> 0 -> 4 -> 14 -> 7 -> 6
        self.order1 = create_mock_order(self.graph, self.nodes[2], self.nodes[6])
        # 3 -> 0 -> 4 -> 5 -> 11 -> 10
        self.order2 = create_mock_order(self.graph, self.nodes[3], self.nodes[10])

    def test_no_critical_path(self):
        self.graph.all_orders = [self.order0, self.order1]
        critical_path = self.graph.order_critical_path_membership(self.order1)[0]
        self.assertEqual(len(critical_path), 0)

    def test_critical_path(self):
        self.graph.all_orders = [self.order1, self.order2]
        critical_path = self.graph.order_critical_path_membership(self.order1)[0]
        self.assertEqual(len(critical_path), 2)
        self.assertTrue(self.nodes[0] in critical_path)
        self.assertTrue(self.nodes[4] in critical_path)

    def test_three_orders(self):
        self.graph.all_orders = [self.order0, self.order1, self.order2]
        critical_path = self.graph.order_critical_path_membership(self.order2)[0]
        self.assertTrue(self.nodes[5] in critical_path)
        self.assertTrue(self.nodes[0] in critical_path)
        self.assertFalse(self.nodes[14] in critical_path)
        self.assertTrue(len(critical_path) >= 4)


class TestNextNodeCriticalPathMembershipCurvedGraph(unittest.TestCase):
    def setUp(self):
        self.graph = TurtleGraph.Graph()
        self.graph.vmap_lines_to_graph("maps/room_04.150_curved.vmap")
        # Create dict with nodes for easier access
        self.nodes = {}
        for n in self.graph.nodes:
            self.nodes[n.nid] = n
        # charge_2 -> pick_up
        self.order0 = create_mock_order(self.graph, self.nodes[61], self.nodes[52])
        # idle_2 -> dropoff
        self.order1 = create_mock_order(self.graph, self.nodes[138], self.nodes[127])
        # pick_up -> charge_1
        self.order2 = create_mock_order(self.graph, self.nodes[52], self.nodes[118])

    def test_critical_path(self):
        self.graph.all_orders = [self.order0, self.order1]
        critical_path = self.graph.order_critical_path_membership(self.order1)[0]
        # Visualization of critical path for debug and understanding issues
        # visualize_path(self.graph, critical_path, [self.nodes[2]])
        self.assertTrue(len(critical_path) > 5)
        self.assertTrue(self.nodes[44] in critical_path)

    def test_critical_path2(self):
        self.graph.all_orders = [self.order0, self.order1, self.order2]
        critical_path = self.graph.order_critical_path_membership(self.order2)[0]
        # visualize_path(self.graph, critical_path)
        self.assertTrue(len(critical_path) > 1)
        self.assertTrue(self.nodes[52] in critical_path)
        self.assertFalse(self.nodes[118] in critical_path)


if __name__ == '__main__':
    unittest.main()
