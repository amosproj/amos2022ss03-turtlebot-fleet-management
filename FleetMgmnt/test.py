import unittest

import main  # Needed to prevent circular imports
from models import TurtleGraph, Order
import vmap_importer


class TestVmapImporter(unittest.TestCase):
    def test_vmap_importer(self):
        points, lines = vmap_importer.import_vmap("maps/demo.vmap")
        self.assertEqual(len(points), 15)
        self.assertEqual(len(lines), 16)


class TestTurtlegraph(unittest.TestCase):
    def test_turtlegraph(self):
        graph = TurtleGraph.Graph()
        graph.vmap_lines_to_graph("maps/demo.vmap")
        self.assertEqual(len(graph.nodes), 15)
        self.assertEqual(len(graph.edges), 32)

    def test_turtlegraph_strongly_connected(self):
        graph = TurtleGraph.Graph()
        graph.vmap_lines_to_graph("maps/demo.vmap")
        self.assertEqual(graph.is_strongly_connected(), True)

    def test_turtlegraph_not_strongly_connected(self):
        graph = TurtleGraph.Graph()
        graph.vmap_lines_to_graph("maps/demo.vmap")
        graph.nodes.pop().start = None
        self.assertEqual(graph.is_strongly_connected(), False)


class TestGraphSearch(unittest.TestCase):
    def setUp(self):
        self.graph = TurtleGraph.Graph()
        self.graph.vmap_lines_to_graph("maps/demo.vmap")

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


class TestNextNodeCriticalPathMembership(unittest.TestCase):
    def setUp(self):
        self.graph = TurtleGraph.Graph()
        self.graph.vmap_lines_to_graph("maps/demo.vmap")
        self.nodes = {}
        # Create dict with nodes for easier access
        for n in self.graph.nodes:
            self.nodes[n.nid] = n
        # 5 -> 11 -> 9 -> 8
        self.order0 = Order.Order(self.graph, self.nodes[5], self.nodes[8])
        self.order0.status = Order.OrderStatus.ACTIVE
        # 2 -> 1 -> 0 -> 4 -> 14 -> 7 -> 6
        self.order1 = Order.Order(self.graph, self.nodes[2], self.nodes[6])
        self.order1.status = Order.OrderStatus.ACTIVE
        # 3 -> 0 -> 4 -> 5 -> 11 -> 10
        self.order2 = Order.Order(self.graph, self.nodes[3], self.nodes[10])
        self.order2.status = Order.OrderStatus.ACTIVE

    def test_no_critical_path(self):
        self.graph.all_orders = [self.order0, self.order1]
        critical_path = self.graph.next_node_critical_path_membership(self.nodes[1], self.order1)
        self.assertEqual(len(critical_path), 0)

    def test_critical_path(self):
        self.graph.all_orders = [self.order1, self.order2]
        critical_path = self.graph.next_node_critical_path_membership(self.nodes[0], self.order1)
        self.assertEqual(len(critical_path), 2)
        self.assertTrue(self.nodes[0] in critical_path)
        self.assertTrue(self.nodes[4] in critical_path)

    def test_three_orders(self):
        self.graph.all_orders = [self.order0, self.order1, self.order2]
        cp1 = self.graph.next_node_critical_path_membership(self.nodes[1], self.order1)
        self.assertEqual(len(cp1), 0)
        cp2 = self.graph.next_node_critical_path_membership(self.nodes[5], self.order2)
        self.assertTrue(self.nodes[5] in cp2)
        self.assertTrue(self.nodes[11] in cp2)
        self.assertFalse(self.nodes[0] in cp2)
        self.assertEqual(len(cp2), 2)


if __name__ == '__main__':
    unittest.main()
