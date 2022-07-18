import unittest

import main  # Needed to prevent circular imports
from models import TurtleGraph, Order
import vmap_importer
import matplotlib.pyplot as plt


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


class TestNextNodeCriticalPathMembership(unittest.TestCase):
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
        critical_path = self.graph.next_node_critical_path_membership(self.nodes[1], self.order1)
        self.assertEqual(len(critical_path), 1)
        self.assertTrue(self.nodes[1] in critical_path)

    def test_critical_path(self):
        self.graph.all_orders = [self.order1, self.order2]
        critical_path = self.graph.next_node_critical_path_membership(self.nodes[0], self.order1)
        self.assertEqual(len(critical_path), 2)
        self.assertTrue(self.nodes[0] in critical_path)
        self.assertTrue(self.nodes[4] in critical_path)

    def test_three_orders(self):
        self.graph.all_orders = [self.order0, self.order1, self.order2]
        # No critical path
        cp1 = self.graph.next_node_critical_path_membership(self.nodes[1], self.order1)
        self.assertEqual(len(cp1), 1)
        self.assertTrue(self.nodes[1] in cp1)
        # Critical path
        cp2 = self.graph.next_node_critical_path_membership(self.nodes[5], self.order2)
        self.assertTrue(self.nodes[5] in cp2)
        self.assertTrue(self.nodes[11] in cp2)
        self.assertFalse(self.nodes[0] in cp2)
        self.assertEqual(len(cp2), 2)


class TestNextNodeCriticalPathMembershipCurvedGraph(unittest.TestCase):
    def setUp(self):
        self.graph = TurtleGraph.Graph()
        self.graph.vmap_lines_to_graph("maps/room_04.150_curved_old.vmap")
        self.nodes = {}
        # Create dict with nodes for easier access
        for n in self.graph.nodes:
            self.nodes[n.nid] = n
        self.order0 = create_mock_order(self.graph, self.nodes[62], self.nodes[53])
        self.order1 = create_mock_order(self.graph, self.nodes[138], self.nodes[127])
        self.order2 = create_mock_order(self.graph, self.nodes[53], self.nodes[111])

    def test_no_critical_path(self):
        self.graph.all_orders = [self.order1, self.order2]
        critical_path = self.graph.next_node_critical_path_membership(self.nodes[138], self.order1)
        self.assertEqual(len(critical_path), 1)
        self.assertTrue(self.nodes[138] in critical_path)

    def test_critical_path(self):
        self.graph.all_orders = [self.order0, self.order1]
        critical_path = self.graph.next_node_critical_path_membership(self.nodes[2], self.order1)
        # Visualization of critical path for debug and understanding issues
        # visualize_path(self.graph, critical_path, [self.nodes[2]])
        self.assertTrue(len(critical_path) > 5)
        self.assertTrue(self.nodes[2] in critical_path)

    def test_critical_path2(self):
        self.graph.all_orders = [self.order0, self.order1]
        critical_path = self.graph.next_node_critical_path_membership(self.nodes[53], self.order2)
        # visualize_path(self.graph, critical_path)
        self.assertTrue(len(critical_path) > 1)
        self.assertTrue(self.nodes[53] in critical_path)
        self.assertFalse(self.nodes[111] in critical_path)


if __name__ == '__main__':
    # Tests somehow doesn't pass when executing all at once. But if you run it one by one all the tests pass
    unittest.main()
