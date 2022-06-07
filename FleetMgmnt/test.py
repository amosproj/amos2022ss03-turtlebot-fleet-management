import unittest

import turtlegraph
import vmap_importer


class TestVmapImporter(unittest.TestCase):
    def test_vmap_importer(self):
        points, lines = vmap_importer.import_vmap("demo.vmap")
        self.assertEqual(len(points), 15)
        self.assertEqual(len(lines), 16)


class TestTurtlegraph(unittest.TestCase):
    def test_turtlegraph(self):
        graph = turtlegraph.Graph()
        graph.vmap_lines_to_graph("demo.vmap")
        self.assertEqual(len(graph.nodes), 15)
        self.assertEqual(len(graph.edges), 32)

    def test_turtlegraph_strongly_connected(self):
        graph = turtlegraph.Graph()
        graph.vmap_lines_to_graph("demo.vmap")
        self.assertEqual(graph.is_strongly_connected(), True)

    def test_turtlegraph_not_strongly_connected(self):
        graph = turtlegraph.Graph()
        graph.vmap_lines_to_graph("demo.vmap")
        graph.nodes.pop().start = None
        self.assertEqual(graph.is_strongly_connected(), False)


class TestGraphSearch(unittest.TestCase):
    def setUp(self):
        self.graph = turtlegraph.Graph()
        self.graph.vmap_lines_to_graph("demo.vmap")

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


if __name__ == '__main__':
    unittest.main()
