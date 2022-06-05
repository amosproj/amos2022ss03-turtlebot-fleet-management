import unittest

import turtlegraph
import vmap_importer


class VmapImporterAndTurtlegraph(unittest.TestCase):
    def test_vmap_importer(self):
        points, lines = vmap_importer.import_vmap("demo.vmap")
        self.assertEqual(len(points), 19)
        self.assertEqual(len(lines), 20)

    def test_turtlegraph(self):
        graph = turtlegraph.Graph()
        graph.vmap_lines_to_graph("demo.vmap")
        self.assertEqual(len(graph.nodes), 19)
        self.assertEqual(len(graph.edges), 40)

    def test_turtlegraph_strongly_connected(self):
        graph = turtlegraph.Graph()
        graph.vmap_lines_to_graph("demo.vmap")
        self.assertEqual(graph.is_strongly_connected(), True)

    def test_turtlegraph_not_strongly_connected(self):
        graph = turtlegraph.Graph()
        graph.vmap_lines_to_graph("demo.vmap")
        graph.nodes.pop().start = None
        self.assertEqual(graph.is_strongly_connected(), False)


if __name__ == '__main__':
    unittest.main()
