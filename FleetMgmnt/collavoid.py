from typing import List

from shapely.geometry import LineString, Polygon

import main
from models.Node import Node, SAFETY_BUFFER_NODE


def get_path_safety_buffer_polygon(agv_pos: (float, float), path: List[Node]):
    if len(path) == 0:
        raise Exception
    elif len(path) == 1:
        return path[0].buffer

    coordinates = list()
    if agv_pos[0] is not None:
        pass
        # coordinates.append(agv_pos)
    for node in path:
        coordinates.append((node.x, node.y))
    linestring = LineString(coordinates)
    return linestring.buffer(SAFETY_BUFFER_NODE)


def get_nodes_colliding_with_polygon(polygon: Polygon):
    positive = list()
    for node in main.graph.nodes:
        if node.buffer.intersects(polygon):
            positive.append(node)
    return positive
