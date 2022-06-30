from typing import List

from shapely.geometry import LineString

from models.Node import Node
from models.TurtleGraph import SAFETY_BUFFER_NODE


def get_path_safety_buffer_polygon(path: List[Node]):
        coordinates = list()
        for node in path:
            coordinates.append((node.x, node.y))
        linestring = LineString(coordinates)
        return linestring.buffer(SAFETY_BUFFER_NODE)
