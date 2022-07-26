import json

from models.Node import Node


""" Contains the state of a edge between two nodes. """
class Edge:
    """ Represents an edge between two nodes in the graph. """

    def __init__(self, eid: int, start: Node, end: Node, length: float):
        self.eid = eid
        self.start = start
        self.end = end
        self.length = length

    def json(self) -> str:
        return json.dumps(
            {
                "eid": self.eid,
                "start": self.start.nid,
                "end": self.end.nid,
                "length": round(self.length * 100),
            }
        )
