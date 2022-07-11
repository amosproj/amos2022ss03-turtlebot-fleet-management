import json
import threading

from shapely.geometry import Point

SAFETY_BUFFER_NODE = 0.1  # m


class Node:
    @staticmethod
    def node_list_to_id_list(l):
        result = list()
        for n in l:
            result.append(n.nid)
        return result

    def __init__(self, nid: int, x: float, y: float, name: str = None):
        self.nid = nid
        self.x = x
        self.y = y
        self.name = name
        self.lock = -1
        self.buffer = Point(self.x, self.y).buffer(SAFETY_BUFFER_NODE)

    def to_dict(self):
        return {k: v for k, v in self.__dict__.items() if v is not None}

    def json(self) -> str:
        return json.dumps({'nid': self.nid, 'x': self.x, 'y': self.y, 'name': self.name})

    def try_lock(self, order_id: int) -> bool:
        if self.lock == -1 or self.lock == order_id:
            self.lock = order_id
            return True
        return False

    def release(self, order_id: int):
        if self.lock == order_id:
            self.lock = -1
