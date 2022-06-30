import json
import threading

from shapely.geometry import Point

SAFETY_BUFFER_NODE = 0.5  # m


class Node:
    def __init__(self, nid: int, x: float, y: float, name: str = None):
        self.nid = nid
        self.x = x
        self.y = y
        self.name = name
        self.lock = threading.Lock()
        self.buffer = Point(self.x, self.y).buffer(SAFETY_BUFFER_NODE)

    def to_dict(self):
        return {k: v for k, v in self.__dict__.items() if v is not None}

    def json(self, pretty: bool = False) -> str:
        if pretty:
            return json.dumps(self.to_dict(), default=lambda o: o.to_dict(), indent=4)
        else:
            return json.dumps(self.to_dict(), default=lambda o: o.to_dict())

    def try_lock(self) -> bool:
        print("Trying lock of node " + str(self.nid))
        return self.lock.acquire(blocking=False)

    def release(self):
        print("Release lock of node " + str(self.nid))
        self.lock.release()
