# VDA5050 Lib
import json
from enum import Enum
from typing import List, Union, Any

INTERFACE_NAME = 'AMOS'
PROTOCOL_VERSION = '1'
MANUFACTURER = 'TurtleBot'
HEADER_ID = {
    'CONNECTION': 0,
    'FACTSHEET': 0,
    'INSTANT_ACTION': 0,
    'ORDER': 0,
    'STATE': 0,
    'VISUALIZATION': 0
}


class Topic(str, Enum):
    CONNECTION = 'connection'
    FACTSHEET = 'factsheet'
    INSTANT_ACTIONS = 'instantActions'
    ORDER = 'order'
    STATE = 'state'
    VISUALIZATION = 'visualization'


class ConnectionState(str, Enum):
    ONLINE = 'ONLINE'
    OFFLINE = 'OFFLINE'
    CONNECTIONBROKEN = 'CONNECTIONBROKEN'


class BlockingType(str, Enum):
    NONE = 'NONE'
    SOFT = 'SOFT'
    HARD = 'HARD'


class JsonSerializable:
    def to_dict(self):
        return {k: v for k, v in self.__dict__.items() if v is not None}

    def json(self, pretty: bool = False) -> str:
        if pretty:
            return json.dumps(self.to_dict(), default=lambda o: o.to_dict(), indent=4)
        else:
            return json.dumps(self.to_dict(), default=lambda o: o.to_dict())


class ActionParameter(JsonSerializable):
    def __init__(self, key: str, value: Union[List[Any], bool, float, str]):
        self.key = key
        self.value = value


class Action(JsonSerializable):
    def __init__(self, action_type: str, action_id: str, blocking_type: BlockingType,
                 action_description: str = None, action_parameters: List[str] = None):
        self.actionType = action_type
        self.actionId = action_id
        self.blockingType = blocking_type
        self.actionDescription = action_description
        self.actionParameters = action_parameters


class NodePosition(JsonSerializable):
    def __init__(self, x: float, y: float, map_id: str, map_description: str = None, theta: float = None,
                 allowed_deviation_xy: float = None, allowed_deviation_theta: float = None):
        self.x = x
        self.y = y
        self.mapId = map_id,
        self.mapDescription = map_description
        self.theta = theta
        self.allowedDeviationXy = allowed_deviation_xy
        self.allowedDeviationTheta = allowed_deviation_theta


class Node(JsonSerializable):
    def __init__(self, node_id: str, sequence_id: int, released: bool, actions: List[str],
                 node_description: str = None, node_position: NodePosition = None):
        self.nodeId = node_id
        self.sequenceId = sequence_id
        self.nodeDescription = node_description
        self.released = released
        self.nodePosition = node_position
        self.actions = actions


class Edge(JsonSerializable):
    def __init__(self, edge_id: str, sequence_id: int, released: bool, start_node_id: str, end_node_id: str,
                 actions: List[Action], edge_description: str = None, max_speed: float = None,
                 max_height: float = None, min_height: float = None, orientation: float = None,
                 orientation_type: str = 'TANGENTIAL', direction: str = None, rotation_allowed: bool = None,
                 max_rotation_speed: float = None, length: float = None, trajectory: dict = None):
        self.edgeId = edge_id
        self.sequenceId = sequence_id
        self.released = released
        self.startNodeId = start_node_id
        self.endNodeId = end_node_id
        self.actions = actions
        self.edgeDescription = edge_description
        self.maxSpeed = max_speed
        self.maxHeight = max_height
        self.minHeight = min_height
        self.orientation = orientation
        self.orientationType = orientation_type
        self.direction = direction
        self.rotationAllowed = rotation_allowed
        self.maxRotationSpeed = max_rotation_speed
        self.length = length
        self.trajectory = trajectory


class Message(JsonSerializable):
    def __init__(self, headerid: int, timestamp: str, version: str, manufacturer: str, serialnumber: str):
        self.headerId = headerid
        self.timestamp = timestamp
        self.version = version
        self.manufacturer = manufacturer
        self.serialNumber = serialnumber

    def get_header_id(self):
        return self.headerId


class ConnectionMessage(Message, JsonSerializable):
    def __init__(self, headerid: int, timestamp: str, version: str, manufacturer: str, serialnumber: str,
                 connection_state: ConnectionState):
        Message.__init__(self, headerid, timestamp, version, manufacturer, serialnumber)
        self.connectionState = connection_state


class OrderMessage(Message, JsonSerializable):
    def __init__(self, headerid: int, timestamp: str, version: str, manufacturer: str, serialnumber: str,
                 order_id: str, order_update_id: int, nodes: List[Node], edges, zone_set_id: str = None, ):
        Message.__init__(self, headerid, timestamp, version, manufacturer, serialnumber)
        self.orderId = order_id
        self.orderUpdateId = order_update_id
        self.nodes = nodes
        self.edges = edges
        self.zoneSetId = zone_set_id


def get_mqtt_topic(serial_number: str, topic: Topic):
    return INTERFACE_NAME + '/v' + PROTOCOL_VERSION + '/' + MANUFACTURER + '/' + serial_number + '/' + topic.value


# Below this comment is playground code that should be removed before release

cm = ConnectionMessage(23, 'dsf', 'hgj', 'sdf', 'sdfs', ConnectionState.ONLINE)

node = Node('dsf', 23, True, list())
print(node.json(True))

print(get_mqtt_topic('0', Topic.CONNECTION))
