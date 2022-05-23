# VDA5050 Lib
import dis
import json
from enum import Enum
from typing import List, Union, Any
from threading import Thread
from threading import Lock


class Topic(str, Enum):
    CONNECTION = 'connection'
    FACTSHEET = 'factsheet'
    INSTANT_ACTIONS = 'instantActions'
    ORDER = 'order'
    STATE = 'state'
    VISUALIZATION = 'visualization'


INTERFACE_NAME = 'AMOS'
PROTOCOL_VERSION = '1'
MANUFACTURER = 'TurtleBot'
HEADER_ID = {
    Topic.CONNECTION: 0,
    Topic.FACTSHEET: 0,
    Topic.INSTANT_ACTIONS: 0,
    Topic.ORDER: 0,
    Topic.STATE: 0,
    Topic.VISUALIZATION: 0
}
HEADER_ID_LOCK = {
    Topic.CONNECTION: Lock(),
    Topic.FACTSHEET: Lock(),
    Topic.INSTANT_ACTIONS: Lock(),
    Topic.ORDER: Lock(),
    Topic.STATE: Lock(),
    Topic.VISUALIZATION: Lock()
}


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

class InstantAction(Message, JsonSerializable):
    def __init__(self,headerid: int,  timestamp: str,  version: str, manufacturer: str,serialnumber: str,
                actions: List[Action] ):
        Message.__init__(self, headerid, timestamp, version, manufacturer, serialnumber)
        self.actions = actions

class nodeState(JsonSerializable):
    def __init__(self, nodeId: str, sequenceId: int, relesed: bool, nodeDescription: str = None, node_position: NodePosition = None):
        self.nodeId = nodeId
        self.sequenceId = sequenceId
        self.nodeDescription = nodeDescription
        self.nodePosition = node_position
        self.relesed = relesed

class edgeState(JsonSerializable):
    def __init__(self, edgeId: str, sequenceId: int, relesed: bool, edgeDescription: str = None, 
                 trajectory: dict = None):
        self.edgeId = edgeId
        self.sequenceId = sequenceId
        self.edgeDescription = edgeDescription
        self.relesed = relesed
        self.trajectory = trajectory
        
class agvPosition(JsonSerializable):
    def __init__(self,positioninitialized: bool, x: float, y: float, theta: float, mapid: str, mapdescription: str = None,
                  localizationscore: float = None , deviationrange: float = None,):
        self.positionInitialized = positioninitialized
        self.localizationScore = localizationscore
        self.deviationRange = deviationrange
        self.x = x
        self.y = y
        self.theta = theta
        self.mapId = mapid
        self.mapDescription = mapdescription

class velocity(JsonSerializable):
    def __init__(self, vx: float = None, vy: float = None, omega: float = None):
        self.vx = vx
        self.vy = vy
        self.omega = omega

class boundingBoxReference(JsonSerializable):
    def __init__(self, x: float, y: float, z: float, theta: float = None):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        
class loadDimensions(JsonSerializable):
    def __init__(self, length: float, width: float, height: float = None):
        self.length = length
        self.width = width
        self.height = height

class load(JsonSerializable):
    def __init__(self, loadid: str = None, loadtype: str = None, loadposition: str = None, boundingboxref: boundingBoxReference = None ,
                 loaddimension: loadDimensions = None , Weight: float = None ):
        self.loadId = loadid
        self.loadType = loadtype
        self.loadPosition = loadposition
        self.boundingBoxReference = boundingboxref
        self.loadDimensions = loaddimension 
        self.weight = Weight


class ActionStatus(str, Enum):
    WAITING = 'WAITING'
    INITIALIZING = 'INITIALIZING'
    RUNNING = 'RUNNING'
    PAUSED = 'PAUSED'
    FINISHED = 'FINISHED'
    FAILED = 'FAILED'


class actionState(JsonSerializable):
    def __init__(self, actionid: str, actionstatus: ActionStatus, resultdescription: str = None,
                  actiontype: str = None, actiondescription: str = None):
        self.actionId = actionid 
        self.actionType = actiontype
        self.actionDescription = actiondescription
        self.actionStatus = ActionStatus 
        self.resultDescription = resultdescription

class batteryState(JsonSerializable):
    def __init__(self, batterycharge: float, charging: bool, reach: int = None,
                 batteryvoltage: float = None, batteryhealth: float = None):
        self.batteryCharge = batterycharge
        self.batteryVoltage = batteryvoltage
        self.batteryHealth = batteryhealth
        self.charging = charging
        self.reach = reach

class errorReference(JsonSerializable):
    def __init__(self, referencekey: str, referencevalue: str):
        self.referenceKey = referencekey
        self.referenceValue = referencevalue

class ErrorLevel(str, Enum):
    WARNING = 'WARNING'
    FATAL = 'FATAL'

class error(JsonSerializable):
    def __init__(self, errortype: str,errorlevel: ErrorLevel , errorreference: List[errorReference] = None ,
                 errordescription: str = None):
        self.errorType = errortype
        self.errorReferences =  errorreference
        self.errorDescription = errordescription
        self.errorLevel = errorlevel

class InfoLevel(str, Enum):
    DEBUG = 'DEBUG'
    INFO = 'INFO'

class infoReference(JsonSerializable):
    def __init__(self, referencekey: str, referencevalue: str):
        self.referenceKey = referencekey
        self.referenceValue = referencevalue

class info(JsonSerializable):
    def __init__(self, infotype: str,infolevel: InfoLevel,
                 inforeference: List[infoReference] = None , infodescription: str = None):
        self.infoType = infotype
        self.infoReferences = inforeference
        self.infoDescription = infodescription
        self.infoLevel = infolevel

class eStop(str, Enum):
    AUTOACK = 'AUTOACK'
    MANUAL = 'MANUAL'
    REMOTE = 'REMOTE'
    NONE = 'NONE'

class safetyState(JsonSerializable):
    def __init__(self, estop: eStop, fieldviolation: bool):
        self.eStop = estop
        self.fieldViolation = fieldviolation

class OperatingMode(str,Enum):
    AUTOMATIC = 'AUTOMATIC'
    SEMIAUTOMATIC = 'SEMIAUTOMATIC'
    MANUAL = 'MANUAL'
    SERVICE = 'SERVICE'
    TEACHIN = 'TEACHIN'
    
class StateMessage(Message, JsonSerializable):
    def __init__(self, headerid: int, timestamp: str, version: str, manufacturer: str, serialnumber: str,
                 order_id: str, order_update_id: int, zone_set_id: str, last_Node_id: str, last_Node_sequence_id: int,
                 action_states: List[actionState],safety_state: safetyState , battery_state: batteryState, operation_Mode: OperatingMode , errors: List[error],
                 node_states: List[nodeState], edge_state: List[edgeState], driving: bool, avg_position: agvPosition = None, velocity: velocity = None ,
                 loads: List[load] = None, paused: bool = None, new_base_request: bool = None, distance_Since_LastNode: float = None,
                 information: List[info] = None):
        Message.__init__(self, headerid, timestamp, version, manufacturer, serialnumber)
        self.orderId = order_id
        self.orderUpdateId = order_update_id
        self.zoneSetId = zone_set_id
        self.lastNodeId = last_Node_id
        self.lastNodeSequenceId = last_Node_sequence_id
        self.nodeStates = node_states 
        self.edgeStates = edge_state 
        self.agvPosition = avg_position
        self.velocity = velocity 
        self.loads = loads 
        self.driving =  driving
        self.paused = paused 
        self.newBaseRequest = new_base_request
        self.distanceSinceLastNode = distance_Since_LastNode
        self.actionStates = action_states 
        self.batteryState = battery_state
        self.operatingMode = operation_Mode
        self.errors = errors
        self.information = information 
        self.safetyState = safety_state
        
      
    
def get_mqtt_topic(serial_number: str, topic: Topic):
    return INTERFACE_NAME + '/v' + PROTOCOL_VERSION + '/' + MANUFACTURER + '/' + serial_number + '/' + topic.value


def get_header_id(topic: Topic):
    HEADER_ID_LOCK[topic].acquire()
    HEADER_ID[topic] += 1
    id = HEADER_ID[topic]
    HEADER_ID_LOCK[topic].release()
    return id


# Below this comment is playground code that should be removed before release

cm = ConnectionMessage(23, 'dsf', 'hgj', 'sdf', 'sdfs', ConnectionState.ONLINE)

node = Node('dsf', 23, True, list())
print(node.json(True))

print(get_mqtt_topic('0', Topic.CONNECTION))
print(get_header_id(Topic.CONNECTION))
print(get_header_id(Topic.ORDER))
print(get_header_id(Topic.CONNECTION))
