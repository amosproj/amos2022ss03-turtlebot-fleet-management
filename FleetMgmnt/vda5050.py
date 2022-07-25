import json
from enum import Enum
from threading import Lock
from typing import Any, List, Union


class Topic(str, Enum):
    CONNECTION = "connection"
    FACTSHEET = "factsheet"
    INSTANT_ACTIONS = "instantActions"
    ORDER = "order"
    STATE = "state"
    VISUALIZATION = "visualization"


INTERFACE_NAME = "AMOS"
PROTOCOL_VERSION = "1"
MANUFACTURER = "TurtleBot"
HEADER_ID = {
    Topic.CONNECTION: 0,
    Topic.FACTSHEET: 0,
    Topic.INSTANT_ACTIONS: 0,
    Topic.ORDER: 0,
    Topic.STATE: 0,
    Topic.VISUALIZATION: 0,
}
HEADER_ID_LOCK = {
    Topic.CONNECTION: Lock(),
    Topic.FACTSHEET: Lock(),
    Topic.INSTANT_ACTIONS: Lock(),
    Topic.ORDER: Lock(),
    Topic.STATE: Lock(),
    Topic.VISUALIZATION: Lock(),
}


class ConnectionState(str, Enum):
    ONLINE = "ONLINE"
    OFFLINE = "OFFLINE"
    CONNECTIONBROKEN = "CONNECTIONBROKEN"


class BlockingType(str, Enum):
    NONE = "NONE"
    SOFT = "SOFT"
    HARD = "HARD"


class JsonSerializable:
    def to_dict(self) -> dict:
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
    def __init__(
        self,
        action_type: str,
        action_id: str,
        blocking_type: BlockingType,
        action_description: str = None,
        action_parameters: List[str] = None,
    ):
        self.actionType = action_type
        self.actionId = action_id
        self.blockingType = blocking_type
        self.actionDescription = action_description
        self.actionParameters = action_parameters


class NodePosition(JsonSerializable):
    def __init__(
        self,
        x: float,
        y: float,
        map_id: str,
        map_description: str = None,
        theta: float = None,
        allowed_deviation_xy: float = None,
        allowed_deviation_theta: float = None,
    ):
        self.x = x
        self.y = y
        self.mapId = map_id
        self.mapDescription = map_description
        self.theta = theta
        self.allowedDeviationXy = allowed_deviation_xy
        self.allowedDeviationTheta = allowed_deviation_theta


class Node(JsonSerializable):
    def __init__(
        self,
        node_id: str,
        sequence_id: int,
        released: bool,
        actions: List[str],
        node_description: str = None,
        node_position: NodePosition = None,
    ):
        self.nodeId = node_id
        self.sequenceId = sequence_id
        self.nodeDescription = node_description
        self.released = released
        self.nodePosition = node_position
        self.actions = actions


class Edge(JsonSerializable):
    def __init__(
        self,
        edge_id: str,
        sequence_id: int,
        released: bool,
        start_node_id: str,
        end_node_id: str,
        actions: List[Action],
        edge_description: str = None,
        max_speed: float = None,
        max_height: float = None,
        min_height: float = None,
        orientation: float = None,
        orientation_type: str = "TANGENTIAL",
        direction: str = None,
        rotation_allowed: bool = None,
        max_rotation_speed: float = None,
        length: float = None,
        trajectory: dict = None,
    ):
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
        # self.orientationType = orientation_type
        self.direction = direction
        self.rotationAllowed = rotation_allowed
        self.maxRotationSpeed = max_rotation_speed
        self.length = length
        self.trajectory = trajectory


class Message(JsonSerializable):
    def __init__(
        self,
        header_id: int,
        timestamp: str,
        version: str,
        manufacturer: str,
        serial_number: str,
    ):
        self.headerId = header_id
        self.timestamp = timestamp
        self.version = version
        self.manufacturer = manufacturer
        self.serialNumber = serial_number

    def get_header_id(self) -> int:
        return self.headerId


class ConnectionMessage(Message, JsonSerializable):
    def __init__(
        self,
        header_id: int,
        timestamp: str,
        version: str,
        manufacturer: str,
        serial_number: str,
        connection_state: ConnectionState,
    ):
        Message.__init__(self, header_id, timestamp, version, manufacturer, serial_number)
        self.connectionState = connection_state


class OrderMessage(Message, JsonSerializable):
    def __init__(
        self,
        header_id: int,
        timestamp: str,
        version: str,
        manufacturer: str,
        serial_number: str,
        order_id: str,
        order_update_id: int,
        nodes: List[Node],
        edges: List[Edge],
        zone_set_id: str = None,
    ):
        Message.__init__(self, header_id, timestamp, version, manufacturer, serial_number)
        self.orderId = order_id
        self.orderUpdateId = order_update_id
        self.nodes = nodes
        self.edges = edges
        self.zoneSetId = zone_set_id

    def is_fully_released(self):
        for node in self.nodes:
            if not node.released:
                return False
        for edge in self.edges:
            if not edge.released:
                return False
        return True


class InstantAction(Message, JsonSerializable):
    def __init__(
        self,
        header_id: int,
        timestamp: str,
        version: str,
        manufacturer: str,
        serial_number: str,
        instant_actions: List[Action],
    ):
        Message.__init__(
            self, header_id, timestamp, version, manufacturer, serial_number
        )
        self.instantActions = instant_actions


class NodeState(JsonSerializable):
    def __init__(
        self,
        node_id: str,
        sequence_id: int,
        released: bool,
        node_description: str = None,
        node_position: NodePosition = None,
    ):
        self.nodeId = node_id
        self.sequenceId = sequence_id
        self.nodeDescription = node_description
        self.nodePosition = node_position
        self.released = released


class EdgeState(JsonSerializable):
    def __init__(
        self,
        edge_id: str,
        sequence_id: int,
        released: bool,
        edge_description: str = None,
        trajectory: dict = None,
    ):
        self.edgeId = edge_id
        self.sequenceId = sequence_id
        self.edgeDescription = edge_description
        self.released = released
        self.trajectory = trajectory


class AgvPosition(JsonSerializable):
    def __init__(
        self,
        position_initialized: bool,
        x: float,
        y: float,
        theta: float,
        map_id: str,
        map_description: str = None,
        localization_score: float = None,
        deviation_range: float = None,
    ):
        self.positionInitialized = position_initialized
        self.localizationScore = localization_score
        self.deviationRange = deviation_range
        self.x = x
        self.y = y
        self.theta = theta
        self.mapId = map_id
        self.mapDescription = map_description


class Velocity(JsonSerializable):
    def __init__(self, vx: float = None, vy: float = None, omega: float = None):
        self.vx = vx
        self.vy = vy
        self.omega = omega


class BoundingBoxReference(JsonSerializable):
    def __init__(self, x: float, y: float, z: float, theta: float = None):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta


class LoadDimensions(JsonSerializable):
    def __init__(self, length: float, width: float, height: float = None):
        self.length = length
        self.width = width
        self.height = height


class Load(JsonSerializable):
    def __init__(
        self,
        load_id: str = None,
        load_type: str = None,
        load_position: str = None,
        bounding_box_ref: BoundingBoxReference = None,
        load_dimension: LoadDimensions = None,
        weight: float = None,
    ):
        self.loadId = load_id
        self.loadType = load_type
        self.loadPosition = load_position
        self.boundingBoxReference = bounding_box_ref
        self.loadDimensions = load_dimension
        self.weight = weight


class ActionStatus(str, Enum):
    WAITING = "WAITING"
    INITIALIZING = "INITIALIZING"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    FINISHED = "FINISHED"
    FAILED = "FAILED"


class ActionState(JsonSerializable):
    def __init__(
        self,
        action_id: str,
        action_status: ActionStatus,
        result_description: str = None,
        action_type: str = None,
        action_description: str = None,
    ):
        self.actionId = action_id
        self.actionType = action_type
        self.actionDescription = action_description
        self.actionStatus = action_status
        self.resultDescription = result_description


class BatteryState(JsonSerializable):
    def __init__(
        self,
        batterycharge: float,
        charging: bool,
        reach: int = None,
        battery_voltage: float = None,
        battery_health: float = None,
    ):
        self.batteryCharge = batterycharge
        self.batteryVoltage = battery_voltage
        self.batteryHealth = battery_health
        self.charging = charging
        self.reach = reach


class ErrorReference(JsonSerializable):
    def __init__(self, reference_key: str, reference_value: str):
        self.referenceKey = reference_key
        self.referenceValue = reference_value


class ErrorLevel(str, Enum):
    WARNING = "WARNING"
    FATAL = "FATAL"


class Error(JsonSerializable):
    def __init__(
        self,
        error_type: str,
        error_level: ErrorLevel,
        error_reference: List[ErrorReference] = None,
        error_description: str = None,
    ):
        self.errorType = error_type
        self.errorReferences = error_reference
        self.errorDescription = error_description
        self.errorLevel = error_level


class InfoLevel(str, Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"


class InfoReference(JsonSerializable):
    def __init__(self, reference_key: str, reference_value: str):
        self.referenceKey = reference_key
        self.referenceValue = reference_value


class Info(JsonSerializable):
    def __init__(
        self,
        info_type: str,
        info_level: InfoLevel,
        info_reference: List[InfoReference] = None,
        info_description: str = None,
    ):
        self.infoType = info_type
        self.infoReferences = info_reference
        self.infoDescription = info_description
        self.infoLevel = info_level


class EStop(str, Enum):
    AUTOACK = "AUTOACK"
    MANUAL = "MANUAL"
    REMOTE = "REMOTE"
    NONE = "NONE"


class SafetyState(JsonSerializable):
    def __init__(self, e_stop: EStop, field_violation: bool):
        self.eStop = e_stop
        self.fieldViolation = field_violation


class OperatingMode(str, Enum):
    AUTOMATIC = "AUTOMATIC"
    SEMIAUTOMATIC = "SEMIAUTOMATIC"
    MANUAL = "MANUAL"
    SERVICE = "SERVICE"
    TEACHIN = "TEACHIN"


class StateMessage(Message, JsonSerializable):
    def __init__(
        self,
        header_id: int,
        timestamp: str,
        version: str,
        manufacturer: str,
        serial_number: str,
        order_id: str,
        order_update_id: int,
        zone_set_id: str,
        last_node_id: str,
        last_node_sequence_id: int,
        action_states: List[ActionState],
        safety_state: SafetyState,
        battery_state: BatteryState,
        operation_mode: OperatingMode,
        errors: List[Error],
        node_states: List[NodeState],
        edge_state: List[EdgeState],
        driving: bool,
        agv_position: AgvPosition = None,
        velocity: Velocity = None,
        loads: List[Load] = None,
        paused: bool = None,
        new_base_request: bool = None,
        distance_since_last_node: float = None,
        information: List[Info] = None,
    ):
        Message.__init__(
            self, header_id, timestamp, version, manufacturer, serial_number
        )
        self.orderId = order_id
        self.orderUpdateId = order_update_id
        self.zoneSetId = zone_set_id
        self.lastNodeId = last_node_id
        self.lastNodeSequenceId = last_node_sequence_id
        self.nodeStates = node_states
        self.edgeStates = edge_state
        self.agvPosition = agv_position
        self.velocity = velocity
        self.loads = loads
        self.driving = driving
        self.paused = paused
        self.newBaseRequest = new_base_request
        self.distanceSinceLastNode = distance_since_last_node
        self.actionStates = action_states
        self.batteryState = battery_state
        self.operatingMode = operation_mode
        self.errors = errors
        self.information = information
        self.safetyState = safety_state


def get_mqtt_topic(serial_number: int, topic: Topic) -> str:
    return (
        "AMOS"
        + "/v"
        + PROTOCOL_VERSION
        + "/"
        + MANUFACTURER
        + "/"
        + str(serial_number)
        + "/"
        + topic.value
    )


def get_header_id(topic: Topic) -> int:
    HEADER_ID_LOCK[topic].acquire()
    HEADER_ID[topic] += 1
    hid = HEADER_ID[topic]
    HEADER_ID_LOCK[topic].release()
    return hid
