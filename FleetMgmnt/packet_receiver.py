import vda5050
import json

def key_for_value(d, Key):
    """Return a key in `d` having a value of `value`."""
    for k, v in d.items():
        if k == Key:
            return v


def packet_receiver_for_connection(json_string : str):
    connection_dict = json.loads(json_string)
    headerId_value = key_for_value(connection_dict,"headerId")
    timestamp_value = key_for_value(connection_dict,"timestamp")
    version_value = key_for_value(connection_dict,"version")
    manufacturer_value = key_for_value(connection_dict,"manufacturer")
    serialNo_value = key_for_value(connection_dict,"serialNumber")
    Conn_state_value = key_for_value(connection_dict,"connectionState")

    connection = vda5050.ConnectionMessage(headerId_value,timestamp_value,version_value,manufacturer_value,serialNo_value,vda5050.ConnectionState(Conn_state_value))
    return connection

def packet_receiver_for_state(json_string : str):
    state_dict = json.loads(json_string)
    headerId_value = key_for_value(state_dict,"headerId")
    timestamp_value = key_for_value(state_dict,"timestamp")
    version_value = key_for_value(state_dict,"version")
    manufacturer_value = key_for_value(state_dict,"manufacturer")
    serialNo_value = key_for_value(state_dict,"serialNumber")
    orderId_value = key_for_value(state_dict,"orderId")
    orderUpdateId_value = key_for_value(state_dict,"orderUpdateId")
    zoneSetId_value = key_for_value(state_dict,"zoneSetId")
    lastNodeId_value = key_for_value(state_dict,"lastNodeId")
    lastNodeSequenceId_value = key_for_value(state_dict,"lastNodeSequenceId")
    driving_value = key_for_value(state_dict,"driving")
    paused_value = key_for_value(state_dict,"paused")
    newBaseRequest_value = key_for_value(state_dict,"newBaseRequest")
    distanceSinceLastNode_value = key_for_value(state_dict,"distanceSinceLastNode")
    operatingMode_value = key_for_value(state_dict,"operatingMode")

    nodestate_list_value = key_for_value(state_dict,"nodeStates")
    nodeStates_value = packet_receiver_for_nodestates(nodestate_list_value)

    edgestate_list_value = key_for_value(state_dict, "edgeStates")
    edgeStates_value = packet_receiver_for_edgestates(edgestate_list_value)

    avgposition_str = key_for_value(state_dict, "agvPosition")
    avgPosition_value = packet_receiver_for_agvposition(avgposition_str)

    velocity_str = key_for_value(state_dict, "velocity")
    velocity_value = packet_receiver_for_velocity(velocity_str)

    batteryState_str = key_for_value(state_dict,"batteryState")
    batteryState_value = packet_receiver_for_batteryState(batteryState_str)

    return vda5050.StateMessage(headerId_value,timestamp_value,version_value,
                                manufacturer_value,serialNo_value,orderId_value,orderUpdateId_value,zoneSetId_value,
                                lastNodeId_value,lastNodeSequenceId_value,action_states=None,safety_state=None,battery_state=batteryState_value,
                                operation_mode=operatingMode_value,errors=None,node_states=nodeStates_value,
                                edge_state=edgeStates_value,driving=driving_value,
                                avg_position=avgPosition_value,velocity=velocity_value,
                                paused=paused_value,new_base_request=newBaseRequest_value,distance_since_last_node=distanceSinceLastNode_value)


def packet_receiver_for_batteryState(json_batterystate : str):
    batteryCharge_value = key_for_value(json_batterystate,"batteryCharge")
    batteryVoltage_value = key_for_value(json_batterystate,"batteryVoltage")
    batteryHealth_value= key_for_value(json_batterystate,"batteryHealth")
    charging_value = key_for_value(json_batterystate, "charging")
    reach_value = key_for_value(json_batterystate, "reach")

    return vda5050.BatteryState(batteryCharge_value,charging_value,reach_value,batteryVoltage_value,batteryHealth_value)



def packet_receiver_for_velocity(json_velocity : str):
    vx_value = key_for_value(json_velocity,"vx")
    vy_value = key_for_value(json_velocity,"vy")
    omega_value= key_for_value(json_velocity,"omega")

    return vda5050.Velocity(vx_value,vy_value,omega_value)


def packet_receiver_for_agvposition(json_avgposition : str):
    x_value = key_for_value(json_avgposition,"x")
    y_value = key_for_value(json_avgposition,"y")
    theta_value= key_for_value(json_avgposition,"theta")
    mapId_value= key_for_value(json_avgposition,"mapId")
    mapDes_value= key_for_value(json_avgposition,"mapDescription")
    posIn_value= key_for_value(json_avgposition,"positionInitialized")
    locscore_value= key_for_value(json_avgposition,"localizationScore")
    deviationRange_value= key_for_value(json_avgposition,"deviationRange")

    return vda5050.AgvPosition(posIn_value,x_value,y_value,theta_value,mapId_value,mapDes_value,locscore_value,deviationRange_value)


def packet_receiver_for_edgestates(json_list):
    edgestates = []
    for edges in json_list:
        edgeId_value = key_for_value(edges,"edgeId")
        sequenceId_value = key_for_value(edges,"sequenceId")
        edgeDescription_value = key_for_value(edges,"edgeDescription")
        released_value = key_for_value(edges, "released")
        trajectory_str = key_for_value(edges,"trajectory")
        trajectory_value = packet_receiver_for_trajectory(trajectory_str)
        edgestates_temp = vda5050.EdgeState(edgeId_value,sequenceId_value,released_value,edgeDescription_value,trajectory_value)
        edgestates.append(edgestates_temp)
    return edgestates

def packet_receiver_for_trajectory(json_trajectory : str):
    degree_value = key_for_value(json_trajectory, "degree")
    knotVector_value = key_for_value(json_trajectory, "knotVector")
    controlPoints_str = key_for_value(json_trajectory, "controlPoints")
    controlPoints_value = packet_receiver_for_controlpoints(controlPoints_str)
    return None

def packet_receiver_for_controlpoints(json_list_controlpoints):
    for points in json_list_controlpoints:
        x_value = key_for_value(points,"x")
        y_value = key_for_value(points,"y")
        weight_value = key_for_value(points,"weight")
    return None


def packet_receiver_for_nodestates(json_list):
    nodestates = []
    for nodes in json_list:
        nodeId_value = key_for_value(nodes,"nodeId")
        sequenceId_value = key_for_value(nodes,"sequenceId")
        nodeDescription_value = key_for_value(nodes,"nodeDescription")
        nodeposition_str = key_for_value(nodes,"nodePosition")
        nodePosition_value = packet_receiver_for_nodeposition(nodeposition_str)
        released_value = key_for_value(nodes,"released")
        nodestates_temp = vda5050.NodeState(nodeId_value,sequenceId_value,released_value,nodeDescription_value,nodePosition_value)
        nodestates.append(nodestates_temp)

    return  nodestates
def packet_receiver_for_nodeposition(json_nodeposition : str):
    x_value= key_for_value(json_nodeposition,"x")
    y_value= key_for_value(json_nodeposition,"y")
    theta_value = key_for_value(json_nodeposition,"theta")
    mapId_value = key_for_value(json_nodeposition,"mapId")
    nodeposition = vda5050.NodePosition(x_value,y_value,mapId_value,theta=theta_value)
    return nodeposition


# Below comments are for test purpose : Removed before the Release

#with open('VADA5050_State.json', 'r') as file:
 #   connection_data= json.load(file)
#json_string = json.dumps(connection_data)

#state=packet_receiver_for_state(json_string)



