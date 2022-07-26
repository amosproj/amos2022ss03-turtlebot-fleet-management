import json
from typing import List

import vda5050

""" Functions for converting received messages from json into an VDA5050 object. """


def key_for_value(d, key):
    """ Return a key in `d` having a value of `value`. """
    for k, v in d.items():
        if k == key:
            return v


def packet_receiver_for_connection(json_string: str) -> vda5050.ConnectionMessage:
    connection_dict = json.loads(json_string)
    header_id_value = key_for_value(connection_dict, "headerId")
    timestamp_value = key_for_value(connection_dict, "timestamp")
    version_value = key_for_value(connection_dict, "version")
    manufacturer_value = key_for_value(connection_dict, "manufacturer")
    serial_number_value = key_for_value(connection_dict, "serialNumber")
    conn_state_value = key_for_value(connection_dict, "connectionState")

    connection = vda5050.ConnectionMessage(header_id_value, timestamp_value, version_value, manufacturer_value,
                                           serial_number_value, vda5050.ConnectionState(conn_state_value))
    return connection


def packet_receiver_for_state(json_string: str) -> vda5050.StateMessage:
    state_dict = json.loads(json_string)
    header_id_value = key_for_value(state_dict, "headerId")
    timestamp_value = key_for_value(state_dict, "timestamp")
    version_value = key_for_value(state_dict, "version")
    manufacturer_value = key_for_value(state_dict, "manufacturer")
    serial_no_value = key_for_value(state_dict, "serialNumber")
    order_id_value = key_for_value(state_dict, "orderId")
    order_update_id_value = key_for_value(state_dict, "orderUpdateId")
    zone_set_id_value = key_for_value(state_dict, "zoneSetId")
    last_node_id_value = key_for_value(state_dict, "lastNodeId")
    last_node_sequence_id_value = key_for_value(state_dict, "lastNodeSequenceId")
    driving_value = key_for_value(state_dict, "driving")
    paused_value = key_for_value(state_dict, "paused")
    new_base_request_value = key_for_value(state_dict, "newBaseRequest")
    distance_since_last_node_value = key_for_value(state_dict, "distanceSinceLastNode")
    operating_mode_value = key_for_value(state_dict, "operatingMode")

    node_state_list_value = key_for_value(state_dict, "nodeStates")
    node_states_value = packet_receiver_for_node_states(node_state_list_value)

    edge_state_list_value = key_for_value(state_dict, "edgeStates")
    edge_states_value = packet_receiver_for_edge_states(edge_state_list_value)

    avg_position_str = key_for_value(state_dict, "agvPosition")
    avg_position_value = packet_receiver_for_agv_position(avg_position_str)

    velocity_str = key_for_value(state_dict, "velocity")
    velocity_value = packet_receiver_for_velocity(velocity_str)

    battery_state_str = key_for_value(state_dict, "batteryState")
    battery_state_value = packet_receiver_for_battery_state(battery_state_str)

    return vda5050.StateMessage(header_id_value, timestamp_value, version_value, manufacturer_value, serial_no_value,
                                order_id_value, order_update_id_value, zone_set_id_value, last_node_id_value,
                                last_node_sequence_id_value, action_states=None, safety_state=None,
                                battery_state=battery_state_value, operation_mode=operating_mode_value, errors=None,
                                node_states=node_states_value, edge_state=edge_states_value, driving=driving_value,
                                agv_position=avg_position_value, velocity=velocity_value, paused=paused_value,
                                new_base_request=new_base_request_value,
                                distance_since_last_node=distance_since_last_node_value)


def packet_receiver_for_battery_state(json_battery_state: str) -> vda5050.BatteryState:
    battery_charge_value = key_for_value(json_battery_state, "batteryCharge")
    battery_voltage_value = key_for_value(json_battery_state, "batteryVoltage")
    battery_health_value = key_for_value(json_battery_state, "batteryHealth")
    charging_value = key_for_value(json_battery_state, "charging")
    reach_value = key_for_value(json_battery_state, "reach")

    return vda5050.BatteryState(battery_charge_value, charging_value, reach_value, battery_voltage_value,
                                battery_health_value)


def packet_receiver_for_velocity(json_velocity: str) -> vda5050.Velocity:
    vx_value = key_for_value(json_velocity, "vx")
    vy_value = key_for_value(json_velocity, "vy")
    omega_value = key_for_value(json_velocity, "omega")

    return vda5050.Velocity(vx_value, vy_value, omega_value)


def packet_receiver_for_agv_position(json_avg_position: str) -> vda5050.AgvPosition:
    x_value = key_for_value(json_avg_position, "x")
    y_value = key_for_value(json_avg_position, "y")
    theta_value = key_for_value(json_avg_position, "theta")
    map_id_value = key_for_value(json_avg_position, "mapId")
    map_desc_value = key_for_value(json_avg_position, "mapDescription")
    pos_init_value = key_for_value(json_avg_position, "positionInitialized")
    loc_score_value = key_for_value(json_avg_position, "localizationScore")
    deviation_range_value = key_for_value(json_avg_position, "deviationRange")

    return vda5050.AgvPosition(pos_init_value, x_value, y_value, theta_value, map_id_value, map_desc_value,
                               loc_score_value, deviation_range_value)


def packet_receiver_for_edge_states(json_list: List[str]) -> List[vda5050.EdgeState]:
    edge_states = []
    for edges in json_list:
        edge_id_value = key_for_value(edges, "edgeId")
        sequence_id_value = key_for_value(edges, "sequenceId")
        edge_description_value = key_for_value(edges, "edgeDescription")
        released_value = key_for_value(edges, "released")
        trajectory_str = key_for_value(edges, "trajectory")
        trajectory_value = None
        edge_states_temp = vda5050.EdgeState(edge_id_value, sequence_id_value, released_value, edge_description_value,
                                             trajectory_value)
        edge_states.append(edge_states_temp)
    return edge_states


def packet_receiver_for_node_states(json_list: List[str]) -> List[vda5050.NodeState]:
    node_states = []
    for nodes in json_list:
        node_id_value = key_for_value(nodes, "nodeId")
        sequence_id_value = key_for_value(nodes, "sequenceId")
        node_description_value = key_for_value(nodes, "nodeDescription")
        node_position_str = key_for_value(nodes, "nodePosition")
        node_position_value = packet_receiver_for_node_position(node_position_str)
        released_value = key_for_value(nodes, "released")
        node_states_temp = vda5050.NodeState(node_id_value, sequence_id_value, released_value, node_description_value,
                                             node_position_value)
        node_states.append(node_states_temp)

    return node_states


def packet_receiver_for_node_position(json_node_position: str) -> vda5050.NodePosition:
    x_value = key_for_value(json_node_position, "x")
    y_value = key_for_value(json_node_position, "y")
    theta_value = key_for_value(json_node_position, "theta")
    map_id_value = key_for_value(json_node_position, "mapId")
    node_position = vda5050.NodePosition(x_value, y_value, map_id_value, theta=theta_value)
    return node_position
