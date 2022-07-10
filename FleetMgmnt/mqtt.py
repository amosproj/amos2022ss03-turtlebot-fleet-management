import math
import base64
import json

import paho.mqtt.client as mqtt
import packet_receiver as pr
from models import TurtleGraph

client: mqtt.Client
graph: TurtleGraph.Graph


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("AMOS/#")
    # main.graph.new_agv()


def on_message(client, userdata, msg):
    # print(msg.topic + " " + str(msg.payload))
    topic = msg.topic.split('/')[-1]
    if topic == "state":
        update_agv_position(pr.packet_receiver_for_state(msg.payload))
        update_agv_battery(pr.packet_receiver_for_state(msg.payload))
        update_agv_charging_status(pr.packet_receiver_for_state(msg.payload))
        update_agv_velocity(pr.packet_receiver_for_state(msg.payload))
        update_agv_last_node_id(pr.packet_receiver_for_state(msg.payload))
        update_agv_driving_status(pr.packet_receiver_for_state(msg.payload))
    elif topic == "connection":
        update_connection_state(pr.packet_receiver_for_connection(msg.payload))
        update_agv_connection_state(pr.packet_receiver_for_connection(msg.payload))
    # TODO Handling all the other information and topics


def update_agv_position(state_msg):
    pos = state_msg.agvPosition
    agv_x = pos.x
    agv_y = pos.y
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_position(agv_x, agv_y)


def update_agv_battery(state_msg):
    battery_state = state_msg.batteryState
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_battery_level(battery_state.batteryCharge)


def update_agv_charging_status(state_msg):
    battery_state = state_msg.batteryState
    if battery_state.charging is True:
        temp = "Charging"
        graph.get_agv_by_id(int(state_msg.serialNumber)).update_charging_status(temp)
    elif battery_state.charging is False:
        temp = "Discharging"
        graph.get_agv_by_id(int(state_msg.serialNumber)).update_charging_status(temp)


def update_agv_velocity(state_msg):
    velocity = state_msg.velocity
    vx = velocity.vx
    vy = velocity.vy
    resultant_velocity = math.sqrt(math.pow(vx, 2) + math.pow(vy, 2))
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_velocity(resultant_velocity)


def update_connection_state(state_msg):
    if state_msg.connectionState == "ONLINE":
        config_path = 'config.json'
        with open(config_path, "r") as config_file:
            config_json = 'maps/' + json.load(config_file)["map"]
            with open(config_json, "r") as file:
                data = file.read()
                encoded = base64.b64encode(data.encode('ascii')).decode()

                client.publish("AMOS/v1/TurtleBot/" + str(state_msg.serialNumber) + "/map", '{"data": "' + encoded + '"}')


def update_agv_last_node_id(state_msg):
    last_node_id = state_msg.lastNodeId
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_last_nodeid(last_node_id)


def update_agv_driving_status(state_msg):
    if state_msg.driving is True and state_msg.paused is False:
        status = "Driving"
    elif state_msg.driving is False and state_msg.paused is True:
        status = "Paused"
    else:
        status = "No Status"
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_driving_status(status)


def update_agv_connection_state(connection_msg):
    connection_state = connection_msg.connectionState
    graph.get_agv_by_id(int(connection_msg.serialNumber)).update_connection_status(connection_state)


def connect(host, port, username, password, real_graph):
    global graph, client
    graph = real_graph
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    if password is not None:
        client.username_pw_set(username, password)
    client.connect(host, int(port), 60)
    client.loop_forever()