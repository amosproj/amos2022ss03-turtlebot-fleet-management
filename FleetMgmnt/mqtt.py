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
    serial_qnd = msg.topic.split('/')[-2]
    if topic == "state":
        serial_number = msg.topic.split('/')[-2]
        update_agv_position(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_battery(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_charging_status(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_velocity(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_last_node_id(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_driving_status(serial_number, pr.packet_receiver_for_state(msg.payload))
    elif topic == "connection":
        serial_number = msg.topic.split('/')[-2]
        update_connection_state(pr.packet_receiver_for_connection(msg.payload), serial_qnd)
        update_agv_connection_state(serial_number, pr.packet_receiver_for_connection(msg.payload))
    elif topic == "order":
        print(msg.topic + " " + str(msg.payload))
    # TODO Handling all the other information and topics


def update_agv_position(serial_number, state_msg):
    pos = state_msg.agvPosition
    agv_x = pos.x
    agv_y = pos.y
    # TODO adjust indexing the agv, when we handle more than one
    graph.get_agv_by_id(int(serial_number)).update_position(agv_x, agv_y)


def update_agv_battery(serial_number, state_msg):
    battery_state = state_msg.batteryState
    graph.get_agv_by_id(int(serial_number)).update_battery_level(battery_state.batteryCharge)
    # TODO Handling the avg ids for more than one AGV


def update_agv_charging_status(serial_number, state_msg):
    battery_state = state_msg.batteryState
    if battery_state.charging is True:
        temp = "Charging"
        graph.get_agv_by_id(int(serial_number)).update_charging_status(temp)
    elif battery_state.charging is False:
        temp = "Discharging"
        graph.get_agv_by_id(int(serial_number)).update_charging_status(temp)
        # TODO Handling the avg ids for more than one AGV


def update_agv_velocity(serial_number, state_msg):
    velocity = state_msg.velocity
    vx = velocity.vx
    vy = velocity.vy
    resultant_velocity = math.sqrt(math.pow(vx, 2) + math.pow(vy, 2))
    graph.get_agv_by_id(int(serial_number)).update_velocity(resultant_velocity)
    # TODO Handling the avg ids for more than one AGV


def update_connection_state(state_msg, serial_qnd):
    if state_msg.connectionState == "ONLINE":
        config_path = 'config.json'
        with open(config_path, "r") as config_file:
            config_json = 'maps/' + json.load(config_file)["map"]
            with open(config_json, "r") as file:
                data = file.read()
                encoded = base64.b64encode(data.encode('ascii')).decode()

                client.publish("AMOS/v1/TurtleBot/" + str(serial_qnd) + "/map", '{"data": "' + encoded + '"}')
                #print(encoded)


def update_agv_last_node_id(serial_number, state_msg):
    last_node_id = state_msg.lastNodeId
    graph.get_agv_by_id(int(serial_number)).update_last_nodeid(last_node_id)
    # TODO Handling the avg ids for more than one AGV


def update_agv_driving_status(serial_number, state_msg):
    if state_msg.driving is True and state_msg.paused is False:
        status = "Driving"
    elif state_msg.driving is False and state_msg.paused is True:
        status = "Paused"
    else:
        status = "No Status"
    graph.get_agv_by_id(int(serial_number)).update_driving_status(status)
    # TODO Handling the avg ids for more than one AGV


def update_agv_connection_state(serial_number, connection_msg):
    connection_state = connection_msg.connectionState
    graph.get_agv_by_id(1).update_connection_status(connection_state)
    # TODO Handling the avg ids for more than one AGV


def connect(host, port, username, password, real_graph):
    global graph, client
    client = mqtt.Client()
    graph = real_graph
    client.on_connect = on_connect
    client.on_message = on_message
    if password is not None:
        client.username_pw_set(username, password)
    client.connect(host, int(port), 60)
    client.loop_forever()
