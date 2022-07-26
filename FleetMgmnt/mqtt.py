import base64
import math
import threading

import paho.mqtt.client as mqtt

import packet_receiver as pr
from models import TurtleGraph

client: mqtt.Client
graph: TurtleGraph.Graph
map_name = ""

""" Connects to MQTT broker and handles incoming messages from turtlebot. """


""" Connects to MQTT broker and handels incoming messages from turtlebot. """
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("AMOS/#")


def on_message(client, userdata, msg):
    thread = threading.Thread(target=on_message_thread, args=(client, userdata, msg))
    thread.start()


def on_message_thread(client, userdata, msg):
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


def update_agv_position(state_msg):
    pos = state_msg.agvPosition
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_position(pos.x, pos.y)


def update_agv_battery(state_msg):
    battery_state = state_msg.batteryState
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_battery_level(battery_state.batteryCharge)


def update_agv_charging_status(state_msg):
    battery_state = state_msg.batteryState
    if battery_state.charging:
        graph.get_agv_by_id(int(state_msg.serialNumber)).update_charging_status("Charging")
    else:
        graph.get_agv_by_id(int(state_msg.serialNumber)).update_charging_status("Discharging")


def update_agv_velocity(state_msg):
    vel = state_msg.velocity
    resultant_velocity = math.sqrt(math.pow(vel.vx, 2) + math.pow(vel.vy, 2))
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_velocity(resultant_velocity)


def update_connection_state(state_msg):
    if state_msg.connectionState == "ONLINE":
        map_path = "maps/" + map_name
        with open(map_path, "r") as file:
            data = file.read()
            encoded = base64.b64encode(data.encode('ascii')).decode()

            client.publish("AMOS/v1/TurtleBot/" + str(state_msg.serialNumber) + "/map", '{"data": "' + encoded + '"}')


def update_agv_last_node_id(state_msg):
    last_node_id = state_msg.lastNodeId
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_last_node_id(last_node_id)


def update_agv_driving_status(state_msg):
    if state_msg.driving and not state_msg.paused:
        status = "Driving"
    elif not state_msg.driving and state_msg.paused:
        status = "Paused"
    else:
        status = "No Status"
    graph.get_agv_by_id(int(state_msg.serialNumber)).update_driving_status(status)


def update_agv_connection_state(connection_msg):
    connection_state = connection_msg.connectionState
    graph.get_agv_by_id(int(connection_msg.serialNumber)).update_connection_status(connection_state)


def connect(host, port, username, password, new_map_name, new_graph):
    global client, graph, map_name
    graph = new_graph
    map_name = new_map_name
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    if password is not None:
        client.username_pw_set(username, password)
    client.connect(host, int(port), 60)
    client.loop_forever()
