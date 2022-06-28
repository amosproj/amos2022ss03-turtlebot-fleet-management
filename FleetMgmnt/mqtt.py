import math
import base64
import json

import paho.mqtt.client as mqtt
import packet_receiver as pr
import main

client = mqtt.Client()


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("AMOS/#")
    # main.graph.new_agv()


def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    topic = msg.topic.split('/')[-1]
    if topic == "state":
        serial_number = msg.topic.split('/')[-2]
        update_agv_position(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_actions_state(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_battery(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_charging_status(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_velocity(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_last_node_id(serial_number, pr.packet_receiver_for_state(msg.payload))
        update_agv_driving_status(serial_number, pr.packet_receiver_for_state(msg.payload))
    elif topic == "connection":
        update_connection_state(pr.packet_receiver_for_connection(msg.payload))
    # TODO Handling all the other information and topics


def update_agv_position(serial_number, state_msg):
    pos = state_msg.agvPosition
    agv_x = pos.x
    agv_y = pos.y
    # TODO adjust indexing the agv, when we handle more than one
    main.graph.get_agv_by_id(1).update_position(agv_x, agv_y)


def update_agv_actions_state(serial_number, state_msg):
    action_state = state_msg.actionStates
    actions = list()
    if action_state is not None:
        for action in action_state:
            actions.append(action.actionStatus)
        main.graph.get_agv_by_id(1).update_status(actions)
    else:
        main.graph.get_agv_by_id(1).update_status(action_state)
    # TODO Handling the avg ids for more than one AGV


def update_agv_battery(serial_number, state_msg):
    battery_state = state_msg.batteryState
    main.graph.get_agv_by_id(1).update_battery_level(battery_state.batteryCharge)
    # TODO Handling the avg ids for more than one AGV


def update_agv_charging_status(serial_number, state_msg):
    battery_state = state_msg.batteryState
    if battery_state.charging is True:
        temp = "Charging"
        main.graph.get_agv_by_id(1).update_charging_status(temp)
    elif battery_state.charging is False:
        temp = "Discharging"
        main.graph.get_agv_by_id(1).update_charging_status(temp)
        # TODO Handling the avg ids for more than one AGV


def update_agv_velocity(serial_number, state_msg):
    velocity = state_msg.velocity
    vx = velocity.vx
    vy = velocity.vy
    resultant_velocity = math.sqrt(math.pow(vx, 2) + math.pow(vy, 2))
    main.graph.get_agv_by_id(1).update_velocity(resultant_velocity)
    # TODO Handling the avg ids for more than one AGV


def update_connection_state(state_msg):
    if state_msg.connectionState == "ONLINE":
        config_path = 'config.json'
        with open(config_path, "r") as config_file:
            config_json = 'maps/' + json.load(config_file)["map"]
            with open(config_json, "r") as file:
                data = file.read()
                encoded = base64.b64encode(data.encode('ascii')).decode()

                client.publish("AMOS/v1/TurtleBot/1/map", '{"data": "' + encoded + '"}')
                #print(encoded)


def update_agv_last_node_id(serial_number, state_msg):
    last_node_id = state_msg.lastNodeId
    main.graph.get_agv_by_id(1).update_last_nodeid(last_node_id)
    # TODO Handling the avg ids for more than one AGV


def update_agv_driving_status(serial_number, state_msg):
    if state_msg.driving is True and state_msg.paused is False:
        status = "Driving Mode"
    elif state_msg.driving is False and state_msg.paused is True:
        status = "Idle Mode"
    else:
        status = "No Status"
    main.graph.get_agv_by_id(1).update_driving_status(status)
    # TODO Handling the avg ids for more than one AGV


def connect(host, port, username, password):
    client.on_connect = on_connect
    client.on_message = on_message
    if password is not None:
        client.username_pw_set(username, password)
    client.connect(host, int(port), 60)
    client.loop_forever()

