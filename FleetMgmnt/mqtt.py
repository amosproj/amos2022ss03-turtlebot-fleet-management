import paho.mqtt.client as mqtt
import packet_receiver as pr
import main

client: mqtt.Client


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("AMOS/#")
    main.graph.new_agv()


def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    if msg.topic.split('/')[-1] == "state":  # TODO optimize the check for the topic
        serial_number = msg.topic.split('/')[-2]
        update_agv_position(serial_number, pr.packet_receiver_for_state(msg.payload))
    # TODO Handling all the other information and topics


def update_agv_position(serial_number, state_msg):
    pos = state_msg.agvPosition
    agv_x = pos.x
    agv_y = pos.y
    # TODO adjust indexing the agv, when we handle more than one
    main.graph.get_agv_by_id(0).update_position(agv_x, agv_y)


def connect():
    global client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    # client.username_pw_set("amos", "gdr734dg")
    client.connect("broker.hivemq.com", 1883, 60)
    client.loop_forever()
