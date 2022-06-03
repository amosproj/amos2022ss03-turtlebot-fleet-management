import paho.mqtt.client as mqtt

client: mqtt.Client


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("#")


def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))


def connect():
    global client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.username_pw_set("amos", "gdr734dg")
    client.connect("116.203.74.124", 31276, 60)
    client.loop_forever()
