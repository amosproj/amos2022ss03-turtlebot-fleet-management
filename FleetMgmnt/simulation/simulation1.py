import json
import sys
import threading
import time

import paho.mqtt.client as mqtt

client = mqtt.Client()


def read_input():
    x = len("AMOS/v1/TurtleBot/1/state b'")
    with open('simulation1.txt', 'r') as f:
        for line in f:
            if 'AMOS/v1/TurtleBot/1/state' in line:
                client.publish('AMOS/v1/TurtleBot/1/state', line[x:-2])
                time.sleep(1)
        sys.exit(0)


def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("AMOS/#")
    threading.Thread(target=read_input).start()


def main():
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect('broker.hivemq.com')
    client.loop_forever()


if __name__ == "__main__":
    main()
