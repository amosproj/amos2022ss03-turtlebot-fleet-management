import json
import os
import threading
import time

import mqtt
from models import TurtleGraph
import webserver

docker = 'FMS_DOCKER' in os.environ
config = {}
graph = TurtleGraph.Graph()

config_path = 'config.json'
if docker:
    print('FMS is running in a Docker container. If this is NOT the case, something is wrong.')
    config_path = '/usr/src/app/config.json'

# Load config file
with open(config_path) as config_file:
    config = json.load(config_file)

if docker:
    graph.vmap_lines_to_graph('/usr/src/app/maps/' + config['map'])
else:
    graph.vmap_lines_to_graph('maps/' + config['map'])

for agv in config['agvs']:
    graph.new_agv(int(agv['serial']), agv['color'])


def main():
    launch_thread(webserver.start, ())
    launch_thread(mqtt.connect, (config['mqtt']['host'], config['mqtt']['port'],
                                 config['mqtt']['username'], config['mqtt']['password']))
    launch_thread(placeholder, ())  # Example


def placeholder():
    while 1:
        time.sleep(50000)


def launch_thread(target_function, args):
    thread = threading.Thread(target=target_function, args=args)
    thread.start()


if __name__ == "__main__":
    main()
