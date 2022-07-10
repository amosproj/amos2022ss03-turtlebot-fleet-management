import json
import os
import threading
import time

import worker
import mqtt
from models import TurtleGraph
import webserver


def launch_thread(target_function, args):
    thread = threading.Thread(target=target_function, args=args)
    thread.start()


def main():
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
        n_agv = graph.new_agv(int(agv['serial']), agv['color'])
        print("Creating AGV thread")
        launch_thread(n_agv.order_executor_thread, ())

    launch_thread(webserver.start, (graph, ))
    launch_thread(mqtt.connect, (config['mqtt']['host'], config['mqtt']['port'],
                                 config['mqtt']['username'], config['mqtt']['password'], config['map'], graph))
    launch_thread(worker.order_distributor, (graph, ))
    launch_thread(graph.create_map_thread(), ())
    launch_thread(placeholder, ())  # Example


def placeholder():
    while 1:
        #print("Global pending orders: " + str(graph.pending_orders.qsize()))
        #print("AGV 1 pending orders: " + str(graph.agvs[0].pending_orders.qsize()))
        #print("AGV 2 pending orders: " + str(graph.agvs[1].pending_orders.qsize()))
        time.sleep(2)


if __name__ == "__main__":
    main()
