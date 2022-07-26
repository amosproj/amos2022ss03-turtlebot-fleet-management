import json
import os
import threading

import mqtt
import recharge
import webserver
import worker
from models import TurtleGraph


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
        launch_thread(worker.agv_order_executor_thread, (n_agv,))

    launch_thread(webserver.start, (graph,))
    launch_thread(mqtt.connect, (config['mqtt']['host'], config['mqtt']['port'],
                                 config['mqtt']['username'], config['mqtt']['password'], config['map'], graph))
    launch_thread(worker.order_distributor, (graph,))
    launch_thread(recharge.generate_recharge_orders, (graph,))
    launch_thread(graph.create_map_thread, ())


if __name__ == "__main__":
    main()
