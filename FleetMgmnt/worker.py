import io

from flask import Response
from matplotlib import pyplot as plt

import main
import mqtt
import turtlegraph
import vda5050


def send_robot_to_node(serial, source_node, target_node):
    source = main.graph.find_node_by_id(int(source_node))
    target = main.graph.find_node_by_id(int(target_node))
    nodes, edges = main.graph.get_shortest_route(source, target)

    order = turtlegraph.create_vda5050_order(nodes, edges)

    mqtt.client.publish(vda5050.get_mqtt_topic(serial, vda5050.Topic.ORDER), order.json(), 2)

    return "Success"


def get_path_image(serial, source_node, target_node):
    fig1, ax1 = plt.subplots()
    plt_io = io.BytesIO()
    for edge in main.graph.edges:
        ax1.plot(
            [edge.start.x, edge.end.x],
            [edge.start.y, edge.end.y],
            color="gray"
        )
        ax1.plot(
            edge.start.x, edge.start.y,
            marker='.',
            color="gray"
        )
        ax1.plot(
            edge.end.x, edge.end.y,
            marker='.',
            color="gray"
        )

    source = main.graph.find_node_by_id(int(source_node))
    target = main.graph.find_node_by_id(int(target_node))
    nodes, edges = main.graph.get_shortest_route(source, target)

    for edge in edges:
        ax1.plot(
            [edge.start.x, edge.end.x],
            [edge.start.y, edge.end.y],
            color="red"
        )
        ax1.plot(
            edge.start.x, edge.start.y,
            marker='.',
            color="red"
        )
        ax1.plot(
            edge.end.x, edge.end.y,
            marker='.',
            color="red"
        )

    for node in nodes:
        ax1.annotate(str(node.nid), (node.x, node.y))

    ax1.get_xaxis().set_visible(False)
    ax1.get_yaxis().set_visible(False)
    fig1.savefig(plt_io, format="png", dpi=300, bbox_inches='tight')
    plt.close(fig1)
    return Response(plt_io.getvalue(), mimetype="image/png")


def get_stations():
    stations = list()
    for station in main.graph.get_stations():
        stations.append({"nid": station.nid, "name": station.name})
    return stations


def get_agv_info():
    agv_and_info = list()
    for agv in main.graph.get_agvs():
        agv_and_info.append({"agv_id": agv.aid, "status": agv.agv_status, "charging_status": agv.charging_status, "battery_level": agv.battery_level, "velocity": agv.velocity})
    return agv_and_info

