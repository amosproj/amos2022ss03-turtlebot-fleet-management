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
    plt_io = io.BytesIO()
    plt.clf()
    for edge in main.graph.edges:
        plt.plot(
            [edge.start.x, edge.end.x],
            [edge.start.y, edge.end.y],
            color="gray"
        )
        plt.plot(
            edge.start.x, edge.start.y,
            marker='.',
            color="gray"
        )
        plt.plot(
            edge.end.x, edge.end.y,
            marker='.',
            color="gray"
        )

    source = main.graph.find_node_by_id(int(source_node))
    target = main.graph.find_node_by_id(int(target_node))
    nodes, edges = main.graph.get_shortest_route(source, target)

    for edge in edges:
        plt.plot(
            [edge.start.x, edge.end.x],
            [edge.start.y, edge.end.y],
            color="red"
        )
        plt.plot(
            edge.start.x, edge.start.y,
            marker='.',
            color="red"
        )
        plt.plot(
            edge.end.x, edge.end.y,
            marker='.',
            color="red"
        )

    for node in nodes:
        plt.annotate(str(node.nid), (node.x, node.y))

    plt.savefig(plt_io, format="png", dpi=300)
    return Response(plt_io.getvalue(), mimetype="image/png")

