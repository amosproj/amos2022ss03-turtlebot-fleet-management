import io
import json
import threading
import time

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

    order = main.graph.create_vda5050_order(nodes, edges, serial)

    thread = threading.Thread(target=order_executor, args=(order, ))
    thread.start()

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


def get_path_coordinate(serial, source_node, target_node):

    source = main.graph.find_node_by_id(int(source_node))
    target = main.graph.find_node_by_id(int(target_node))
    nodes, edges = main.graph.get_shortest_route(source, target)

    nodes_list = list()
    for edge in edges:
        temp = list()
        temp.append({"x": edge.start.x, "y": edge.start.y})
        temp.append({"x": edge.end.x, "y": edge.end.y})
        nodes_list.append(temp)
    return Response(json.dumps(nodes_list), mimetype="application/json")


def get_agv_and_coordinates():
    agv_info = list()
    for agv in main.graph.get_agvs():
        if agv.x is not None and agv.y is not None:
            agv_info.append({"x": agv.x, "y": agv.y, "color": agv.color})
    return agv_info


def get_stations():
    stations = list()
    for station in main.graph.get_stations():
        stations.append({"nid": station.nid, "name": station.name, "x": station.x, "y": station.y})
    return stations


def get_agv_info():
    agv_and_info = list()
    for agv in main.graph.get_agvs():
        agv_and_info.append({"agv_id": agv.aid, "status": agv.agv_status, "charging_status": agv.charging_status, "battery_level": agv.battery_level, "velocity": agv.velocity})
    return agv_and_info


def get_node_coordinates():
    nodes = list()
    for edges in main.graph.edges:
        temp = list()
        temp.append({"x": edges.start.x, "y": edges.start.y})
        temp.append({"x": edges.end.x, "y": edges.end.y})
        nodes.append(temp)
    return nodes


def get_orders():
    orders = list()
    for order in main.graph.orders:
        orders.append(json.loads(order.json()))
    return orders


def order_executor(order: vda5050.OrderMessage):
    node_id = 0
    edge_id = 0

    locked_by_us = list()
    while True:
        main.graph.lock.acquire()
        success = True
        for node in order.nodes:
            n = main.graph.find_node_by_id(int(node.nodeId))
            success = n.try_lock()
            if success:
                locked_by_us.append(n)
            else:
                break
        main.graph.lock.release()
        time.sleep(3)
        if success:
            print("All locks acquired successfully ")
            break
        for node in locked_by_us:
            node.release()
        locked_by_us.clear()

    while True:
        if node_id < len(order.nodes):
            order.nodes[node_id].released = True
        if edge_id < len(order.edges):
            order.edges[edge_id].released = True
        mqtt.client.publish(vda5050.get_mqtt_topic(order.serialNumber, vda5050.Topic.ORDER), order.json(), 2)
        if order.is_fully_released():
            break
        time.sleep(3)
        order.orderUpdateId += 1
        node_id += 1
        edge_id += 1
    print("Order is fully released")
    main.graph.lock.acquire()
    for node in order.nodes:
        main.graph.find_node_by_id(int(node.nodeId)).release()
    main.graph.lock.release()
    main.graph.orders.remove(order)
    main.graph.completed_orders.append(order)
