import io
import json
import math
import random
import threading
import time

from flask import Response
from matplotlib import pyplot as plt

import collavoid
import mqtt
import vda5050
from models import TurtleGraph
from models.Order import Order, OrderType

graph: TurtleGraph.Graph


def send_robot_to_node(serial, source_node, target_node):
    source = graph.find_node_by_id(int(source_node))
    target = graph.find_node_by_id(int(target_node))
    nodes, edges = graph.get_shortest_route(source, target)

    # order3 = main.graph.create_vda5050_order(nodes, edges, serial)
    agv = graph.get_agv_by_id(int(serial))

    new_order = Order(graph, source, target)
    agv.order = new_order
    new_order.agv = agv
    print(new_order.completed)
    print(new_order.base)
    print(new_order.horizon)

    new_order.try_extension(0, 0)

    print(new_order.completed)
    print(new_order.base)
    print(new_order.horizon)

    msg = new_order.create_vda5050_message(agv)
    mqtt.client.publish(vda5050.get_mqtt_topic(str(serial), vda5050.Topic.ORDER), msg.json(), 2)

    return "Success"


def get_path_image(serial, source_node, target_node):
    fig1, ax1 = plt.subplots()
    plt_io = io.BytesIO()
    for edge in graph.edges:
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

    source = graph.find_node_by_id(int(source_node))
    target = graph.find_node_by_id(int(target_node))
    nodes, edges = graph.get_shortest_route(source, target)

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
    for station in graph.get_stations():
        stations.append({"nid": station.nid, "name": station.name})
    return stations


def get_agv_info():
    agv_and_info = list()
    for agv in graph.get_agvs():
        agv_and_info.append(
            {"agv_id": agv.aid, "driving_status": agv.driving_status, "connection_state": agv.connection_status,
             "charging_status": agv.charging_status, "battery_level": agv.battery_level, "velocity": agv.velocity})
    return agv_and_info


def get_orders():
    return []
    orders = list()
    for order in graph.get_active_orders():
        orders.append(json.loads(order.json()))
    return orders


def order_distributor(real_graph):
    global graph
    graph = real_graph
    while True:
        next_order = graph.pending_orders.get()
        print("Order Distributor is now distributing an order")

        agvs = graph.agvs

        target = random.randint(0, 1)
        if next_order.agv == 'AUTO1':
            target = 0
        elif next_order.agv == 'AUTO2':
            target = 1
        selected_agv = agvs[target]

        distance = 0
        if selected_agv.x is not None:
            distance = math.dist((selected_agv.x, selected_agv.y), (next_order.start.x, next_order.start.y))
        if distance > 1:
            nearest = graph.get_nearest_node_from_agv(selected_agv)
            reloc_order = Order(graph, nearest, next_order.start)
            selected_agv.pending_orders.put(reloc_order)
            print("Relocation order also created")
        selected_agv.pending_orders.put(next_order)
        print("Put order into queue for  " + str(selected_agv.aid) + ' ' + str(selected_agv) + ' ' + str(
            selected_agv.pending_orders))

        continue

        # Copy the list from the graph in order to be able to change it without effecting the iteration over the objects
        pending_orders = main.graph.pending_orders.copy()
        for order in pending_orders:
            if len(free_agvs) == 0:
                break

            if order.agv is not None:
                # An agv is already assigned to the order
                if order.agv in free_agvs:
                    agv = order.agv
                else:
                    continue
            else:
                # Assign the nearest free agv to the order
                agv = main.graph.get_nearest_free_agv(order.start)
                order.agv = agv

            nearest_node = main.graph.get_nearest_node_from_agv(agv)
            if nearest_node == order.start:
                # AGV is already on or near the start node of the order
                main.graph.pending_orders.remove(order)
                main.graph.current_orders.append(order)
                executing_order = order
            else:
                # AGV is not near the start node (another node is nearer) -> Make a relocation order
                reloc_order = Order(nearest_node, order.start, OrderType.RELOCATION)
                main.graph.current_orders.append(reloc_order)
                executing_order = reloc_order

            agv.order = executing_order
            # Send order_message to turtlebot
            msg = executing_order.create_vda5050_message(agv)
            # Agv-id same as Serial-Number ??
            mqtt.client.publish(vda5050.get_mqtt_topic(agv.aid, vda5050.Topic.ORDER), msg.json(), 2)

            free_agvs.remove(agv)
        # Wait for 10 seconds until checking again for free robots / available orders
        time.sleep(10)


def order_executor(order: Order):
    node_id = 0
    edge_id = 0

    locked_by_us = list()
    while True:
        graph.lock.acquire()
        success = True
        for node in order.nodes:
            n = graph.find_node_by_id(int(node.nodeId))
            success = n.try_lock()
            if success:
                locked_by_us.append(n)
            else:
                break
        graph.lock.release()
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
    graph.lock.acquire()
    for node in order.nodes:
        graph.find_node_by_id(int(node.nodeId)).release()
    graph.lock.release()
    graph.current_orders.remove(order)
    graph.completed_orders.append(order)
