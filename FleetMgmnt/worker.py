import io
import json
import threading
import time

from flask import Response
from matplotlib import pyplot as plt

import collavoid
import main
import mqtt
import vda5050
from models.Order import Order, OrderType


def send_robot_to_node(serial, source_node, target_node):
    source = main.graph.find_node_by_id(int(source_node))
    target = main.graph.find_node_by_id(int(target_node))
    nodes, edges = main.graph.get_shortest_route(source, target)

    # order3 = main.graph.create_vda5050_order(nodes, edges, serial)
    agv = main.graph.get_agv_by_id(int(serial))

    new_order = Order(source, target)
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
    mqtt.client.publish(vda5050.get_mqtt_topic(serial, vda5050.Topic.ORDER), msg.json(), 2)

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


def get_orders():
    orders = list()
    for order in main.graph.current_orders:
        orders.append(json.loads(order.json()))
    return orders


def order_distributor():
    while True:
        free_agvs = main.graph.get_free_agvs()
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
    main.graph.current_orders.remove(order)
    main.graph.completed_orders.append(order)
