import io
import math
import random
import time

import matplotlib
import matplotlib.pyplot as plt
from flask import Response

from models import TurtleGraph, AGV
from models.Order import Order, OrderType

matplotlib.use("Agg")
graph: TurtleGraph.Graph

""" Handles api requests. """


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
        stations.append({"nid": station.nid, "name": station.name, "x": station.x, "y": station.y})
    return stations


def get_agv_info():
    agv_and_info = list()
    for agv in graph.get_agvs():
        agv_and_info.append(
            {"agv_id": agv.aid, "driving_status": agv.driving_status, "connection_state": agv.connection_status,
             "charging_status": agv.charging_status, "battery_level": agv.battery_level, "velocity": agv.velocity, "x": agv.x, "y": agv.y, "color": agv.color})
    return agv_and_info


def get_node_for_graph():
    nodes_edges = list()
    for edges in graph.edges:
        temp = list()
        temp.append({"x": edges.start.x, "y": edges.start.y})
        temp.append({"x": edges.end.x, "y": edges.end.y})
        nodes_edges.append(temp)
    return nodes_edges


def order_distributor(real_graph):
    global graph
    graph = real_graph
    while True:
        next_order = graph.pending_orders.get()
        # print("Order Distributor is now distributing an order")

        if type(next_order.agv) is AGV.AGV:
            selected_agv = next_order.agv
        elif type(next_order.agv) is str:
            # Actually shouldn't be a String, but just for safety reasons
            if next_order.agv == 'AUTO1':
                target = 1
            elif next_order.agv == 'AUTO2':
                target = 2
            else:
                target = random.randint(0, 1) + 1
                print('Random choose of agv')
            selected_agv = graph.get_agv_by_id(target)
        else:
            target = random.randint(0, 1) + 1
            selected_agv = graph.get_agv_by_id(target)
            print('Random choose of agv')

        selected_agv.pending_orders.put(next_order)
        # print("Put order into queue for  " + str(selected_agv.aid) + ' ' + str(selected_agv) + ' ' + str(
        #    selected_agv.pending_orders))
        time.sleep(1)


def create_order_helper(start, end):
    return Order(graph, start, end, OrderType.RELOCATION)


def agv_order_executor_thread(agv):
    while True:
        if agv.charging_status == "Charging":
            time.sleep(10)
            continue

        # print("AGV " + str(self.aid) + " order executor thread is online " + str(self) + ' ' +
        # str(self.pending_orders))
        next_order = agv.pending_orders.get()
        # print("AGV is now starting on new order")
        if next_order.status != 'CREATED':
            continue

        agv.lock.acquire()

        distance = 0
        if agv.x is not None:
            distance = math.dist((agv.x, agv.y), (next_order.start.x, next_order.start.y))
        if distance > 1:
            nearest = agv.graph.get_nearest_node_from_agv(agv)
            reloc_order = create_order_helper(nearest, next_order.start)
            agv.pending_orders.put(reloc_order)
            agv.pending_orders.put(next_order)
            agv.lock.release()
            continue

        # print("AGV has gotten lock")
        agv.order = next_order
        agv.order.agv = agv
        # print("AGV " + str(self.aid) + " has a new order, executing now...")
        throttle = 0
        while agv.order.extension_required(agv.x, agv.y):
            if agv.order.try_extension(agv.x, agv.y):
                throttle = 0
            else:
                throttle += 1
                if throttle > 20:
                    time.sleep(0.1)
                if throttle > 100:
                    time.sleep(1)
                    print("Throttling active")
        agv.lock.release()

        agv.order.sem.acquire()
        # print("AGV " + str(self.aid) + " has finished order")
        time.sleep(1)
