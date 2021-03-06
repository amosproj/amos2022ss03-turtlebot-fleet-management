import json
import logging

from flask import Flask, Response, send_from_directory

import worker
from models import TurtleGraph

app = Flask(__name__)
graph = TurtleGraph.Graph()


def start(real_graph):
    global graph
    graph = real_graph

    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    app.run(host="0.0.0.0", port=8080)


@app.get("/api/station/<station_id>/status")
def station_status(station_id):
    return "Test"


@app.post("/api/station/<station_id>/req")
def station_req(station_id):
    return "Test"


@app.post("/api/station/<station_id>/move")
def station_move(station_id):
    return "Test"


@app.post("/api/station/<station_id>/sendTo/<target_station_id>")
def station_send_to(station_id, target_station_id):
    return str(station_id) + " " + str(target_station_id)


@app.route("/graph")
def graph_image():
    return Response(graph.image.getvalue(), mimetype="image/png")


@app.route("/api/graph")
def graph_json():
    return Response(graph.create_json(), mimetype="application/json")


@app.route("/api/graph/stations")
def graph_stations():
    return Response(json.dumps(worker.get_stations()), mimetype="application/json")


@app.route("/api/agvs")
def get_agv_info():
    return Response(json.dumps(worker.get_agv_info()), mimetype="application/json")


@app.route("/api/orders")
def get_orders():
    orders = list()
    for order in graph.all_orders:
        serialized_order = order.serialize()
        del serialized_order['cosp']
        orders.append(serialized_order)
    return Response(json.dumps(orders), mimetype="application/json")


@app.delete("/api/orders/<order_id>")
def cancel_order(order_id):
    for order in graph.all_orders:
        if order.order_id == int(order_id):
            order.cancel()
    return Response(json.dumps({"success": True}), mimetype="application/json")


@app.post("/api/orders/<order_id>/resend")
def resend_order(order_id):
    for order in graph.all_orders:
        if order.order_id == int(order_id):
            order.resend()
    return Response(json.dumps({"success": True}), mimetype="application/json")


@app.post("/api/agv/<robot_serial>/sendFromTo/<source_node_id>/<target_node_id>")
def robot_send_to(robot_serial, source_node_id, target_node_id):
    # ToDo: This is a legacy API endpoint and should be removed or changed before final release
    # return worker.send_robot_to_node(robot_serial, source_node_id, target_node_id)
    return graph.append_new_order(source_node_id, target_node_id, robot_serial)


@app.get("/api/agv/<robot_serial>/pathDisplay/<source_node_id>/<target_node_id>")
def robot_send_to_path(robot_serial, source_node_id, target_node_id):
    # ToDo: This is a legacy API endpoint and should be removed or changed before final release
    return worker.get_path_image(robot_serial, source_node_id, target_node_id)
    # return str(robot_serial) + " " + str(target_node_id)


@app.route("/")
def serve_index():
    return send_from_directory("./WebUI", "index.html")


@app.route("/<path:filename>")
def serve_file(filename):
    return send_from_directory("./WebUI", filename)
