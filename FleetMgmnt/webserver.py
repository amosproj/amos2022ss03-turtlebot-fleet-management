import json

from flask import Flask, Response, send_from_directory

import turtlegraph
import worker
import main

app = Flask(__name__)


def start():
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
    img = main.graph.create_image()
    return Response(img.getvalue(), mimetype="image/png")


@app.route("/graph.json")
def graph_json():
    return Response(main.graph.create_json(), mimetype="application/json")


@app.route("/api/graph/stations")
def graph_stations():
    return Response(json.dumps(worker.get_stations()), mimetype="application/json")


@app.route("/api/orders")
def get_orders():
    return Response(json.dumps(worker.get_orders()), mimetype="application/json")


@app.post("/api/agv/<robot_serial>/sendFromTo/<source_node_id>/<target_node_id>")
def robot_send_to(robot_serial, source_node_id, target_node_id):
    return worker.send_robot_to_node(robot_serial, source_node_id, target_node_id)
    # return str(robot_serial) + " " + str(target_node_id)


@app.get("/api/agv/<robot_serial>/pathDisplay/<source_node_id>/<target_node_id>")
def robot_send_to_path(robot_serial, source_node_id, target_node_id):
    return worker.get_path_image(robot_serial, source_node_id, target_node_id)
    # return str(robot_serial) + " " + str(target_node_id)


@app.route("/")
def serve_index():
    return send_from_directory("./WebUI", "index.html")


@app.route("/<path:filename>")
def serve_file(filename):
    return send_from_directory("./WebUI", filename)
