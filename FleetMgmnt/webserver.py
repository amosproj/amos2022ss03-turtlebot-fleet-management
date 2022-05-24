import io

from flask import Flask, send_from_directory, Response
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

import turtlegraph

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
    graph = turtlegraph.Graph()
    graph.vmap_lines_to_graph("demo.vmap")
    img = graph.create_image()
    return Response(img.getvalue(), mimetype="image/png")


@app.route("/graph.json")
def graph_json():
    graph = turtlegraph.Graph()
    graph.vmap_lines_to_graph("demo.vmap")
    return Response(graph.create_json(), mimetype="application/json")


@app.route("/")
def serve_index():
    return send_from_directory("./WebUI", "index.html")


@app.route("/<path:filename>")
def serve_file(filename):
    return send_from_directory("./WebUI", filename)
