import io

from flask import Flask, send_from_directory, Response
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

import turtlegraph

app = Flask(__name__)


def start():
    app.run(host='0.0.0.0', port=8080)


@app.route("/graph")
def graph_image():
    graph = turtlegraph.Graph()
    graph.vmap_lines_to_graph("demo.vmap")
    img = graph.create_image()
    return Response(img.getvalue(), mimetype='image/png')


@app.route("/")
def serve_index():
    return send_from_directory('./WebUI', 'index.html')


@app.route("/<path:filename>")
def serve_file(filename):
    return send_from_directory('./WebUI', filename)
