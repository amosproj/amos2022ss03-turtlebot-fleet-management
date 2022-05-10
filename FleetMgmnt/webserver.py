from flask import Flask, send_from_directory

app = Flask(__name__)


def start():
    app.run(host='0.0.0.0')


@app.route("/")
def serve_index():
    return send_from_directory('./WebUI', 'index.html')


@app.route("/<path:filename>")
def serve_file(filename):
    return send_from_directory('./WebUI', filename)

