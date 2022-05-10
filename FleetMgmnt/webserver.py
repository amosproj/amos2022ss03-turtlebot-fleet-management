from flask import Flask

app = Flask(__name__)


def start():
    app.run(host='0.0.0.0')


@app.route("/")
def hello_world():
    return "<p>Hello, World!</p>"
