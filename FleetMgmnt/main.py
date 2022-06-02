import threading
import time

import mqtt
import turtlegraph
import webserver

vmap_file = "demo.vmap"
graph = turtlegraph.Graph()
graph.vmap_lines_to_graph(vmap_file)


def main():
    launch_thread(webserver.start, ())
    # launch_thread(mqtt.connect(), ())
    launch_thread(placeholder, ())  # Example


def placeholder():
    while 1:
        time.sleep(50000)


def launch_thread(target_function, args):
    thread = threading.Thread(target=target_function, args=args)
    thread.start()


if __name__ == "__main__":
    main()
