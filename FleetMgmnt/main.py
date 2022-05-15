import threading
import time

# Internal dependencies
import webserver


def main():
    launch_thread(webserver.start, ())
    launch_thread(placeholder, ())


def placeholder():
    while 1:
        time.sleep(50000)


def launch_thread(target_function, args):
    thread = threading.Thread(target=target_function, args=args)
    thread.start()


if __name__ == '__main__':
    main()


