# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621
"""
Test harness for data listener alone.
"""
import socket
import time
import threading
from udp_parser import parse_long
from udp_data_decoder import data_decode

DATA_PORT = 1995
MTU = 1500
# updated by main thread
n = 0
max_key = 0
# updated by counter thread
m = 0


def stats() -> None:
    global m
    interval = 1.0
    t0 = time.time()
    while True:
        d = time.time() - t0
        dt = d % interval
        time.sleep(interval - dt)
        d = time.time() - t0
        i = n - m
        m = n
        print(f"DATA {d:.0f} {i:d} {max_key}")


def recv() -> None:
    global n, max_key
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", DATA_PORT))
    while True:

        message: bytes = server_socket.recv(MTU)
        timestamp, offset = parse_long(message, 0)
        print(f"DATA timestamp {timestamp}")
        for key, val_type, val in data_decode(message, offset):
            n += 1
            max_key = max(max_key, key)
            print(f"DATA key: {key} val_type: {val_type} val: {val}")


def main() -> None:
    """For testing only."""
    receiver = threading.Thread(target=recv)
    monitor = threading.Thread(target=stats)
    receiver.start()
    monitor.start()
    receiver.join()  # will block forever because the receiver never exits


if __name__ == "__main__":
    main()
