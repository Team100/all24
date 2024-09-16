# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621
"""
Listens for metadata updates via UDP.
"""

import socket
import time
import threading
from typing import Any, Generator

from udp_parser import parse_byte, parse_short, parse_string, parse_long
from udp_primitive_protocol import Types

META_PORT = 1996
MTU = 1500


def meta_decode(
    message: bytes, offset: int
) -> Generator[tuple[int, Types, Any], None, None]:
    """
    message is a list of (key (2), type (1), length (1), bytes (N))
    yields (key, type, label)
    throws struct.error if parse fails
    """
    while offset < len(message):
        key, offset = parse_short(message, offset)
        type_id, offset = parse_byte(message, offset)
        val_type: Types = Types(type_id)
        label, offset = parse_string(message, offset)
        yield (key, val_type, label)


# updated by main thread
n = 0
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
        print(f"META: {d:.0f} {i:d}")


def meta_recv() -> None:
    global n
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", META_PORT))
    while True:

        message: bytes = server_socket.recv(MTU)
        timestamp, offset = parse_long(message, 0)
        print(f"META timestamp {timestamp}")
        for key, val_type, val in meta_decode(message, offset):
            n += 1
            print(f"META key: {key} val_type: {val_type} val: {val}")


def main() -> None:
    """For testing only."""
    receiver = threading.Thread(target=meta_recv)
    monitor = threading.Thread(target=stats)
    receiver.start()
    monitor.start()
    receiver.join()  # will block forever because the receiver never exits


if __name__ == "__main__":
    main()
