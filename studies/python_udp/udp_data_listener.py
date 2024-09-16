# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621
"""
Listens for data updates via UDP.
"""

import socket
import time
import threading
from typing import Any, Generator

from udp_parser import parse, parse_boolean, parse_double, parse_string
from udp_primitive_protocol import Types

DATA_PORT = 1995


def decode(
    message: bytes, offset: int
) -> Generator[tuple[int, Types, Any], None, None]:
    """
    message is (key (2), type (1), data (varies))
    yields (key, type, data)
    throws struct.error if parse fails
    """
    while offset < len(message):
        key, offset = parse(">H", message, offset)
        type_id, offset = parse(">B", message, offset)
        val_type: Types = Types(type_id)
        match val_type:
            case Types.BOOLEAN:
                bool_val, offset = parse_boolean(message, offset)
                yield (key, val_type, bool_val)

            case Types.DOUBLE:
                double_val, offset = parse_double(message, offset)
                yield (key, val_type, double_val)

            case Types.INT:
                int_val, offset = parse(">i", message, offset)
                yield (key, val_type, int_val)

            case Types.DOUBLE_ARRAY:
                array_len, offset = parse(">B", message, offset)
                array_val = []
                for _ in range(array_len):
                    item, offset = parse(">d", message, offset)
                    array_val.append(item)
                yield (key, val_type, array_val)

            case Types.LONG:
                long_val, offset = parse(">q", message, offset)
                yield (key, val_type, long_val)

            case Types.STRING:
                string_len, offset = parse(">B", message, offset)
                string_val, offset = parse_string(message, offset, string_len)
                yield (key, val_type, string_val)

            case _:
                print(f"weird key {key} at offset {offset}")


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
        print(f"{d:.0f} {i:d} {max_key}")


def recv() -> None:
    global n, max_key
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", DATA_PORT))
    while True:

        message: bytes = server_socket.recv(1500)
        for key, val_type, val in decode(message):
            n += 1
            max_key = max(max_key, key)
            print(f"key: {key} val_type: {val_type} val: {val}")


def main() -> None:
    """For testing only."""
    receiver = threading.Thread(target=recv)
    monitor = threading.Thread(target=stats)
    receiver.start()
    monitor.start()
    receiver.join()  # will block forever because the receiver never exits


if __name__ == "__main__":
    main()
