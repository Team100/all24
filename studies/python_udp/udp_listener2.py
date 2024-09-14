# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

import socket
import struct
import time
import threading
from typing import Any

from udp_primitive_protocol import Types


def decode(message: bytes) -> tuple[str, Types, Any]:
    """
    Message format v2.

    A message is a 
    """
    key_len: int = struct.unpack(">B", message[0:1])[0]
    key: str = message[1 : key_len + 1].decode("us-ascii")
    type_offset: int = key_len + 1
    type_id: int = struct.unpack(">B", message[type_offset : type_offset + 1])[0]
    val_type: Types = Types(type_id)
    val_offset: int = type_offset + 1
    val: Any
    match val_type:
        case Types.BOOLEAN:
            val = struct.unpack(">?", message[val_offset : val_offset + 1])[0]
        case Types.DOUBLE:
            val = struct.unpack(">d", message[val_offset : val_offset + 8])[0]
        case Types.INT:
            val = struct.unpack(">i", message[val_offset : val_offset + 4])[0]
        case Types.DOUBLE_ARRAY:
            array_len = struct.unpack(">i", message[val_offset : val_offset + 4])[0]
            array_offset = val_offset + 4
            val = []
            for i in range(array_len):
                item_offset = array_offset + 8 * i
                item = struct.unpack(">d", message[item_offset : item_offset + 8])[0]
                val.append(item)
        case Types.LONG:
            val = struct.unpack(">q", message[val_offset : val_offset + 8])[0]
        case Types.STRING:
            string_len = struct.unpack(">i", message[val_offset : val_offset + 4])[0]
            str_offset = val_offset + 4
            val = message[str_offset : str_offset + string_len].decode("us-ascii")
        case Types.LABEL:
            val = None
        case _:
            val = None

    return (key, val_type, val)


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
        print(f"{d:.0f} {i:d}")


def recv() -> None:
    global n
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", 1995))
    while True:

        # message, _ = server_socket.recvfrom(1024)
        message: bytes = server_socket.recv(1500)
        (key, val_type, val) = decode(message)
        n += 1
        print(f"key: {key} val_type: {val_type} val: {val}")


def main() -> None:
    """For testing only."""
    t1 = threading.Thread(target=recv)
    t2 = threading.Thread(target=stats)
    t1.start()
    t2.start()
    t1.join()  # will block forever because t1 never exits


if __name__ == "__main__":
    main()
