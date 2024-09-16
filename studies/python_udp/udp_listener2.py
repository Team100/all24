# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

import socket
import struct
import time
import threading
import traceback
from typing import Any, Generator

from udp_primitive_protocol import Types

DATA_PORT = 1995

def parse(fmt, buf, offset) -> tuple[Any, int]:
    return struct.unpack_from(fmt, buf, offset)[0], offset + struct.calcsize(fmt)


def parse_string(buf, offset, string_len) -> tuple[str, int]:
    return buf[offset : offset + string_len].decode("us-ascii"), offset + string_len


def decode(message: bytes) -> Generator[tuple[int, Types, Any], None, None]:
    """
    format is (key (2), type (1), data (varies))
    Message format v2.  See UdpPrimitiveProtocol2.java.
    """
    try:
        offset = 0
        timestamp, offset = parse(">q", message, offset)
        # TODO: new timestamp means new log file
        while offset < len(message):
            key, offset = parse(">H", message, offset)
            type_id, offset = parse(">B", message, offset)
            val_type: Types = Types(type_id)
            match val_type:
                case Types.BOOLEAN:
                    val, offset = parse(">?", message, offset)
                case Types.DOUBLE:
                    val, offset = parse(">d", message, offset)
                case Types.INT:
                    val, offset = parse(">i", message, offset)
                case Types.DOUBLE_ARRAY:
                    array_len, offset = parse(">B", message, offset)
                    val = []
                    for _ in range(array_len):
                        item, offset = parse(">d", message, offset)
                        val.append(item)
                case Types.LONG:
                    val, offset = parse(">q", message, offset)
                case Types.STRING:
                    string_len, offset = parse(">B", message, offset)
                    val, offset = parse_string(message, offset, string_len)
                case _:
                    print(f"weird key {key} at offset {offset}")
                    val = None
            yield (key, val_type, val)

    except struct.error:
        print("skipping fragment: ", message[offset:])
        traceback.print_exc()
        return


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
