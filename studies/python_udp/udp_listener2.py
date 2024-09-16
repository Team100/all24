# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

import socket
import struct
import time
import threading
import traceback
from typing import Any, Generator, Iterator

from udp_primitive_protocol import Types

DATA_PORT = 1995
MIN_KEY = 16

# TODO: handle timestamp from sender: open a new file when the timestamp changes, maybe use the timestamp as the filename.


def parse_label_map(
    message: bytes, offset: int
) -> Generator[tuple[int, Types, Any], None, int]:
    """yields (key, Type.LABEL, label).
    returns the new offset."""
    if offset + 3 > len(message):
        return offset
    label_offset: int = struct.unpack(">H", message[offset : offset + 2])[0]
    offset += 2
    array_len = struct.unpack(">B", message[offset : offset + 1])[0]
    offset += 1
    for i in range(array_len):
        if offset + 1 > len(message):
            return offset
        string_len = struct.unpack(">B", message[offset : offset + 1])[0]
        offset += 1
        if offset + string_len > len(message):
            return offset
        val = message[offset : offset + string_len].decode("us-ascii")
        offset += string_len
        yield (MIN_KEY + label_offset + i, Types.LABEL, val)
    return offset


def decode(message: bytes) -> Iterator[tuple[int, Types, Any]]:
    """
    returns (key, type, data)
    Message format v2.  See UdpPrimitiveProtocol2.java.
    """
    try:
        offset = 0
        while True:
            if offset + 2 > len(message):
                return
            # loop over sections
            type_id: int = struct.unpack(">H", message[offset : offset + 2])[0]
            offset += 2
            val_type: Types = Types(type_id)
            if val_type == Types.LABEL:
                offset = yield from parse_label_map(message, offset)
                continue
            while True:
                if offset + 2 > len(message):
                    return
                # loop over key-value pairs
                key: int = struct.unpack(">H", message[offset : offset + 2])[0]
                offset += 2
                if key < 16:
                    # change types
                    val_type = Types(key)
                    # label type works differently
                    if val_type == Types.LABEL:
                        offset = yield from parse_label_map(message, offset)
                        continue
                    continue
                offset = yield from parse_value(message, offset, key, val_type)
    except struct.error:
        print("ignoring invalid packet for offset: ", offset, "\n", message)
        traceback.print_exc()
        return


def parse_value(
    message, offset, key, val_type
) -> Generator[tuple[int, Types, Any], None, int]:
    val: Any
    match val_type:
        case Types.BOOLEAN:
            if offset + 1 > len(message):
                return offset
            val = struct.unpack(">?", message[offset : offset + 1])[0]
            offset += 1
        case Types.DOUBLE:
            if offset + 8 > len(message):
                return offset
            val = struct.unpack(">d", message[offset : offset + 8])[0]
            offset += 8
        case Types.INT:
            if offset + 4 > len(message):
                return offset
            val = struct.unpack(">i", message[offset : offset + 4])[0]
            offset += 4
        case Types.DOUBLE_ARRAY:
            if offset + 1 > len(message):
                return offset
            array_len: int = struct.unpack(">B", message[offset : offset + 1])[0]
            offset += 1
            val = []
            for _ in range(array_len):
                if offset + 8 > len(message):
                    return offset
                item = struct.unpack(">d", message[offset : offset + 8])[0]
                offset += 8
                val.append(item)
        case Types.LONG:
            if offset + 8 > len(message):
                return offset
            val = struct.unpack(">q", message[offset : offset + 8])[0]
            offset += 8
        case Types.STRING:
            if offset + 1 > len(message):
                return offset
            string_len: int = struct.unpack(">B", message[offset : offset + 1])[0]
            offset += 1
            if offset + string_len > len(message):
                return offset
            val = message[offset : offset + string_len].decode("us-ascii")
            offset += string_len
        case Types.LABEL:
            raise ValueError("label type shouldn't be here")
        case _:
            print(f"weird key {key} at offset {offset}")
            val = None
    yield (key, val_type, val)
    return offset


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
