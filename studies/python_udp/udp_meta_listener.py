# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

# runs in a thread, listens for metadata updates.

import socket
import struct
import time
import threading
import traceback
from typing import Any, Generator

from udp_primitive_protocol import Types

META_PORT = 1996


def parse(fmt, buf, offset) -> tuple[Any, int]:
    return struct.unpack_from(fmt, buf, offset)[0], offset + struct.calcsize(fmt)


def parse_string(buf, offset, string_len) -> tuple[str, int]:
    return buf[offset : offset + string_len].decode("us-ascii"), offset + string_len


def meta_decode(message: bytes) -> Generator[tuple[int, Types, Any], None, None]:
    """
    message should be a list of (key (2), type (1), length (1), bytes (N))
    yields (key, type, label)
    """
    try:
        offset = 0
        timestamp, offset = parse(">q", message, offset)
        # TODO: new timestamp means new log file
        while offset < len(message):
            key, offset = parse(">H", message, offset)
            type_id, offset = parse(">B", message, offset)
            val_type: Types = Types(type_id)
            string_len, offset = parse(">B", message, offset)
            val, offset = parse_string(message, offset, string_len)
            yield (key, val_type, val)

    except struct.error:
        print("skipping fragment: ", message[offset:])
        traceback.print_exc()
        return


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

        message: bytes = server_socket.recv(1500)
        for key, val_type, val in meta_decode(message):
            n += 1
            print(f"key: {key} val_type: {val_type} val: {val}")


def main() -> None:
    """For testing only."""
    receiver = threading.Thread(target=meta_recv)
    monitor = threading.Thread(target=stats)
    receiver.start()
    monitor.start()
    receiver.join()  # will block forever because the receiver never exits


if __name__ == "__main__":
    main()
