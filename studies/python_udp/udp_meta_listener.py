# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

# runs in a thread, listens for metadata updates.

import socket
import struct
import threading
import traceback
from typing import Any, Generator

from udp_primitive_protocol import Types

META_PORT = 1996


def meta_decode(message: bytes) -> Generator[tuple[int, Types, Any], None, None]:
    """format is (key (2), type (1), length (1), bytes (N))"""
    try:
        offset = 0
        timestamp: int = struct.unpack(">q", message[offset : offset + 8])[0]
        # TODO: new timestamp means new log file
        offset += 8
        while True:
            if offset >= len(message):
                return
            key: int = struct.unpack(">H", message[offset : offset + 2])[0]
            offset += 2
            type_id: int = struct.unpack(">B", message[offset : offset + 1])[0]
            offset += 1
            val_type: Types = Types(type_id)
            string_len = struct.unpack(">B", message[offset : offset + 1])[0]
            offset += 1
            val = message[offset : offset + string_len].decode("us-ascii")
            offset += string_len
            yield (key, val_type, val)

    except struct.error:
        print("ignoring invalid packet\n", message)
        traceback.print_exc()
        return


def meta_recv() -> None:
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", META_PORT))
    while True:

        message: bytes = server_socket.recv(1500)
        for key, val_type, val in meta_decode(message):
            print(f"key: {key} val_type: {val_type} val: {val}")


def main() -> None:
    """For testing only."""
    receiver = threading.Thread(target=meta_recv)
    receiver.start()
    receiver.join()  # will block forever because the receiver never exits


if __name__ == "__main__":
    main()
