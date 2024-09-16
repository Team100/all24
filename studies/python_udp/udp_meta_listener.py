# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

# runs in a thread, listens for metadata updates.

import socket
import struct
import time
import threading
import traceback
from typing import Any, Generator, Iterator

from udp_primitive_protocol import Types

META_PORT = 1996

def meta_decode(message: bytes) -> Iterator[tuple[int, Types, Any]]:

def meta_recv() -> None:
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", META_PORT))
    while True:

        message: bytes = server_socket.recv(1500)
        for key, val_type, val in meta_decode(message):
            n += 1
            max_key = max(max_key, key)
            print(f"key: {key} val_type: {val_type} val: {val}")

def main() -> None:
    """For testing only."""
    receiver = threading.Thread(target=meta_recv)
    receiver.start()
    receiver.join()  # will block forever because the receiver never exits


if __name__ == "__main__":
    main()
