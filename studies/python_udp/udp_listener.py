""" Test to receive UDP packets. """

import socket
import struct
from typing import Any

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(("", 1995))


def decode(message) -> tuple[str, Any]:
    """Return a key"""
    key_len = struct.unpack("B", message[0:1])[0]
    key = message[1 : key_len + 1].decode("us-ascii")
    val_type = struct.unpack("B", message[key_len + 1 : key_len + 2])[0]
    if val_type == 1:
        val = struct.unpack("?", message[key_len + 2 : key_len + 3])[0]
        return (key, val)
    else:
        return (key, None)


def main():
    """Run forever"""
    while True:
        message, _ = server_socket.recvfrom(1024)
        (key, val) = decode(message)
        print(f"key: {key} val: {val}")


main()
