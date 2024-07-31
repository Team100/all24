""" Test to receive UDP packets. """

import socket
import struct
from typing import Any

server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(("", 1995))


def decode(message) -> tuple[str, Any]:
    """Return a key"""
    key_len = struct.unpack(">B", message[0:1])[0]
    key = message[1 : key_len + 1].decode("us-ascii")
    type_offset = key_len + 1
    val_type = struct.unpack(">B", message[type_offset : type_offset + 1])[0]
    val_offset = type_offset + 1
    if val_type == 1:  # bool
        val = struct.unpack(">?", message[val_offset : val_offset + 1])[0]
        return (key, val)
    if val_type == 2:  # double
        val = struct.unpack(">d", message[val_offset : val_offset + 8])[0]
        return (key, val)
    if val_type == 3:  # int
        val = struct.unpack(">i", message[val_offset : val_offset + 4])[0]
        return (key, val)
    elif val_type == 4:  # float
        val = struct.unpack(">f", message[val_offset : val_offset + 4])[0]
        return (key, val)
    elif val_type == 5:  # double array
        array_len = struct.unpack(">i", message[val_offset : val_offset + 4])[0]
        array_offset = val_offset + 4
        val = []
        for i in range(array_len):
            item_offset = array_offset + 8 * i
            item = struct.unpack(">d", message[item_offset : item_offset + 8])[0]
            val.append(item)
        return (key, val)
    elif val_type == 6:  # long long
        val = struct.unpack(">q", message[val_offset : val_offset + 8])[0]
        return (key, val)
    elif val_type == 7:  # string
        string_len = struct.unpack(">i", message[val_offset : val_offset + 4])[0]
        str_offset = val_offset + 4
        val = message[str_offset : str_offset + string_len].decode("us-ascii")
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
