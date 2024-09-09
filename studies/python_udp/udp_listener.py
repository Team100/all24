""" Test to receive UDP packets. """

import socket
import struct
from typing import Any

from udp_primitive_protocol import Types


def decode(message) -> tuple[str, Types, Any]:
    """
    Decode a message into (key, type, value).
    TODO: support network tables time-alignment.
    """
    key_len = struct.unpack(">B", message[0:1])[0]
    key: str = message[1 : key_len + 1].decode("us-ascii")
    type_offset = key_len + 1
    type_id = struct.unpack(">B", message[type_offset : type_offset + 1])[0]
    val_type: Types = Types(type_id)
    val_offset = type_offset + 1
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
        case _:
            val = None

    return (key, val_type, val)


def main() -> None:
    """Run forever, for testing only."""
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", 1995))
    while True:
        message, _ = server_socket.recvfrom(1024)
        (key, val_type, val) = decode(message)
        print(f"key: {key} val_type: {val_type} val: {val}")


if __name__ == "__main__":
    main()
