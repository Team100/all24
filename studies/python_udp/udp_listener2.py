# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

import socket
import struct
import time
import threading
from typing import Any, Iterator

from udp_primitive_protocol import Types


def decode(message: bytes) -> Iterator[tuple[int, Types, Any]]:
    """
    returns (key, type, data)
    Message format v2.  See UdpPrimitiveProtocol2.java.
    """
    print(len(message))
    print(message)
    try:
        offset = 0
        while True:
            # loop over sections
            type_id: int = struct.unpack(">H", message[offset : offset + 2])[0]
            offset += 2
            val_type: Types = Types(type_id)
            print("val type ", val_type)
            while True:
                if offset >= len(message):
                    return
                # loop over key-value pairs
                key: int = struct.unpack(">H", message[offset : offset + 2])[0]
                offset += 2
                if key < 16:
                    # change types
                    val_type = Types(key)
                    print("val type ", val_type)
                    continue
                val: Any
                match val_type:
                    case Types.BOOLEAN:
                        val = struct.unpack(">?", message[offset : offset + 1])[0]
                        offset += 1
                    case Types.DOUBLE:
                        val = struct.unpack(">d", message[offset : offset + 8])[0]
                        offset += 8
                    case Types.INT:
                        val = struct.unpack(">i", message[offset : offset + 4])[0]
                        offset += 4
                    case Types.DOUBLE_ARRAY:
                        array_len: int = struct.unpack(">B", message[offset : offset + 1])[0]
                        offset += 1
                        val = []
                        for _ in range(array_len):
                            item = struct.unpack(">d", message[offset : offset + 8])[0]
                            offset += 8 
                            val.append(item)
                    case Types.LONG:
                        val = struct.unpack(">q", message[offset : offset + 8])[0]
                        offset += 8 
                    case Types.STRING:
                        string_len: int = struct.unpack(">B", message[offset : offset + 1])[0]
                        offset += 1
                        val = message[offset : offset + string_len].decode("us-ascii")
                        offset += string_len
                    case Types.LABEL:
                        # TODO: add label map parser
                        val = None
                    case _:
                        val = None
                yield (key, val_type, val)
    except struct.error as err:
        print(err)
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
        print(f"{d:.0f} {i:d}")


def recv() -> None:
    global n
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", 1995))
    while True:

        # message, _ = server_socket.recvfrom(1024)
        message: bytes = server_socket.recv(1500)
        for (key, val_type, val) in decode(message):
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
