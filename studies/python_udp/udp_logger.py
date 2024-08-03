""" Bridge UDP messages network tables. """

import socket

import ntcore
import wpilib

from udp_listener import decode
from udp_primitive_protocol import Types

def main():
    """Run forever"""
    inst = ntcore.NetworkTableInstance.getDefault()
    wpilib.DataLogManager.start()
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", 1995))
    while True:
        message, _ = server_socket.recvfrom(1024)
        (key, val_type, val) = decode(message)
        print(f"key: {key} val_type: {val_type} actual type {type(val)} val: {val}")
        match val_type:
            case Types.BOOLEAN:
                inst.getBooleanTopic(key).publish().set(val)
            case Types.DOUBLE:
                inst.getDoubleTopic(key).publish().set(val)
            case Types.INT:
                inst.getIntegerTopic(key).publish().set(val)
            case Types.DOUBLE_ARRAY:
                inst.getDoubleArrayTopic(key).publish().set(val)
            case Types.LONG:
                inst.getIntegerTopic(key).publish().set(val)
            case Types.STRING:
                inst.getStringTopic(key).publish().set(val)


if __name__ == "__main__":
    main()
