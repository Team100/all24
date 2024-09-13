"""
Bridge UDP messages to network tables for dashboard and logging.

This acts as the NT server, and writes a log file to a local USB.

Using DataLogManager to combine these functions did not work, it stopped
writing at about 8 kB.

TODO: maybe consolidate this into one file to make it easier to deploy,
or alternatively combine it with the other python stuff.
"""

import datetime
import socket
from typing import Any

import ntcore
from wpiutil.log import (
    DataLog,
    BooleanLogEntry,
    DoubleLogEntry,
    IntegerLogEntry,
    DoubleArrayLogEntry,
    StringLogEntry,
    RawLogEntry,
)

from udp_listener import decode
from udp_primitive_protocol import Types

# For this log directory to work, you have to have a USB stick mounted
# in the pi, and to do that, you need this line in /etc/fstab:
# /dev/sda1 /media/usb1 vfat defaults,dmask=000,fmask=111 0 0
LOG_DIR = "/media/usb1"

# This matches the log file name that the RoboRIO uses, but not for
# FMS-managed cases (where it inserts a character for match type and number)
LOG_FILENAME = datetime.datetime.now(tz=datetime.timezone.utc).strftime(
    "FRC_%Y%m%d_%H%M%S.wpilog"
)

# We have to hang on to the publishers to keep them from disappearing.
publishers = {}
# The log file uses a "handle" for each column so we have to remember them.
entries = {}


def main() -> None:
    """Run forever"""
    print("starting...")
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer()

    log_file = DataLog(dir=LOG_DIR, filename=LOG_FILENAME)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_socket.bind(("", 1995))
    while True:
        message, _ = server_socket.recvfrom(1024)
        (key, val_type, val) = decode(message)
        # print(f"key: {key} val_type: {val_type} actual type {type(val)} val: {val}")

        # write to network tables and flush immediately
        if key not in publishers:
            t: Any
            match val_type:
                case Types.BOOLEAN:
                    t = inst.getBooleanTopic(key)
                case Types.DOUBLE:
                    t = inst.getDoubleTopic(key)
                case Types.INT | Types.LONG:
                    t = inst.getIntegerTopic(key)
                case Types.DOUBLE_ARRAY:
                    t = inst.getDoubleArrayTopic(key)
                case Types.STRING:
                    t = inst.getStringTopic(key)
                case _:
                    t = inst.getRawTopic(key)
            p = t.publish(options=ntcore.PubSubOptions(keepDuplicates=True))
            t.setRetained(True)
            publishers[key] = p
        else:
            p = publishers[key]
        p.set(val)
        inst.flush()

        # write to the USB without flushing
        if key not in entries:
            e: Any
            match val_type:
                case Types.BOOLEAN:
                    e = BooleanLogEntry(log_file, key)
                case Types.DOUBLE:
                    e = DoubleLogEntry(log_file, key)
                case Types.INT | Types.LONG:
                    e = IntegerLogEntry(log_file, key)
                case Types.DOUBLE_ARRAY:
                    e = DoubleArrayLogEntry(log_file, key)
                case Types.STRING:
                    e = StringLogEntry(log_file, key)
                case _:
                    e = RawLogEntry(log_file, key)
            entries[key] = e
        else:
            e = entries[key]
        e.append(val)


if __name__ == "__main__":
    main()
