# pylint: disable=C0103,C0114,C0115,C0116,E0601,E0611,R0904,R0913,W0603,W0621
"""
Bridge UDP messages to network tables for dashboard and logging.

This acts as the NT server, and writes a log file to a local USB.
"""

import datetime
import socket
import time
import threading
import traceback
import struct
from typing import Any

from ntcore import NetworkTableInstance, PubSubOptions

from wpiutil.log import (
    DataLog,
    BooleanLogEntry,
    DoubleLogEntry,
    IntegerLogEntry,
    DoubleArrayLogEntry,
    StringLogEntry,
)

from udp_data_decoder import data_decode
from udp_meta_decoder import meta_decode
from udp_parser import parse_long
from udp_primitive_protocol import Types

DATA_PORT = 1995
META_PORT = 1996
MTU = 1500


# write to network tables
PUB = False
# write to disk
LOG = False

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
publishers: dict[int, Any] = {}
# The log file uses a "handle" for each column so we have to remember them.
entries: dict[int, Any] = {}

# updated by main thread
data_rows = 0
meta_rows = 0
# updated by counter thread
prev_data_rows = 0
prev_meta_rows = 0


def stats() -> None:
    global prev_data_rows, prev_meta_rows
    interval = 1.0
    t0 = time.time()
    while True:
        d = time.time() - t0
        dt = d % interval
        time.sleep(interval - dt)
        d = time.time() - t0
        data_i = data_rows - prev_data_rows
        meta_i = meta_rows - prev_meta_rows
        prev_data_rows = data_rows
        prev_meta_rows = meta_rows
        print(f"sec: {d:.0f} data_rows: {data_i:d} meta_rows: {meta_i:d}")


def data_reader() -> None:
    global data_rows
    data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    data_socket.bind(("", DATA_PORT))
    while True:
        try:
            message: bytes = data_socket.recv(MTU)
            # message always starts with timestamp.
            timestamp, offset = parse_long(message, 0)
            # print(f"DATA timestamp {timestamp}")
            # TODO: new timestamp means new log file
            for key, val_type, val in data_decode(message, offset):
                data_rows += 1
                # print(f"DATA key: {key} val_type: {val_type} val: {val}")
                if PUB and key in publishers:
                    publishers[key].set(val)
                if LOG and key in entries:
                    entries[key].append(val)
        except struct.error:
            print("parse fail for input: ", message)
            traceback.print_exc()
            return

def meta_reader() -> None:
    global meta_rows
    if PUB:
        inst = NetworkTableInstance.getDefault()
        inst.startServer()

    if LOG:
        # log_file = DataLog(dir=LOG_DIR, filename=LOG_FILENAME)
        log_file = DataLog(period=0.1)

    meta_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    meta_socket.bind(("", META_PORT))
    while True:
        try:
            message: bytes = meta_socket.recv(MTU)
            # message always starts with timestamp.
            timestamp, offset = parse_long(message, 0)
            # print(f"META timestamp {timestamp}")
            # TODO: new timestamp means new log file
            for key, val_type, label in meta_decode(message, offset):
                meta_rows += 1
                # print(f"META key: {key} val_type: {val_type} label: {label}")
                if PUB and key not in publishers:
                    add_publisher(inst, key, val_type, label)
                if LOG and key not in entries:
                    add_entry(log_file, key, val_type, label)
        except struct.error:
            print("parse fail for input: ", message)
            traceback.print_exc()
            return

def add_publisher(inst, key, val_type, label) -> None:
    t: Any
    match val_type:
        case Types.BOOLEAN:
            t = inst.getBooleanTopic(label)
        case Types.DOUBLE:
            t = inst.getDoubleTopic(label)
        case Types.INT | Types.LONG:
            t = inst.getIntegerTopic(label)
        case Types.DOUBLE_ARRAY:
            t = inst.getDoubleArrayTopic(label)
        case Types.STRING:
            t = inst.getStringTopic(label)
        case _:
            print(f"skip unknown type {val_type} for key {key}")
            return
    p = t.publish(options=PubSubOptions(keepDuplicates=True))
    t.setRetained(True)
    publishers[key] = p

def add_entry(log_file, key, val_type, label) -> None:
    match val_type:
        case Types.BOOLEAN:
            entries[key] = BooleanLogEntry(log_file, label)
        case Types.DOUBLE:
            entries[key] = DoubleLogEntry(log_file, label)
        case Types.INT | Types.LONG:
            entries[key] = IntegerLogEntry(log_file, label)
        case Types.DOUBLE_ARRAY:
            entries[key] = DoubleArrayLogEntry(log_file, label)
        case Types.STRING:
            entries[key] = StringLogEntry(log_file, label)
        case _:
            print(f"skip unknown type {val_type} for key {key}")


def main() -> None:
    data_thread = threading.Thread(target=data_reader)
    meta_thread = threading.Thread(target=meta_reader)
    monitor = threading.Thread(target=stats)
    data_thread.start()
    meta_thread.start()
    monitor.start()
    data_thread.join()  # blocks forever


if __name__ == "__main__":
    main()
