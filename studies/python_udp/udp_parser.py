# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621
"""
This mainly exists to keep all the struct codes in one place.
"""
import struct
from typing import Any


def parse(fmt: str, buf: bytes, offset: int) -> tuple[Any, int]:
    return struct.unpack_from(fmt, buf, offset)[0], offset + struct.calcsize(fmt)


def parse_string(buf: bytes, offset: int) -> tuple[str, int]:
    """one-byte length followed by ascii string"""
    string_len, offset = parse_byte(buf, offset)
    return buf[offset : offset + string_len].decode("us-ascii"), offset + string_len


def parse_boolean(buf: bytes, offset: int) -> tuple[bool, int]:
    return parse(">?", buf, offset)


def parse_double(buf: bytes, offset: int) -> tuple[float, int]:
    return parse(">d", buf, offset)


def parse_int(buf: bytes, offset: int) -> tuple[int, int]:
    return parse(">i", buf, offset)


def parse_double_array(buf: bytes, offset: int) -> tuple[list[float], int]:
    """one-byte length followed by eight-byte doubles"""
    array_len, offset = parse_byte(buf, offset)
    array_val = []
    for _ in range(array_len):
        item, offset = parse_double(buf, offset)
        array_val.append(item)
    return array_val, offset


def parse_long(buf: bytes, offset: int) -> tuple[int, int]:
    """eight-byte long"""
    return parse(">q", buf, offset)


def parse_short(buf: bytes, offset: int) -> tuple[int, int]:
    """two-byte short"""
    return parse(">H", buf, offset)


def parse_byte(buf: bytes, offset: int) -> tuple[int, int]:
    """one byte"""
    return parse(">B", buf, offset)
