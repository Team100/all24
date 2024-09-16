# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621

import struct
from typing import Any


def parse(fmt: str, buf: bytes, offset: int) -> tuple[Any, int]:
    return struct.unpack_from(fmt, buf, offset)[0], offset + struct.calcsize(fmt)


def parse_string(buf: bytes, offset: int, string_len) -> tuple[str, int]:
    return buf[offset : offset + string_len].decode("us-ascii"), offset + string_len


def parse_timestamp(buf: bytes, offset: int) -> tuple[int, int]:
    return parse(">q", buf, offset)


def parse_boolean(buf: bytes, offset: int) -> tuple[bool, int]:
    return parse(">?", buf, offset)


def parse_double(buf: bytes, offset: int) -> tuple[float, int]:
    return parse(">d", buf, offset)
