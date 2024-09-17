# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621
"""
Decode metadata packets (after timestamp), yield (key, type, label).
"""
from typing import Any, Generator
from udp_parser import parse_byte, parse_short, parse_string
from udp_primitive_protocol import Types


def meta_decode(
    message: bytes, offset: int
) -> Generator[tuple[int, Types, Any], None, None]:
    """
    message is a list of (key (2), type (1), length (1), bytes (N))
    yields (key, type, label)
    throws struct.error if parse fails
    """
    while offset < len(message):
        key, offset = parse_short(message, offset)
        type_id, offset = parse_byte(message, offset)
        val_type: Types = Types(type_id)
        label, offset = parse_string(message, offset)
        yield (key, val_type, label)
