# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0603,W0621
"""
Decode data packets (after timestamp), yield (key, type, data).
"""
from typing import Any, Generator
from udp_parser import (
    parse_boolean,
    parse_byte,
    parse_double,
    parse_double_array,
    parse_int,
    parse_short,
    parse_string,
    parse_long,
)
from udp_primitive_protocol import Types

DATA_PORT = 1995
MTU = 1500


def data_decode(
    message: bytes, offset: int
) -> Generator[tuple[int, Types, Any], None, None]:
    """
    message is (key (2), type (1), data (varies))
    yields (key, type, data)
    throws struct.error if parse fails
    """
    while offset < len(message):
        key, offset = parse_short(message, offset)
        type_id, offset = parse_byte(message, offset)
        val_type: Types = Types(type_id)
        match val_type:
            case Types.BOOLEAN:
                bool_val, offset = parse_boolean(message, offset)
                yield (key, val_type, bool_val)

            case Types.DOUBLE:
                double_val, offset = parse_double(message, offset)
                yield (key, val_type, double_val)

            case Types.INT:
                int_val, offset = parse_int(message, offset)
                yield (key, val_type, int_val)

            case Types.DOUBLE_ARRAY:
                array_val, offset = parse_double_array(message, offset)
                yield (key, val_type, array_val)

            case Types.LONG:
                long_val, offset = parse_long(message, offset)
                yield (key, val_type, long_val)

            case Types.STRING:
                string_val, offset = parse_string(message, offset)
                yield (key, val_type, string_val)

            case _:
                print(f"weird key {key} at offset {offset}")
