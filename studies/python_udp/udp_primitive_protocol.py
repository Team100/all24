""" Encoding types. """

from enum import Enum


class Types(Enum):
    """Keep this in sync with the java enum."""

    BOOLEAN = 1
    DOUBLE = 2
    INT = 3
    DOUBLE_ARRAY = 4
    LONG = 5
    STRING = 6
    UNKNOWN = None

    @classmethod
    def _missing_(cls, value):
        return Types.UNKNOWN
