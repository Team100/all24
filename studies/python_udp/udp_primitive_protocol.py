""" Encoding types. """

from enum import Enum


class Types(Enum):
    """Keep this in sync with the java enum."""

    LABEL_SECTION = 1
    DATA_SECTION = 2
    BOOLEAN = 3
    DOUBLE = 4
    INT = 5
    DOUBLE_ARRAY = 6
    LONG = 7
    STRING = 8
    UNKNOWN = None

    @classmethod
    def _missing_(cls, value):
        return Types.UNKNOWN
