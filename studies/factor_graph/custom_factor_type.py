from enum import Enum


class CustomFactorType(Enum):
    UNKNOWN = 0
    BETWEEN = 1
    BOUNDARY = 2
    BEARING = 3
    RANGE = 4
    UNARY = 5
    OTHER = 6
