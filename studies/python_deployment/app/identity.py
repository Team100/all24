from enum import unique, Enum
from typing import Any


def _read_cpu_info() -> str:
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""


try:
    # the raspberry pi 4 puts a sort-of-unique number here
    # TODO: does the rpi 5 do the same thing?
    _SERIAL = _read_cpu_info()
except FileNotFoundError:
    # windows doesn't have this file
    _SERIAL = ""


@unique
class Identity(Enum):
    """Keep this synchronized with java team100.config.Camera."""

    SHOOTER = "10000000a7a892c0"
    RIGHTAMP = "10000000caeaae82"
    LEFTAMP = "100000004e0a1fb9"
    GAME_PIECE = "1000000013c9c96c"
    DEV = "10000000a7c673d9" # rpi4 used for development
    FLIPPED = "flipme"  # example for per-identity config
    UNKNOWN = "unknown"

    @classmethod
    def _missing_(cls, value: object) -> Any:
        return Identity.UNKNOWN

    @staticmethod
    def get() -> "Identity":
        serial = _SERIAL
        print(f"Coprocessor serial: {serial}")
        identity: Identity = Identity(serial)
        print(f"Coprocessor identity: {identity.name}")
        return identity
