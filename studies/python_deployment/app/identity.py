from enum import unique, Enum
from typing import Any


@unique
class Identity(Enum):
    """Keep this synchronized with java team100.config.Camera."""

    SHOOTER = "10000000a7a892c0"
    RIGHTAMP = "10000000caeaae82"
    LEFTAMP = "100000004e0a1fb9"
    GAME_PIECE = "1000000013c9c96c"
    UNKNOWN = "unknown"

    @classmethod
    def _missing_(cls, value: object) -> Any:
        return Identity.UNKNOWN

    @staticmethod
    def get() -> "Identity":
        serial = Identity.get_serial()
        return Identity(serial)

    @staticmethod
    def get_serial() -> str:
        with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
            for line in cpuinfo:
                if line[0:6] == "Serial":
                    return line[10:26]
        return ""
