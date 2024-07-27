from enum import unique, Enum
from typing import Any


@unique
class Identity(Enum):
    """Keep this synchronized with java team100.config.Camera."""

    SHOOTER = "10000000a7a892c0"
    RIGHTAMP = "10000000caeaae82"
    LEFTAMP = "100000004e0a1fb9"
    GAME_PIECE = "1000000013c9c96c"
    FLIPPED = "flipme" # example for per-identity config
    UNKNOWN = "unknown"

    @classmethod
    def _missing_(cls, value: object) -> Any:
        return Identity.UNKNOWN

    @staticmethod
    def get() -> "Identity":
        serial = Identity.get_serial()
        print(f"Coprocessor serial: {serial}")
        identity: Identity = Identity(serial)
        print(f"Coprocessor identity: {identity.name}")
        return identity

    @staticmethod
    def get_serial() -> str:
        with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
            for line in cpuinfo:
                if line[0:6] == "Serial":
                    return line[10:26]
        return ""
