"""
This is a clock that works on both Raspberry Pi and on Windows, for testing.
"""

# pylint: disable=R0903

try:
    # this is the linux/raspberry pi version.
    from time import clock_gettime_ns, CLOCK_BOOTTIME  # type:ignore

    def _time() -> int:
        """Nanoseconds since boot, aligns with SensorTimestamp."""
        return clock_gettime_ns(CLOCK_BOOTTIME)  # type:ignore

except ImportError:
    # this is the windows version
    from time import time_ns

    def _time() -> int:
        return time_ns()


class Timer:
    @staticmethod
    def time_ns() -> int:
        """Nanoseconds since boot, aligns with SensorTimestamp."""
        return _time()
