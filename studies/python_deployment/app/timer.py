# pylint: disable=too-few-public-methods

from time import clock_gettime_ns, CLOCK_BOOTTIME


class Timer:
    @staticmethod
    def time_ns() -> int:
        """ Nanoseconds since boot, aligns with SensorTimestamp. """
        return clock_gettime_ns(CLOCK_BOOTTIME)
