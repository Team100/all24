""" Interface spec for camera types. """

# pylint: disable=R0903,W2301

from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import NDArray
from typing_extensions import Buffer

Mat = NDArray[np.uint8]


class Request(Protocol):
    def fps(self) -> float:
        """FPS calculated from the previous capture."""
        ...

    def delay_us(self) -> int:
        """Duration between the capture instant of the center of the frame
        and the current instant, microseconds"""
        ...

    def rgb(self) -> AbstractContextManager[Buffer]:
        """Context-managed Buffer containing RGB888.
        Remember that when OpenCV says "RGB" it really means "BGR"
        github.com/raspberrypi/picamera2/issues/848"""
        ...

    def yuv(self) -> AbstractContextManager[Buffer]:
        """Context-managed Buffer containing YUV420."""
        ...

    def release(self) -> None: ...


@dataclass(frozen=True, kw_only=True)
class Size:
    """Sensor width and height must be equal to one of the 'size' options
    in the list of sensor formats."""

    sensor_width: int
    sensor_height: int
    width: int
    height: int


class Camera(Protocol):
    def capture_request(self) -> Request: ...
    def stop(self) -> None: ...
    def get_size(self) -> Size: ...
    def get_intrinsic(self) -> Mat: ...
    def get_dist(self) -> Mat: ...
    def is_rolling_shutter(self) -> bool: ...
