""" Interface spec for camera types. """

# pylint: disable=R0903

from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray
from typing_extensions import Buffer

Mat = NDArray[np.uint8]


class Request(Protocol):
    def metadata(self) -> dict[str, Any]: ...
    def rgb(self) -> AbstractContextManager[Buffer]: ...
    def yuv(self) -> AbstractContextManager[Buffer]: ...
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
