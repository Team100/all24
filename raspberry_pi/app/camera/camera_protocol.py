""" Interface spec for camera types.

To learn about Protocols for interface specification, see
https://typing.readthedocs.io/en/latest/spec/protocol.html
"""

# pylint: disable=R0903


from mmap import mmap
from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

Mat = NDArray[np.uint8]


class Request(Protocol):
    def metadata(self) -> dict[str, Any]: ...
    def rgb(self) -> AbstractContextManager[mmap]: ...
    def yuv(self) -> AbstractContextManager[mmap]: ...
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
