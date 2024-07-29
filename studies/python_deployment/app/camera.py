""" Interface spec for camera types.

To learn about Protocols for interface specification, see
https://typing.readthedocs.io/en/latest/spec/protocol.html
"""


# pylint: disable=too-few-public-methods


from mmap import mmap
from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray


Mat = NDArray[np.uint8]


class Request(Protocol):
    def release(self) -> None: ...
    def buffer(self) -> AbstractContextManager[mmap]: ...
    def metadata(self) -> dict[str, Any]: ...


@dataclass(frozen=True, kw_only=True)
class Size:
    fullwidth: int
    fullheight: int
    width: int
    height: int


class Camera(Protocol):
    def capture_request(self) -> Request: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def get_size(self) -> Size: ...
    def get_intrinsic(self) -> Mat: ...
    def get_dist(self) -> Mat: ...


