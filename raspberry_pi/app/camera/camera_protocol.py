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


class HasMat(Protocol):
    @property
    def array(self) -> Mat: ...


class Request(Protocol):
    def release(self) -> None: ...
    def buffer(self) -> AbstractContextManager[mmap]: ...
    def array(self) -> AbstractContextManager[HasMat]: ...
    def metadata(self) -> dict[str, Any]: ...


@dataclass(frozen=True, kw_only=True)
class Size:
    fullwidth: int
    fullheight: int
    width: int
    height: int


class Camera(Protocol):
    def capture_request(self) -> Request: ...
    def stop(self) -> None: ...
    def get_size(self) -> Size: ...
    def get_intrinsic(self) -> Mat: ...
    def get_dist(self) -> Mat: ...
