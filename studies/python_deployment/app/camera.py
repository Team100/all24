# pylint: disable=import-outside-toplevel
# pylint: disable=too-few-public-methods

from mmap import mmap
from contextlib import AbstractContextManager
from dataclasses import dataclass
from typing import Any, Protocol
from app.identity import Identity


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


class Factory:
    @staticmethod
    def get(identity: Identity) -> Camera:
        try:
            # this will fail if we're not running on a Raspberry Pi.
            from app.real_camera import RealCamera

            return RealCamera(identity)

        except ImportError:
            from app.fake_camera import FakeCamera

            return FakeCamera()
