""" This is a camera for desktop testing. """

# pylint: disable=consider-using-with
from mmap import mmap
from contextlib import AbstractContextManager, nullcontext
from tempfile import TemporaryFile
from typing import Any

from app.camera import Size


class FakeRequest:
    def __init__(self) -> None:
        self.tempfile = TemporaryFile()
        self.tempfile.write(b"foo")
        self.tempfile.seek(0)
        self.mmap = mmap(self.tempfile.fileno(), 0)

    def release(self) -> None:
        pass

    def buffer(self) -> AbstractContextManager[mmap]:
        return nullcontext(self.mmap)

    def metadata(self) -> dict[str, Any]:
        return {}


class FakeCamera:
    def capture_request(self) -> FakeRequest:
        return FakeRequest()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_size(self) -> Size:
        return Size(fullwidth=0, fullheight=0, width=0, height=0)
