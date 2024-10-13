""" This is a camera for desktop testing."""

# pylint: disable=E1101,R0903,R1732

from contextlib import AbstractContextManager, nullcontext
from mmap import mmap
from pathlib import Path
from tempfile import SpooledTemporaryFile
from typing import Any

import cv2
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray
from typing_extensions import override

from app.camera.camera_protocol import Camera, Request, Size
from app.util.timer import Timer

Mat = NDArray[np.uint8]


class FakeRequest(Request):
    def __init__(self, img: MatLike) -> None:
        # make a copy; we use a temp file so we can use mmap,
        # which is what the consumer wants
        self.tempfile = SpooledTemporaryFile(max_size=500000)
        self.tempfile.write(img.data)
        self.tempfile.seek(0)
        # zero below means 8 bit grayscale
        # see https://docs.opencv.org/3.4/d8/d6a/group__imgcodecs__flags.html
        self.mmap = mmap(self.tempfile.fileno(), 0)
        self.mmap.seek(0)

    @override
    def release(self) -> None:
        self.tempfile.close()

    @override
    def rgb(self) -> AbstractContextManager[mmap]:
        return nullcontext(self.mmap)

    @override
    def yuv(self) -> AbstractContextManager[mmap]:
        return nullcontext(self.mmap)

    @override
    def metadata(self) -> dict[str, Any]:
        return {"SensorTimestamp": Timer.time_ns(), "FrameDuration": 300}


class FakeCamera(Camera):
    """Always returns the same image, which is from https://berndpfrommer.github.io/tagslam_web/making_tags/"""

    def __init__(self) -> None:
        p = Path(__file__).with_name("tag_and_board.png")
        # ok oops, this file is an 8-bit monochrome image, so when the tag
        # detector makes a length-limited array from it, it works.
        pathstr: str = str(p)
        self.img = cv2.imread(pathstr, 0)

    @override
    def capture_request(self) -> FakeRequest:
        return FakeRequest(self.img)

    @override
    def stop(self) -> None:
        pass

    @override
    def get_size(self) -> Size:
        return Size(fullwidth=1101, fullheight=619, width=1101, height=619)

    @override
    def get_intrinsic(self) -> Mat:
        return np.array(
            [
                [480, 0, 550],
                [0, 480, 310],
                [0, 0, 1],
            ]
        )

    @override
    def get_dist(self) -> Mat:
        return np.array([0, 0, 0, 0])


class BlindCamera(Camera):
    """Always returns the same image, a 100x100 white square."""

    def __init__(self) -> None:
        p = Path(__file__).with_name("white_square.png")
        # this is also 8-bit mono, so "Y"
        pathstr: str = str(p)
        self.img = cv2.imread(pathstr, 0)

    @override
    def capture_request(self) -> FakeRequest:
        return FakeRequest(self.img)

    @override
    def stop(self) -> None:
        pass

    @override
    def get_size(self) -> Size:
        return Size(fullwidth=100, fullheight=100, width=100, height=100)

    @override
    def get_intrinsic(self) -> Mat:
        return np.array(
            [
                [480, 0, 550],
                [0, 480, 310],
                [0, 0, 1],
            ]
        )

    @override
    def get_dist(self) -> Mat:
        return np.array([0, 0, 0, 0])


class NoteCamera(Camera):
    """Always returns the same image, an orange blob on a grey background."""

    def __init__(self) -> None:
        p = Path(__file__).with_name("blob.png")
        # this is an 8 bit indexed RGB image, which won't work directly as YUV input.
        pathstr: str = str(p)
        self.img = cv2.imread(pathstr, 1)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_RGB2YUV_I420)

    @override
    def capture_request(self) -> FakeRequest:
        return FakeRequest(self.img)

    @override
    def stop(self) -> None:
        pass

    @override
    def get_size(self) -> Size:
        return Size(fullwidth=100, fullheight=100, width=100, height=100)

    @override
    def get_intrinsic(self) -> Mat:
        return np.array(
            [
                [480, 0, 550],
                [0, 480, 310],
                [0, 0, 1],
            ]
        )

    @override
    def get_dist(self) -> Mat:
        return np.array([0, 0, 0, 0])
