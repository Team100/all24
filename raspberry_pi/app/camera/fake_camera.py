""" This is a camera for desktop testing."""

# pylint: disable=E1101,R0903,R1732

from contextlib import AbstractContextManager, nullcontext
from mmap import mmap
from pathlib import Path
from tempfile import TemporaryFile
from typing import Any, Optional

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
        """img should be cv2 RGB (really BGR)"""
        # Makes a copy so we can write into the image.
        # Uses a temp file so we can return mmap.
        self.img = img
        self.tempfile = TemporaryFile()

    @override
    def release(self) -> None:
        self.tempfile.close()

    @override
    def rgb(self) -> AbstractContextManager[mmap]:
        return self.make_context(self.img)

    @override
    def yuv(self) -> AbstractContextManager[mmap]:
        yuv = cv2.cvtColor(self.img, cv2.COLOR_RGB2YUV_I420)
        return self.make_context(yuv)

    def make_context(self, mat: MatLike) -> AbstractContextManager[mmap]:
        self.tempfile.truncate(0)
        self.tempfile.write(mat.data)
        self.tempfile.seek(0)
        val = mmap(self.tempfile.fileno(), 0)
        val.seek(0)
        return nullcontext(val)

    @override
    def metadata(self) -> dict[str, Any]:
        return {"SensorTimestamp": Timer.time_ns(), "FrameDuration": 300}


class FakeCamera(Camera):
    def __init__(self, filename: str, size: Optional[tuple[int, int]] = None) -> None:
        """If no size is supplied, the native size is used."""
        p = Path(__file__).with_name(filename)
        pathstr: str = str(p)
        self.img = cv2.imread(pathstr)
        if size is not None:
            self.img = cv2.resize(self.img, size)
        self.h = self.img.shape[0]
        self.w = self.img.shape[1]
        self.c = self.img.shape[2]

    @override
    def capture_request(self) -> FakeRequest:
        return FakeRequest(self.img)

    @override
    def stop(self) -> None:
        pass

    @override
    def get_size(self) -> Size:
        return Size(
            sensor_width=self.w,
            sensor_height=self.h,
            width=self.w,
            height=self.h,
        )

    @override
    def get_intrinsic(self) -> Mat:
        return np.array(
            [
                [480, 0, self.w / 2],
                [0, 480, self.h / 2],
                [0, 0, 1],
            ]
        )

    @override
    def get_dist(self) -> Mat:
        return np.array([0, 0, 0, 0])
    
    @override
    def is_rolling_shutter(self) -> bool:
        return True
