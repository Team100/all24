""" A camera for desktop testing."""

# pylint: disable=E1101,R0903,R1732

from contextlib import AbstractContextManager, nullcontext
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray
from typing_extensions import Buffer, override

from app.camera.camera_protocol import Camera, Request, Size
from app.util.timer import Timer

Mat = NDArray[np.uint8]


class FakeRequest(Request):
    def __init__(self, img: MatLike, fps: float) -> None:
        """img should be cv2 RGB (really BGR)"""
        self.img = img
        self._fps = fps

    @override
    def fps(self) -> float:
        return self._fps

    @override
    def delay_us(self) -> int:
        return 500

    @override
    def rgb(self) -> AbstractContextManager[Buffer]:
        return nullcontext(self.img.copy().data)

    @override
    def yuv(self) -> AbstractContextManager[Buffer]:
        return nullcontext(cv2.cvtColor(self.img, cv2.COLOR_RGB2YUV_I420).data)

    @override
    def release(self) -> None:
        pass


class FakeCamera(Camera):
    def __init__(
        self, filename: str, size: Optional[tuple[int, int]] = None, k1: float = 0.0
    ) -> None:
        """
        size: if no size is supplied, the native size is used.
        k1: quadratic distortion term, used to distort the image and returned in get_dist()
        """
        p = Path(__file__).with_name(filename)
        pathstr: str = str(p)
        self.img = cv2.imread(pathstr)
        if size is not None:
            self.img = cv2.resize(self.img, size)
        self.h = self.img.shape[0]
        self.w = self.img.shape[1]
        self.c = self.img.shape[2]
        self.frame_time = Timer.time_ns()
        self.k1 = k1
        # use undistort to distort the image, using the inverse
        mtx = self.get_intrinsic()
        dist = self.get_dist()
        dist[0] *= -1 # invert!
        self.img = cv2.undistort(self.img, mtx, dist)


    @override
    def capture_request(self) -> FakeRequest:
        capture_start: int = Timer.time_ns()
        total_time_ms = (capture_start - self.frame_time) / 1000000
        self.frame_time = capture_start
        fps = 1000 / total_time_ms
        return FakeRequest(self.img, fps)

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
        k1 = self.k1  # radial quadratic term
        k2 = 0  # radial quartic term
        p1 = 0  # tangential
        p2 = 0  # tangential
        return np.array([k1, k2, p1, p2])

    @override
    def is_rolling_shutter(self) -> bool:
        return True
