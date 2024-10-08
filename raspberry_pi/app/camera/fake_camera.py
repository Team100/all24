""" This is a camera for desktop testing.
"""

# pylint: disable=E1101

from mmap import mmap
from pathlib import Path
from contextlib import AbstractContextManager, nullcontext
from tempfile import TemporaryFile
from typing import Any

import numpy as np
import cv2
from numpy.typing import NDArray

from app.camera.camera_protocol import Camera, Request, Size
from app.util.timer import Timer

Mat = NDArray[np.uint8]


class FakeRequest(Request):
    def __init__(self, pathstr: str) -> None:
        img = cv2.imread(pathstr, 0)

        self.tempfile = TemporaryFile()
        # black square
        # self.tempfile.write(bytearray(1000000))
        self.tempfile.write(img.data)
        self.tempfile.seek(0)
        # zero below means 8 bit grayscale
        # see https://docs.opencv.org/3.4/d8/d6a/group__imgcodecs__flags.html
        self.mmap = mmap(self.tempfile.fileno(), 0)

    def release(self) -> None:
        pass

    def buffer(self) -> AbstractContextManager[mmap]:
        return nullcontext(self.mmap)

    def metadata(self) -> dict[str, Any]:
        return {"SensorTimestamp": Timer.time_ns(), "FrameDuration": 300}


class FakeCamera(Camera):
    def __init__(self) -> None:
        # this image is from https://berndpfrommer.github.io/tagslam_web/making_tags/
        p = Path(__file__).with_name("tag_and_board.png")
        self.pathstr: str = str(p)
        # print(f"Using fake camera with file {self.pathstr}")

    def capture_request(self) -> FakeRequest:
        return FakeRequest(self.pathstr)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_size(self) -> Size:
        return Size(fullwidth=1101, fullheight=619, width=1101, height=619)

    def get_intrinsic(self) -> Mat:
        return np.array(
            [
                [480, 0, 550],
                [0, 480, 310],
                [0, 0, 1],
            ]
        )

    def get_dist(self) -> Mat:
        return np.array([0, 0, 0, 0])
