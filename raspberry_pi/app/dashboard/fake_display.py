""" A display for unit tests."""

import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray
from robotpy_apriltag import AprilTagDetection
from typing_extensions import override
from wpimath.geometry import Transform3d

from app.dashboard.display import Display

Mat = NDArray[np.uint8]


class FakeDisplay(Display):
    def __init__(self) -> None:
        self.tags: list[AprilTagDetection] = []
        self.poses: list[Transform3d] = []
        self.notes: list[MatLike] = []
        self.circles: list[tuple[int, int]] = []
        self.msgs: list[str] = []
        self.locs: list[tuple[int, int]] = []
        self.frame_count = 0

    @override
    def tag(self, image: MatLike, tag: AprilTagDetection, pose: Transform3d) -> None:
        self.tags.append(tag)
        self.poses.append(pose)

    @override
    def note(self, image: MatLike, contour: MatLike, c_x: int, c_y: int) -> None:
        self.notes.append(contour)
        self.circles.append((c_x, c_y))

    @override
    def text(self, image: MatLike, msg: str, loc: tuple[int, int]) -> None:
        self.msgs.append(msg)
        self.locs.append(loc)

    @override
    def put(self, img: MatLike) -> None:
        self.frame_count += 1
