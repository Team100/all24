""" A display for unit tests.
"""

import numpy as np
from numpy.typing import NDArray
from robotpy_apriltag import AprilTagDetection
from wpimath.geometry import Transform3d

from app.dashboard.display import Display

Mat = NDArray[np.uint8]


class FakeDisplay(Display):
    def __init__(self) -> None:
        self.result_items: list[AprilTagDetection] = []
        self.poses: list[Transform3d] = []
        self.msgs: list[str] = []
        self.locs: list[tuple[int, int]] = []
        self.frame_count = 0

    def draw_result(
        self, image: Mat, result_item: AprilTagDetection, pose: Transform3d
    ) -> None:
        self.result_items.append(result_item)
        self.poses.append(pose)

    def draw_text(self, image: Mat, msg: str, loc: tuple[int, int]) -> None:
        self.msgs.append(msg)
        self.locs.append(loc)

    def put_frame(self, img: Mat) -> None:
        self.frame_count += 1
