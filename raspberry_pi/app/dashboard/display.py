""" Interface spec for dashboard video."""

from typing import Protocol

import numpy as np
from cv2.typing import MatLike
from numpy.typing import NDArray
from robotpy_apriltag import AprilTagDetection
from wpimath.geometry import Transform3d

Mat = NDArray[np.uint8]


class Display(Protocol):
    def tag(
        self, image: MatLike, tag: AprilTagDetection, pose: Transform3d
    ) -> None: ...
    def note(self, image: MatLike, contour: MatLike, c_x: int, c_y: int) -> None: ...
    def text(self, image: MatLike, msg: str, loc: tuple[int, int]) -> None: ...
    def put(self, img: MatLike) -> None: ...
