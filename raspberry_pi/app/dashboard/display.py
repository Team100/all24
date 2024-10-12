"""Interface spec for dashboard video
"""

from typing import Protocol

import numpy as np
from numpy.typing import NDArray
from cv2.typing import MatLike
from robotpy_apriltag import AprilTagDetection
from wpimath.geometry import Transform3d

Mat = NDArray[np.uint8]


class Display(Protocol):
    def draw_result(
        self, image: Mat, result_item: AprilTagDetection, pose: Transform3d
    ) -> None: ...
    def draw_text(self, image: MatLike, msg: str, loc: tuple[int, int]) -> None: ...
    def put_frame(self, img: MatLike) -> None: ...
