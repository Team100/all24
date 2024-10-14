""" Annotate and show the captured image through the CameraServer."""

# pylint: disable=C0415,E0611

from platform import system

import numpy as np
from cv2 import (FONT_HERSHEY_SIMPLEX, circle, drawContours, line, putText,
                 resize)
from cv2.typing import MatLike
from numpy.typing import NDArray
from robotpy_apriltag import AprilTagDetection
from typing_extensions import override
from wpimath.geometry import Transform3d

from app.dashboard.display import Display

FONT = FONT_HERSHEY_SIMPLEX
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

Mat = NDArray[np.uint8]


class RealDisplay(Display):
    def __init__(self, width: int, height: int, name: str) -> None:
        self.width = width
        self.height = height
        if system() == "Windows":
            from app.dashboard.mjpeg_streamer import MjpegServer, Stream

            print("Using MJpegServer for Windows")
            # on windows, cvsource breaks with cvnp contiguous-array error
            self._stream = Stream(name, (width, height), quality=50, fps=30)
            self._server = MjpegServer("localhost", 1181)
            self._server.add_stream(self._stream)
            self._server.start()
        else:
            from cscore import CameraServer

            print("Using CameraServer for Linux.  See localhost:1181,1182,etc")
            self._cvsource = CameraServer.putVideo(name, width, height)


    @override
    def tag(self, image: MatLike, tag: AprilTagDetection, pose: Transform3d) -> None:
        # TODO use corners instead of detection

        # Draw lines around the tag
        for i in range(4):
            j = (i + 1) % 4
            point1 = (int(tag.getCorner(i).x), int(tag.getCorner(i).y))
            point2 = (int(tag.getCorner(j).x), int(tag.getCorner(j).y))
            line(image, point1, point2, WHITE, 2)

        (c_x, c_y) = (int(tag.getCenter().x), int(tag.getCenter().y))
        circle(image, (c_x, c_y), 10, WHITE, -1)

        tag_id = tag.getId()
        self.text(image, f"id {tag_id}", (c_x, c_y))

        # type the translation into the image, in WPI coords (x-forward)

        t = pose.translation()
        self.text(
            image,
            f"t: {t.z:4.1f},{-t.x:4.1f},{-t.y:4.1f}",
            (c_x - 50, c_y + 40),
        )

    @override
    def note(self, image: MatLike, contour: MatLike, c_x: int, c_y: int) -> None:
        drawContours(image, [contour], -1, (0, 255, 0), 3)
        circle(image, (c_x, c_y), 7, (0, 0, 0), -1)

    # these are white with black outline
    @override
    def text(self, image: MatLike, msg: str, loc: tuple[int, int]) -> None:
        putText(image, msg, loc, FONT, 1.5, BLACK, 6)
        putText(image, msg, loc, FONT, 1.5, WHITE, 2)

    @override
    def put(self, img: MatLike) -> None:
        img_out = resize(img, (self.width, self.height))
        #img_out = img
        if system() == "Windows":
            self._stream.set_frame(img_out)  # type: ignore
        else:
            self._cvsource.putFrame(img_out)  # type: ignore
