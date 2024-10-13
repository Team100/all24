""" Annotate and show the captured image through the CameraServer.
"""

# pylint: disable=E0611

from platform import system

import numpy as np
from cscore import CameraServer
from cv2 import (FONT_HERSHEY_SIMPLEX, circle, drawContours, line, putText,
                 resize)
from cv2.typing import MatLike
from numpy.typing import NDArray
from robotpy_apriltag import AprilTagDetection
from typing_extensions import override
from wpimath.geometry import Transform3d

from app.dashboard.display import Display
from app.dashboard.mjpeg_streamer import MjpegServer, Stream

FONT = FONT_HERSHEY_SIMPLEX
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

Mat = NDArray[np.uint8]


class RealDisplay(Display):
    def __init__(self, width: int, height: int, name: str) -> None:
        if system() == "Windows":
            print("Using MJpegServer for Windows")
            # on windows, cvsource breaks with cvnp contiguous-array error
            # i think (width,height) are optional, it will use frame shape.
            # TODO: remove width,height
            self._stream = Stream(name, (width, height), quality=50, fps=30)
            self._server = MjpegServer("localhost", 1181)
            self._server.add_stream(self._stream)
            self._server.start()
        else:
            print("Using CameraServer for Linux")
            self._cvsource = CameraServer.putVideo(name, 416, 308)

    @override
    def tag(
        self, image: MatLike, tag: AprilTagDetection, pose: Transform3d
    ) -> None:
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
        drawContours(image, [contour], -1, (0, 255, 0), 2)
        circle(image, (c_x, c_y), 7, (0, 0, 0), -1)

    # these are white with black outline
    @override
    def text(self, image: MatLike, msg: str, loc: tuple[int, int]) -> None:
        putText(image, msg, loc, FONT, 1.5, BLACK, 6)
        putText(image, msg, loc, FONT, 1.5, WHITE, 2)

    @override
    def put(self, img: MatLike) -> None:
        # connect to localhost:1181 to see this
        # windows complains about noncontiguous
        # img = np.zeros((100,100), dtype=np.uint8)
        # print("shape ", img.shape)
        # print("itemsize ", img.itemsize)
        # print("strides ", img.strides)
        # print("ndim ", img.ndim)
        # print("dtype ", img.dtype)
        # self.output_stream.putFrame(np.ascontiguousarray(img))
        #
        # shrink the driver view to avoid overloading the radio
        #
        # for now put big images
        # TODO: turn this off for prod!!
        img_out = resize(img, (416, 308))
        if system() == "Windows":
            self._stream.set_frame(img_out)  # type: ignore
        else:
            self._cvsource.putFrame(img_out)  # type: ignore
