""" Annotate and show the captured image through the CameraServer. """

# pylint: disable=no-name-in-module

from platform import system
import numpy as np
from numpy.typing import NDArray
from cscore import CameraServer
from robotpy_apriltag import AprilTagDetection
from cv2 import circle, line, putText, resize, FONT_HERSHEY_SIMPLEX
from wpimath.geometry import Transform3d

from app.mjpeg_streamer import MjpegServer, Stream

FONT = FONT_HERSHEY_SIMPLEX
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

Mat = NDArray[np.uint8]


class Display:
    def __init__(self, width: int, height: int, camera_num: str) -> None:
        print("width ", width)
        print("height ", height)
        if system() == "Windows":
            print("Windows")
            # on windows, cvsource breaks with cvnp contiguous-array error
            self.stream = Stream("Processed", (width, height), quality=50, fps=30)
            self.server = MjpegServer("localhost", 1181)
            self.server.add_stream(self.stream)
            self.server.start()
        else:
            self.cvsource = CameraServer.putVideo(camera_num, 416, 308)

    def draw_result(
        self, image: Mat, result_item: AprilTagDetection, pose: Transform3d
    ) -> None:
        # TODO use corners instead of detection

        # Draw lines around the tag
        for i in range(4):
            j = (i + 1) % 4
            point1 = (int(result_item.getCorner(i).x), int(result_item.getCorner(i).y))
            point2 = (int(result_item.getCorner(j).x), int(result_item.getCorner(j).y))
            line(image, point1, point2, WHITE, 2)

        (c_x, c_y) = (int(result_item.getCenter().x), int(result_item.getCenter().y))
        circle(image, (c_x, c_y), 10, WHITE, -1)

        tag_id = result_item.getId()
        self.draw_text(image, f"id {tag_id}", (c_x, c_y))

        # type the translation into the image, in WPI coords (x-forward)
        if pose is not None:
            t = pose.translation()
            self.draw_text(
                image,
                f"t: {t.z:4.1f},{-t.x:4.1f},{-t.y:4.1f}",
                (c_x - 50, c_y + 40),
            )

    # these are white with black outline
    def draw_text(self, image: Mat, msg: str, loc: tuple[int, int]) -> None:
        putText(image, msg, loc, FONT, 1.5, BLACK, 6)
        putText(image, msg, loc, FONT, 1.5, WHITE, 2)

    def put_frame(self, img: Mat) -> None:
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
            self.stream.set_frame(img_out)
        else:
            self.cvsource.putFrame(img_out)
