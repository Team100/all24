""" A wrapper for the note detector."""

# pylint: disable=C0103,E1101,R0902,R0903,R0913,R0914,W0612

from typing import cast

import cv2
import numpy as np
from numpy.typing import NDArray
from wpimath.geometry import Rotation3d

from app.camera.camera_protocol import Camera, Request, Size
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.display import Display
from app.network.real_network import Network

Mat = NDArray[np.uint8]


class NoteDetector(Interpreter):
    def __init__(
        self,
        identity: Identity,
        cam: Camera,
        camera_num: int,
        display: Display,
        network: Network,
    ) -> None:
        self.cam = cam
        self.display = display
        self.network = network

        self.mtx = self.cam.get_intrinsic()
        self.dist = self.cam.get_dist()

        size: Size = cam.get_size()
        self.width: int = size.width
        self.height: int = size.height

        # opencv hue values are 0-180, half the usual number
        self.object_lower = np.array((0, 200, 190))
        self.object_lower2 = np.array(6, 0, 0)
        self.object_higher = np.array((8, 255, 255))
        self.object_higher2 = np.array((180, 255, 255))

        # TODO: move the identity part of this path to the Network object
        path = "noteVision/" + identity.value + "/" + str(camera_num)
        self._notes = network.get_note_sender(path + "/Rotation3d")

    def analyze(self, req: Request) -> None:
        with req.rgb() as buffer:

            img = cast(Mat, np.frombuffer(buffer, dtype=np.uint8))  # type:ignore
            img_bgr = img.reshape((self.height, self.width, 3))

            # TODO: figure out the crop
            # img_bgr : Mat = img_bgr[65:583, :, :]

            img_bgr = cv2.undistort(img_bgr, self.mtx, self.dist)
            img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
            img_hsv = np.ascontiguousarray(img_hsv)

            img_range = cv2.inRange(img_hsv, self.object_lower, self.object_higher)

            floodfill = img_range.copy()
            mask = np.zeros((self.height + 2, self.width + 2), np.uint8)
            cv2.floodFill(floodfill, mask, [0, 0], [255])
            

            floodfill_inv = cv2.bitwise_not(floodfill)
            img_floodfill = cv2.bitwise_or(img_range, floodfill_inv)
            median = cv2.medianBlur(img_floodfill, 5)
            contours, _ = cv2.findContours(
                median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )

            objects: list[Rotation3d] = []
            for c in contours:
                # _, _, cnt_width, cnt_height = cv2.boundingRect(c)
                # reject anything taller than it is wide
                # if cnt_width / cnt_height < 1.2:
                #     continue
                # # reject big bounding box
                # if cnt_width > width / 2 or cnt_height > height / 2:
                #     continue

                # if (cnt_height < 20 or cnt_width < 20) and cnt_width/cnt_height < 3:
                #     continue

                mmnts = cv2.moments(c)
                # reject too small (m00 is in pixels)
                # TODO: make this adjustable at runtime
                # to pick out distant targets
                if mmnts["m00"] < 100:
                    continue

                cX = int(mmnts["m10"] / mmnts["m00"])
                cY = int(mmnts["m01"] / mmnts["m00"])

                yNormalized = (self.height / 2 - cY) / self.mtx[1, 1]
                xNormalized = (self.width / 2 - cX) / self.mtx[0, 0]

                initial = np.array([1, 0, 0], dtype=np.float64)
                final = np.array([1, xNormalized, yNormalized], dtype=np.float64)
                rotation = Rotation3d(initial=initial, final=final)

                objects.append(rotation)
                self.display.note(img_bgr, c, cX, cY)

            delay_us = req.delay_us()

            self._notes.send(objects, delay_us)
            # must flush!  otherwise 100ms update rate.
            self.network.flush()

            fps = req.fps()
            self.display.text(img_bgr, f"FPS {fps:2.0f}", (5, 65))
            self.display.text(img_bgr, f"delay (ms) {delay_us/1000:2.0f}", (5, 105))
            #self.display.put(img_range)
            self.display.put(img_bgr)