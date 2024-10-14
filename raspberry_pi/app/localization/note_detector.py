""" This is a wrapper for the note detector."""

# pylint: disable=C0103,E1101,R0902,R0913,R0914,W0612

from mmap import mmap
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray
from wpimath.geometry import Rotation3d

from app.camera.camera_protocol import Camera, Request
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.display import Display
from app.network.real_network import Network
from app.util.timer import Timer

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

        self.frame_time = Timer.time_ns()

        # opencv hue values are 0-180, half the usual number
        self.object_lower = np.array((0, 150, 50))
        self.object_lower2 = np.array((178, 150, 50))
        self.object_higher = np.array((8, 255, 255))
        self.object_higher2 = np.array((180, 255, 255))

        # TODO: move the identity part of this path to the Network object
        path = "noteVision/" + identity.value + "/" + str(camera_num)
        self._notes = network.get_note_sender(path + "/Rotation3d")

    def analyze(self, req: Request) -> None:
        metadata: dict[str, Any] = req.metadata()
        with req.rgb() as buffer:
            self.analyze2(metadata, buffer)

    def analyze2(self, metadata: dict[str, Any], buffer: mmap) -> None:
        # Wants a buffer in BGR format.  Remember that when OpenCV says
        # "RGB" it really means "BGR"
        # github.com/raspberrypi/picamera2/issues/848


        size = self.cam.get_size()
        width = size.width
        height = size.height

        img: Mat = np.frombuffer(buffer, dtype=np.uint8)
        img_bgr = img.reshape((height, width, 3))

        # TODO: figure out the crop
        # img_bgr : Mat = img_bgr[65:583, :, :]


        img_bgr = cv2.undistort(img_bgr, self.mtx, self.dist)
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)

        img_range = cv2.inRange(img_hsv, self.object_lower, self.object_higher)

        floodfill = img_range.copy()
        h, w = img_range.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(floodfill, mask, [0, 0], [255])
        floodfill_inv = cv2.bitwise_not(floodfill)
        img_floodfill = cv2.bitwise_or(img_range, floodfill_inv)
        median = cv2.medianBlur(img_floodfill, 5)
        contours, _ = cv2.findContours(median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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

            yNormalized = (height / 2 - cY) / self.mtx[1, 1]
            xNormalized = (width / 2 - cX) / self.mtx[0, 0]

            rotation = Rotation3d(
                initial=np.array([1, 0, 0], dtype=np.float64),
                final=np.array([1, xNormalized, yNormalized], dtype=np.float64),
            )

            objects.append(rotation)
            self.display.note(img_bgr, c, cX, cY)


        # compute time since last frame
        current_time = Timer.time_ns()
        total_time_ms = (current_time - self.frame_time) / 1000000
        # total_et = current_time - self.frame_time
        self.frame_time = current_time

        # TODO: move this timing logic to the camera
        # time of first row received
        sensor_timestamp_ns = metadata["SensorTimestamp"]

        # For a global shutter, the whole frame is exposed a little before
        # the SensorTimestamp.
        sensor_midpoint_ns = sensor_timestamp_ns

        if self.cam.is_rolling_shutter():
            # For a rolling shutter, rows are exposed over the entire
            # frame duration (1/fps).
            # TODO: assign a different timestamp to each tag, depending on
            # where it is in the frame.
            frame_duration_ns = metadata["FrameDuration"] * 1000
            sensor_midpoint_ns = sensor_timestamp_ns + frame_duration_ns / 2

        delay_ns: int = Timer.time_ns() - sensor_midpoint_ns
        delay_us = delay_ns // 1000
        self._notes.send(objects, delay_us)
        # must flush!  otherwise 100ms update rate.
        self.network.flush()

        fps = 1000 / total_time_ms
        self.display.text(img_bgr, f"FPS {fps:2.0f}", (5, 65))
        self.display.text(img_bgr, f"delay (ms) {delay_us/1000:2.0f}", (5, 105))

        # img_output = cv2.resize(img_bgr, (269, 162))

        # self.display.put_frame(img_range)
        self.display.put(img_bgr)
