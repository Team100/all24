""" This is a wrapper for the note detector.
"""

# pylint: disable=E1101

from mmap import mmap
from typing import Any
import numpy as np
import cv2
from wpimath.geometry import Rotation3d
from app.camera.camera_protocol import Camera, Request
from app.dashboard.display import Display
from app.util.timer import Timer


class NoteDetector:
    def __init__(self, cam: Camera, display: Display) -> None:
        self.camera = cam
        self.display: Display = display
        self.frame_time = Timer.time_ns()

    def find_object(self, img_yuv: mmap) -> None:
        # this says YUV->RGB but it actually makes BGR.
        # github.com/raspberrypi/picamera2/issues/848
        img_bgr = cv2.cvtColor(img_yuv, cv2.COLOR_YUV420p2RGB)
        serial = getserial()
        identity = Camera(serial)
        if identity == Camera.GAME_PIECE:
            img_bgr = img_bgr[65:583, :, :]

        img_bgr = cv2.undistort(img_bgr, camera.mtx, camera.dist)
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)

        img_range = cv2.inRange(img_hsv, self.object_lower, self.object_higher)

        floodfill = img_range.copy()
        h, w = img_range.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(floodfill, mask, (0, 0), 255)
        floodfill_inv = cv2.bitwise_not(floodfill)
        img_floodfill = img_range | floodfill_inv
        median = cv2.medianBlur(img_floodfill, 5)
        contours, hierarchy = cv2.findContours(
            median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        width, height = camera.width, camera.height
        objects = []
        for c in contours:
            _, _, cnt_width, cnt_height = cv2.boundingRect(c)
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

            pitchRad = math.atan((cY - height / 2) / camera.mtx[1, 1])
            yawRad = math.atan((width / 2 - cX) / camera.mtx[0, 0])
            # Puts up angle to the target from the POV of the camera
            rotation = Rotation3d(0, pitchRad, yawRad)
            self.objects.append(rotation)
            self.draw_result(img_bgr, c, cX, cY)
        img_output = cv2.resize(img_bgr, (269, 162))
        self.display.put_frame(img_range)

    def analyze(self, req: Request) -> None:
        metadata: dict[str, Any] = req.metadata()
        with req.buffer() as buffer:
            self.analyze2(metadata, buffer)

    def analyze2(self, metadata: dict[str, Any], buffer: mmap) -> None:

        self.find_object(buffer, camera)

        current_time = time.time()
        total_et = current_time - camera.frame_time
        camera.frame_time = current_time

        fps = 1 / total_et

        camera.fps = fps
        camera.FPSPublisher.set(fps)

        sensor_timestamp = metadata["SensorTimestamp"]
        # include all the work above in the latency
        system_time_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        time_delta_ms = (system_time_ns - sensor_timestamp) // 1000000
        camera.LatencyPublisher.set(time_delta_ms)
        self.inst.flush()
