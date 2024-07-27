""" This is a wrapper for the AprilTag detector.
"""

import time

import robotpy_apriltag

## todo: remove this import.
from picamera2.request import _MappedBuffer  # type: ignore

from app.camera import Request


class TagDetector:
    def __init__(self):
        self.at_detector = robotpy_apriltag.AprilTagDetector()

    def analyze(self, req: Request):
        # how old is the frame when we receive it?
        received_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        metadata = req.req.get_metadata()
        with _MappedBuffer(req.req, "lores") as buffer:
            self.analyze2(metadata, buffer)

    def analyze2(self, metadata, buffer):
        pass
