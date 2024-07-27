""" This is a wrapper for the AprilTag detector. """

from mmap import mmap
from typing import Any
from robotpy_apriltag import AprilTagDetector
from app.camera import Request
from app.timer import Timer


class TagDetector:
    def __init__(self, width: int, height: int) -> None:
        self.at_detector: AprilTagDetector = AprilTagDetector()
        self.width: int = width
        self.height: int = height

    def analyze(self, req: Request) -> None:
        metadata: dict[str, Any] = req.metadata()
        with req.buffer() as buffer:
            self.analyze2(metadata, buffer)

    def analyze2(self, metadata: dict[str, Any], buffer: mmap) -> None:
        # how old is the frame when we receive it?
        received_time: int = Timer.time_ns()
        print("hello")
