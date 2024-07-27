""" This is a wrapper for the AprilTag detector. """

from mmap import mmap
from typing import Any

import numpy as np
from numpy.typing import NDArray

from robotpy_apriltag import AprilTagDetector, AprilTagPoseEstimator
from app.camera import Camera, Request
from app.identity import Identity
from app.timer import Timer

Mat = NDArray[np.uint8]


class TagDetector:
    def __init__(
        self, identity: Identity, width: int, height: int, cam: Camera
    ) -> None:
        self.identity: Identity = identity
        self.width: int = width
        self.height: int = height

        self.frame_time = Timer.time_ns()

        self.at_detector: AprilTagDetector = AprilTagDetector()

        config = self.at_detector.Config()
        config.numThreads = 4
        self.at_detector.setConfig(config)
        self.at_detector.addFamily("tag36h11")

        tag_size = 0.1651  # tagsize 6.5 inches

        mtx: Mat = cam.get_intrinsic()

        self.estimator = AprilTagPoseEstimator(
            AprilTagPoseEstimator.Config(
                tag_size,
                mtx[0,0],
                mtx[1,1],
                mtx[0,2],
                mtx[1,2],
            )
        )

    def analyze(self, req: Request) -> None:
        metadata: dict[str, Any] = req.metadata()
        with req.buffer() as buffer:
            self.analyze2(metadata, buffer)

    def analyze2(self, metadata: dict[str, Any], buffer: mmap) -> None:
        # how old is the frame when we receive it?
        received_time: int = Timer.time_ns()
        print("hello")
