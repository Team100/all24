""" This is a wrapper for the AprilTag detector. """

# pylint: disable=E0611,R0902,R0913,R0914,W0212

from typing import Any, cast

import numpy as np
from cv2 import undistortImagePoints
from numpy.typing import NDArray
from robotpy_apriltag import AprilTagDetection, AprilTagDetector, AprilTagPoseEstimator
from typing_extensions import Buffer, override

from app.camera.camera_protocol import Camera, Request, Size
from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.dashboard.display import Display
from app.network.real_network import Blip24, Network
from app.util.timer import Timer

Mat = NDArray[np.uint8]


class TagDetector(Interpreter):
    def __init__(
        self,
        identity: Identity,
        cam: Camera,
        camera_num: int,
        display: Display,
        network: Network,
    ) -> None:
        self.identity = identity
        self.cam = cam
        self.camera_num = camera_num
        self.display = display
        self.network = network

        size: Size = cam.get_size()
        # TODO: move the identity part of this path to the Network object
        path = "vision/" + identity.value + "/" + str(camera_num)
        self._blips = network.get_blip_sender(path + "/blips")

        self.width: int = size.width
        self.height: int = size.height

        self.y_len = self.width * self.height

        self.frame_time_ns = Timer.time_ns()

        self.at_detector: AprilTagDetector = AprilTagDetector()

        config = self.at_detector.Config()
        config.numThreads = 4
        self.at_detector.setConfig(config)
        self.at_detector.addFamily("tag36h11")

        tag_size = 0.1651  # tagsize 6.5 inches

        self.mtx: Mat = cam.get_intrinsic()
        self.dist: Mat = cam.get_dist()

        self.estimator = AprilTagPoseEstimator(
            AprilTagPoseEstimator.Config(
                tag_size,
                self.mtx[0, 0],
                self.mtx[1, 1],
                self.mtx[0, 2],
                self.mtx[1, 2],
            )
        )

    @override
    def analyze(self, req: Request) -> None:
        metadata: dict[str, Any] = req.metadata()
        with req.yuv() as buffer:
            self.analyze2(metadata, buffer)

    def analyze2(self, metadata: dict[str, Any], buffer: Buffer) -> None:
        # truncate, ignore chrominance. this makes a view, very fast (300 ns)
        img: Mat = cast(
            Mat,
            np.frombuffer(buffer, dtype=np.uint8, count=self.y_len),  # type:ignore
        )

        # this  makes a view, very fast (150 ns)
        img: Mat = img.reshape((self.height, self.width))  # type:ignore

        # TODO: crop regions that never have targets
        # this also makes a view, very fast (150 ns)
        # img = img[int(self.height / 4) : int(3 * self.height / 4), : self.width]
        # for now use the full frame
        # TODO: probably remove this
        # if self.identity == Identity.SHOOTER:
        #     img = img[62:554, : self.width]
        # else:
        #     img = img[: self.height, : self.width]

        result: list[AprilTagDetection] = self.at_detector.detect(img.data)

        blips: list[Blip24] = []
        result_item: AprilTagDetection
        for result_item in result:
            if result_item.getHamming() > 0:
                continue

            # UNDISTORT EACH ITEM
            # undistortPoints is at least 10X faster than undistort on the whole image.
            corners: tuple[float, float, float, float, float, float, float, float] = (
                result_item.getCorners((0, 0, 0, 0, 0, 0, 0, 0))
            )
            # undistortPoints wants [[x0,y0],[x1,y1],...]
            pairs = np.reshape(corners, [4, 2])
            pairs = undistortImagePoints(pairs, self.mtx, self.dist)
            # the estimator wants [x0, y0, x1, y1, ...]
            # corners = np.reshape(pairs, [8])
            # pairs has an extra dimension
            corners = (
                pairs[0][0][0],
                pairs[0][0][1],
                pairs[1][0][0],
                pairs[1][0][1],
                pairs[2][0][0],
                pairs[2][0][1],
                pairs[3][0][0],
                pairs[3][0][1],
            )

            homography = result_item.getHomography()
            pose = self.estimator.estimate(homography, corners)

            blips.append(Blip24(result_item.getId(), pose))
            self.display.tag(img, result_item, pose)

        # time since last frame
        current_time_ns = Timer.time_ns()
        total_time_ms = (current_time_ns - self.frame_time_ns) / 1000000
        self.frame_time_ns = current_time_ns

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

        # oldest_pixel_ms = (system_time_ns - (
        # sensor_timestamp - 1000 * metadata["ExposureTime"])) // 1000000
        # sensor timestamp is the boottime when the first byte was received from the sensor

        # send all the data over the network
        delay_ns: int = Timer.time_ns() - sensor_midpoint_ns
        delay_us = delay_ns // 1000
        self._blips.send(blips, delay_us)
        # must flush!  otherwise 100ms update rate.
        self.network.flush()

        # do the drawing (after the NT payload is written)
        # none of this is particularly fast or important for prod.
        fps = 1000 / total_time_ms
        self.display.text(img, f"FPS {fps:2.0f}", (5, 65))
        self.display.text(img, f"delay (ms) {delay_us/1000:2.0f}", (5, 105))
        self.display.put(img)
