"""
MANUAL CALIBRATOR

This simply finds tags in the viewport and displays their centers in pixels and meters.

It uses 20% scaled tags 33.02 mm
"""
# pylint: disable=missing-module-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=import-error


import cv2
import sys
import libcamera
import numpy as np

import robotpy_apriltag

from cscore import CameraServer
from picamera2 import Picamera2
from wpimath.geometry import Transform3d


class TagFinder:
    def __init__(self, width, height, model):
        self.width = width
        self.height = height
        self.model = model

        self.at_detector = robotpy_apriltag.AprilTagDetector()
        self.at_detector.addFamily("tag36h11")

        # i think we should just focus on k1 and k2, and assume zero tangential, and ignore the higher order k factors.

        if self.model == "imx708_wide":
            print("V3 WIDE CAMERA")
            self.mtx = np.array([[497, 0, 578], [0, 498, 328], [0, 0, 1]])
            self.dist = np.array(
                [
                    [
                        -1.18341279e00,
                        7.13453990e-01,
                        7.90204163e-04,
                        -7.38879856e-04,
                        -2.94529084e-03,
                        -1.14073111e00,
                        6.16356154e-01,
                        5.86094708e-02,
                    ]
                ]
            )
            self.estimator = robotpy_apriltag.AprilTagPoseEstimator(
                robotpy_apriltag.AprilTagPoseEstimator.Config(
                    0.1651,  # tagsize 6.5 inches
                    497,  # fx
                    498,  # fy
                    width / 2,  # cx
                    height / 2,  # cy
                )
            )
        elif self.model == "imx219":
            print("V2 CAMERA (NOT WIDE ANGLE)")
            self.mtx = np.array([[658, 0, 422], [0, 660, 318], [0, 0, 1]])
            self.dist = np.array(
                [
                    [
                        2.26767723e-02,
                        3.92792657e01,
                        5.34833047e-04,
                        -1.76949201e-03,
                        -6.59779907e01,
                        -5.75883422e-02,
                        3.81831051e01,
                        -6.37029103e01,
                    ]
                ]
            )
            self.estimator = robotpy_apriltag.AprilTagPoseEstimator(
                robotpy_apriltag.AprilTagPoseEstimator.Config(
                    0.1651,  # tagsize 6.5 inches
                    658,  # fx
                    660,  # fy
                    width / 2,  # cx
                    height / 2,  # cy
                )
            )
        else:
            print("UNKNOWN CAMERA MODEL")
            sys.exit()

        self.output_stream = CameraServer.putVideo("Processed", width, height)

    def analyze(self, request):
        buffer = request.make_buffer("lores")

        y_len = self.width * self.height

        # truncate, ignore chrominance. this makes a view, very fast (300 ns)
        img = np.frombuffer(buffer, dtype=np.uint8, count=y_len)

        # this  makes a view, very fast (150 ns)
        img = img.reshape((self.height, self.width))

        # for calibration we don't undistort
        # turn this on to check the calibration!
        # img = cv2.undistort(img, self.mtx, self.dist)

        result = self.at_detector.detect(img)

        for result_item in result:
            if result_item.getHamming() > 0:
                continue
            pose = self.estimator.estimate(result_item)
            self.draw_result(img, result_item, pose)

        self.output_stream.putFrame(img)

    def draw_result(self, image, result_item, pose: Transform3d):
        color = (255, 255, 255)

        # Draw lines around the tag
        for i in range(4):
            j = (i + 1) % 4
            point1 = (int(result_item.getCorner(i).x), int(result_item.getCorner(i).y))
            point2 = (int(result_item.getCorner(j).x), int(result_item.getCorner(j).y))
            cv2.line(image, point1, point2, color, 2)

        (c_x, c_y) = (int(result_item.getCenter().x), int(result_item.getCenter().y))
        cv2.circle(image, (c_x, c_y), 10, (255, 255, 255), -1)
        self.draw_text(image, f"cx {c_x} cy {c_y}", (c_x, c_y))
        if pose is not None:
            t = pose.translation()
            self.draw_text(
                image,
                f"t: {t.x:4.1f},{t.y:4.1f},{t.z:4.1f}",
                (c_x - 50, c_y + 40),
            )

    # these are white with black outline
    def draw_text(self, image, msg, loc):
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 6)
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)


def main():
    camera = Picamera2()

    model = camera.camera_properties["Model"]
    print("\nMODEL " + model)

    if model == "imx708_wide":
        print("V3 Wide Camera")
        # full frame is 4608x2592; this is 2x2
        fullwidth = 2304
        fullheight = 1296
        # medium detection resolution, compromise speed vs range
        width = 1152
        height = 648
    elif model == "imx219":
        print("V2 Camera")
        # full frame, 2x2, to set the detector mode to widest angle possible
        fullwidth = 1664  # slightly larger than the detector, to match stride
        fullheight = 1232
        # medium detection resolution, compromise speed vs range
        width = 832
        height = 616
    else:
        print("UNKNOWN CAMERA MODEL: " + model)
        sys.exit()

    camera_config = camera.create_still_configuration(
        # 2 buffers => low latency (32-48 ms), low fps (15-20)
        # 5 buffers => mid latency (40-55 ms), high fps (22-28)
        # 3 buffers => high latency (50-70 ms), mid fps (20-23)
        # robot goes at 50 fps, so roughly a frame every other loop
        # fps doesn't matter much, so minimize latency
        buffer_count=2,
        main={
            "format": "YUV420",
            "size": (fullwidth, fullheight),
        },
        lores={"format": "YUV420", "size": (width, height)},
        controls={
            # these manual controls are useful sometimes but turn them off for now
            # because auto mode seems fine
            # fast shutter means more gain
            # "AnalogueGain": 8.0,
            # try faster shutter to reduce blur.  with 3ms, 3 rad/s seems ok.
            "ExposureTime": 3000,
            # limit auto: go as fast as possible but no slower than 30fps
            # without a duration limit, we slow down in the dark, which is fine
            # "FrameDurationLimits": (5000, 33333),  # 41 fps
            # noise reduction takes time, don't need it.
            "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off,
        },
    )

    print("\nREQUESTED CONFIG")
    print(camera_config)
    camera.align_configuration(camera_config)
    print("\nALIGNED CONFIG")
    print(camera_config)
    camera.configure(camera_config)
    print("\nCONTROLS")
    print(camera.camera_controls)
    output = TagFinder(width, height, model)

    camera.start()
    try:
        while True:
            # the most recent completed frame, from the recent past
            request = camera.capture_request()
            try:
                output.analyze(request)
            finally:
                # the frame is owned by the camera so remember to release it
                request.release()
    finally:
        camera.stop()


main()
