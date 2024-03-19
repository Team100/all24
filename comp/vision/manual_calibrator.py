"""
MANUAL CALIBRATOR

This simply finds tags in the viewport and displays their centers in pixels and meters.

It uses 20% scaled tags 33.02 mm
"""
# pylint: disable=missing-module-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=import-error
import dataclasses
import time

from enum import Enum

import cv2
import libcamera
import numpy as np
import ntcore

import robotpy_apriltag

from cscore import CameraServer
from picamera2 import Picamera2
from wpimath.geometry import Transform3d
from wpiutil import wpistruct


@wpistruct.make_wpistruct
@dataclasses.dataclass
class Blip24:
    id: int
    pose: Transform3d


class Camera(Enum):
    """Keep this synchronized with java team100.config.Camera."""

    A = "10000000caeaae82"  # "BETA FRONT"
    B = "1000000013c9c96c"  # "BETA BACK"
    C = "10000000a7c673d9"  # "GAMMA INTAKE"

    SHOOTER = "100000004e0a1fb9"  # "DELTA SHOOTER"
    AMP = "1000000031b9d05b"  # "DELTA AMP-PLACER"
    GAME_PIECE = "10000000a7c673dc"  # "DELTA INTAKE"

    G = "10000000a7a892c0"  # ""
    UNKNOWN = None

    @classmethod
    def _missing_(cls, value):
        return Camera.UNKNOWN


class TagFinder:
    def __init__(self, serial, width, height, model):
        self.frame_time = time.time()
        # the cpu serial number
        self.serial = serial
        self.width = width
        self.height = height
        self.model = model

        # for the driver view
        scale = 0.25
        self.view_width = int(width * scale)
        self.view_height = int(height * scale)

        self.initialize_nt()

        self.at_detector = robotpy_apriltag.AprilTagDetector()
        self.at_detector.addFamily("tag36h11")

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
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
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
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
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
            print("UNKNOWN CAMERA")
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
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                        0.00000000e00,
                    ]
                ]
            )
            self.estimator = robotpy_apriltag.AprilTagPoseEstimator(
                robotpy_apriltag.AprilTagPoseEstimator.Config(
                    0.1651,  # tagsize 6.5 inches
                    666,  # fx
                    666,  # fy
                    width / 2,  # cx
                    height / 2,  # cy
                )
            )

        self.output_stream = CameraServer.putVideo("Processed", width, height)

    def analyze(self, request):
        buffer = request.make_buffer("lores")
        metadata = request.get_metadata()

        y_len = self.width * self.height

        # truncate, ignore chrominance. this makes a view, very fast (300 ns)
        img = np.frombuffer(buffer, dtype=np.uint8, count=y_len)

        # this  makes a view, very fast (150 ns)
        img = img.reshape((self.height, self.width))

        # TODO: crop regions that never have targets
        # this also makes a view, very fast (150 ns)
        # img = img[int(self.height / 4) : int(3 * self.height / 4), : self.width]
        # for now use the full frame
        # TODO: probably remove this
        serial = getserial()
        identity = Camera(serial)
        if identity == Camera.SHOOTER:
            img = img[62:554, : self.width]
        else:        
            img = img[: self.height, : self.width]

        # for calibration we don't undistort
        # img = cv2.undistort(img, self.mtx, self.dist)

        result = self.at_detector.detect(img)

        blips = []
        for result_item in result:
            if result_item.getHamming() > 0:
                continue
            pose = self.estimator.estimate(result_item)
            blips.append(Blip24(result_item.getId(), pose))
            # TODO: turn this off for prod
            self.draw_result(img, result_item, pose)

        # compute time since last frame
        current_time = time.time()
        total_et = current_time - self.frame_time
        self.frame_time = current_time

        fps = 1 / total_et

        self.vision_nt_struct.set(blips)
        self.vision_fps.set(fps)

        # sensor timestamp is the boottime when the first byte was received from the sensor
        sensor_timestamp = metadata["SensorTimestamp"]
        # include all the work above in the latency
        system_time_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        time_delta_ms = (system_time_ns - sensor_timestamp) // 1000000
        self.vision_latency.set(time_delta_ms)

        # must flush!  otherwise 100ms update rate.
        self.inst.flush()

        # now do the drawing (after the NT payload is written)
        # none of this is particularly fast or important for prod,
        # TODO: consider disabling it after dev is done
        self.draw_text(img, f"fps {fps:.1f}", (5, 65))
        # self.draw_text(img, f"latency(ms) {time_delta_ms:.0f}", (5, 105))

        # shrink the driver view to avoid overloading the radio
        # TODO: turn this back on for prod!!
        # driver_img = cv2.resize(img, (self.view_width, self.view_height))
        # self.output_stream.putFrame(driver_img)

        # for now put big images
        # TODO: turn this off for prod!!
        img_output = cv2.resize(img, (416,308)) 
        self.output_stream.putFrame(img_output)

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
    def draw_text(self, image, msg, loc):
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 6)
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)

    def initialize_nt(self):
        """Start NetworkTables with Rio as server, set up publisher."""
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("tag_finder24")

        # roboRio address. windows machines can impersonate this for simulation.
        self.inst.setServer("10.1.0.2")

        topic_name = "vision/" + self.serial
        self.vision_fps = self.inst.getDoubleTopic(topic_name + "/fps").publish()
        self.vision_latency = self.inst.getDoubleTopic(
            topic_name + "/latency"
        ).publish()

        # work around https://github.com/robotpy/mostrobotpy/issues/60
        self.inst.getStructTopic("bugfix", Blip24).publish().set(
            Blip24(0, Transform3d())
        )

        # blip array topic
        self.vision_nt_struct = self.inst.getStructArrayTopic(
            topic_name + "/blips", Blip24
        ).publish()


def getserial():
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""


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
        print("UNKNOWN CAMERA: " + model)
        fullwidth = 100
        fullheight = 100
        width = 100
        height = 100

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

    serial = getserial()
    identity = Camera(serial)
    # if identity == Camera.FRONT:
    #     camera_config["transform"] = libcamera.Transform(hflip=1, vflip=1)

    print("\nREQUESTED CONFIG")
    print(camera_config)
    camera.align_configuration(camera_config)
    print("\nALIGNED CONFIG")
    print(camera_config)
    camera.configure(camera_config)
    print("\nCONTROLS")
    print(camera.camera_controls)
    print(serial)
    output = TagFinder(serial, width, height, model)

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
