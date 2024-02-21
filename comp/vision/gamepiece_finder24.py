import dataclasses
import time
from enum import Enum

import cv2
import libcamera
import numpy as np

from cscore import CameraServer
from ntcore import NetworkTableInstance
from picamera2 import Picamera2
from wpimath.geometry import Rotation3d
import math

class Camera(Enum):
    """Keep this synchronized with java team100.config.Camera."""

    FRONT = "1000000013c9c96c"  # "2"
    REAR = "100000004e0a1fb9"  # "1"
    LEFT = "10000000a7c673d9"  # "4"
    RIGHT = "10000000a7a892c0"  # "3"
    UNKNOWN = None

    @classmethod
    def _missing_(cls, value):
        return Camera.UNKNOWN


class GamePieceFinder:
    def __init__(self, serial, width, height, model):
        self.serial = serial
        self.width = width
        self.height = height
        self.model = model

        # opencv hue values are 0-180, half the usual number
        self.object_lower = (4,200, 100)
        self.object_higher = (12, 255, 255)
        self.frame_time = 0
        self.theta = 0
        self.initialize_nt()

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
        self.horzFOV = 2 * math.atan(self.mtx[0,2]/self.mtx[0,0])
        self.vertFOV = 2 * math.atan(self.mtx[1,2]/self.mtx[1,1])
        self.output_stream = CameraServer.putVideo("Processed", width, height)

    def initialize_nt(self):
        """Start NetworkTables with Rio as server, set up publisher."""
        self.inst = NetworkTableInstance.getDefault()
        self.inst.startClient4("gamepiece_finder24")

        # roboRio address. windows machines can impersonate this for simulation.
        self.inst.setServer("10.1.0.2")

        topic_name = "noteVision/" + self.serial
        self.vision_fps = self.inst.getDoubleTopic(topic_name + "/fps").publish()
        self.vision_latency = self.inst.getDoubleTopic(
            topic_name + "/latency"
        ).publish()

        # work around https://github.com/robotpy/mostrobotpy/issues/60
        self.inst.getStructTopic("bugfix", Rotation3d).publish().set(
            Rotation3d(0, 0, 0)
        )

        self.vision_nt_struct = self.inst.getStructArrayTopic(
            topic_name + "/Rotation3d", Rotation3d
        ).publish(keepDuplicates=True)

    def find_object(self, img_yuv):
        # this says YUV->RGB but it actually makes BGR.
        # github.com/raspberrypi/picamera2/issues/848
        img_bgr = cv2.cvtColor(img_yuv, cv2.COLOR_YUV420p2RGB)

        img_bgr = cv2.undistort(img_bgr, self.mtx, self.dist)

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
        objects = []
        for c in contours:
            _, _, cnt_width, cnt_height = cv2.boundingRect(c)
            # reject anything taller than it is wide
            if cnt_width / cnt_height < 1.2:
                continue
            # reject big bounding box
            if cnt_width > self.width / 2 or cnt_height > self.height / 2:
                continue

            if (cnt_height < 20 or cnt_width < 20) and cnt_width/cnt_height < 3:
                continue

            mmnts = cv2.moments(c)
            # reject too small (m00 is in pixels)
            # TODO: make this adjustable at runtime
            # to pick out distant targets
            if mmnts["m00"] < 100:
                continue

            cX = int(mmnts["m10"] / mmnts["m00"])
            cY = int(mmnts["m01"] / mmnts["m00"])

            pitchRad = math.atan((cY-self.height/2)/self.mtx[1,1])
            yawRad = math.atan((self.width/2-cX)/self.mtx[0,0])
            # Puts up angle to the target from the POV of the camera
            rotation = Rotation3d(0, pitchRad, yawRad)
            objects.append(rotation)
            self.draw_result(img_bgr, c, cX, cY)
            
        self.output_stream.putFrame(img_range)
        return objects

    def draw_result(self, img, cnt, cX, cY):
        # float_formatter = {"float_kind": lambda x: f"{x:4.1f}"}
        cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(img, (int(cX), int(cY)), 7, (0, 0, 0), -1)
        # cv2.putText(img, f"t: {np.array2string(wpi_t.flatten(), formatter=float_formatter)}", (int(cX) - 20, int(cY) - 20),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    def analyze(self, request):
        img_yuv = request.make_array("lores")
        metadata = request.get_metadata()

        objects = self.find_object(img_yuv)

        current_time = time.time()
        total_et = current_time - self.frame_time
        self.frame_time = current_time

        fps = 1 / total_et

        self.vision_nt_struct.set(objects)
        self.vision_fps.set(fps)

        sensor_timestamp = metadata["SensorTimestamp"]
        # include all the work above in the latency
        system_time_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        time_delta_ms = (system_time_ns - sensor_timestamp) // 1000000
        self.vision_latency.set(time_delta_ms)
        self.inst.flush()


def getserial():
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""


def main():
    print("main")

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
        # one buffer to write, one to read, one in between so we don't have to wait
        buffer_count=2,
        main={
            "format": "YUV420",
            "size": (fullwidth, fullheight),
        },
        lores={"format": "YUV420", "size": (width, height)},
        controls={
            # no duration limit => sacrifice speed for color
            # "FrameDurationLimits": (33333, 33333),  # 41 fps
            # noise reduction takes time
            "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off,
            "AwbEnable": False,
            "AeEnable": False,
            "ExposureTime": 30000,
            # "AnalogueGain": 1.0
        },
    )

    serial = getserial()
    identity = Camera(serial)
    if identity == Camera.REAR or identity == Camera.FRONT:
        camera_config["transform"] = libcamera.Transform(hflip=1, vflip=1)

    print("\nREQUESTED CONFIG")
    print(camera_config)
    camera.align_configuration(camera_config)
    print("\nALIGNED CONFIG")
    print(camera_config)
    camera.configure(camera_config)
    print("\nCONTROLS")
    print(camera.camera_controls)
    print(serial)
    output = GamePieceFinder(serial, width, height, model)

    camera.start()
    try:
        while True:
            request = camera.capture_request()
            try:
                output.analyze(request)
            finally:
                request.release()
    finally:
        camera.stop()


main()
