# pylint: disable=C0103,C0114,C0115,C0116,E1101,R0902,R0914

import sys
import time
from enum import Enum
from typing import Any, cast

import cv2
import ntcore as nt
import numpy as np
from cscore import CameraServer
from cv2.typing import MatLike
from ntcore import NetworkTableInstance
from numpy.typing import NDArray
from picamera2 import CompletedRequest  # type: ignore
from picamera2 import Picamera2  # type: ignore
from wpimath.geometry import Rotation3d

Mat = NDArray[np.uint8]


class Camera(Enum):
    """Keep this synchronized with java team100.config.Camera."""

    C = "10000000a7c673d9"  # "GAMMA INTAKE"
    SHOOTER = "10000000a7a892c0"  # "DELTA SHOOTER"
    RIGHTAMP = "10000000caeaae82"  # "DELTA AMP-PLACER"
    LEFTAMP = "100000004e0a1fb9"  # "DELTA AMP-PLACER"
    GAME_PIECE = "1000000013c9c96c"  # "DELTA INTAKE"
    G = "10000000a7a892c0"  # ""
    UNKNOWN = None

    @classmethod
    def _missing_(cls, value: object) -> "Camera":
        return Camera.UNKNOWN


class GamePieceFinder:
    def __init__(self, serial: str, width: int, height: int, model: str) -> None:
        self.serial = serial
        self.width = width
        self.height = height
        self.model = model

        # opencv hue values are 0-180, half the usual number
        # for light
        lower_value_bound = 190
        lower_saturation_bound = 10
        self.object_lower = np.array((0, lower_saturation_bound, lower_value_bound))
        self.object_higher = np.array((15, 255, 255))
        self.secobject_lower = np.array(
            (165, lower_saturation_bound, lower_value_bound)
        )
        self.secobject_higher = np.array((180, 255, 255))
        # for darkness
        # self.object_lower = np.array((4,150, 100))
        # self.object_higher = np.array((16, 255, 255))
        self.frame_time = 0
        self.theta = 0
        self.initialize_nt()

        # from testing on 3/22/24, k1 and k2 only

        if self.model == "imx708_wide":
            print("V3 WIDE CAMERA")
            fx = 498
            fy = 498
            cx = 584
            cy = 316
            k1 = 0.01
            k2 = -0.0365
        elif self.model == "imx219":
            print("V2 CAMERA (NOT WIDE ANGLE)")
            fx = 660
            fy = 660
            cx = 426
            cy = 303
            k1 = -0.003
            k2 = 0.04
        # TODO get these real distortion values
        elif model == "imx296":
            fx = 1680
            fy = 1680
            cx = 728
            cy = 544
            k1 = 0
            k2 = 0
        else:
            print("UNKNOWN CAMERA MODEL")
            sys.exit()

        p1 = 0
        p2 = 0

        self.mtx = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.dist = np.array([[k1, k2, p1, p2]])

        # self.horzFOV = 2 * math.atan(self.mtx[0,2]/self.mtx[0,0])
        # self.vertFOV = 2 * math.atan(self.mtx[1,2]/self.mtx[1,1])

        self.output_stream = CameraServer.putVideo("Processed", width, height)

    def initialize_nt(self) -> None:
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
        ).publish(nt.PubSubOptions(keepDuplicates=True))

    def find_object(self, img_yuv: Mat) -> list[Rotation3d]:
        # this says YUV->RGB but it actually makes BGR.
        # github.com/raspberrypi/picamera2/issues/848
        img_bgr = cv2.cvtColor(img_yuv, cv2.COLOR_YUV420p2RGB)
        serial: str = getserial()
        identity = Camera(serial)
        if identity == Camera.GAME_PIECE:
            img_bgr: MatLike = img_bgr[65:583, :, :]

        img_bgr = cv2.undistort(img_bgr, self.mtx, self.dist)
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        img_hsv = np.ascontiguousarray(img_hsv)

        img1 = cv2.inRange(img_hsv, self.object_lower, self.object_higher)
        img2 = cv2.inRange(img_hsv, self.secobject_lower, self.secobject_higher)
        img_range = cv2.bitwise_or(img1, img2)

        floodfill = img_range.copy()
        h, w = img_range.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        cv2.floodFill(floodfill, mask, [0, 0], [255])
        floodfill_inv = cv2.bitwise_not(floodfill)
        img_floodfill = cv2.bitwise_or(img_range, floodfill_inv)
        median = cv2.medianBlur(img_floodfill, 5)
        contours, _ = cv2.findContours(
            median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        objects: list[Rotation3d] = []
        for c in contours:
            _, _, cnt_width, cnt_height = cv2.boundingRect(c)
            # reject anything taller than it is wide
            if cnt_width / cnt_height < 1.2:
                continue
            # reject big bounding box
            if cnt_width > self.width / 2 or cnt_height > self.height / 2:
                continue

            if (cnt_height < 20 or cnt_width < 20) and cnt_width / cnt_height < 3:
                continue

            mmnts = cv2.moments(c)
            # reject too small (m00 is in pixels)
            # TODO: make this adjustable at runtime
            # to pick out distant targets
            if mmnts["m00"] < 100:
                continue

            cX = int(mmnts["m10"] / mmnts["m00"])
            cY = int(mmnts["m01"] / mmnts["m00"])

            yNormalized: float = (cY - self.height / 2) / self.mtx[1, 1]
            zNormalized: float = (self.width / 2 - cX) / self.mtx[0, 0]
            # pitchRad = math.atan(yNormalized)
            # yawRad = math.atan(zNormalized)
            # Puts up angle to the target from the POV of the camera
            # these are not extrinsic euler angles; this is wrong.
            # rotation = Rotation3d(0, pitchRad, yawRad)
            # the correct rotation is one that matches the normalized (x,y) coordinates
            rotation = Rotation3d(
                initial=np.array([1, 0, 0]),
                final=np.array([1, yNormalized, zNormalized]),
            )
            objects.append(rotation)
            self.draw_result(img_bgr, c, cX, cY)
        img_output = cv2.resize(img_bgr, (269, 162))
        self.output_stream.putFrame(img_output)  # type: ignore
        return objects

    def draw_result(self, img: MatLike, cnt: MatLike, cX: int, cY: int) -> None:
        # float_formatter: dict[str, Callable[[float], str]] = {"float_kind": lambda x: f"{x:4.1f}"}
        cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(img, (cX, cY), 7, (0, 0, 0), -1)
        # cv2.putText(img, f"t: {np.array2string(wpi_t.flatten(), formatter=float_formatter)}", (cX - 20, cY - 20),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    def analyze(self, request: CompletedRequest) -> None:
        img_yuv: Mat = cast(Mat, request.make_array("lores"))  # type: ignore
        metadata: dict[str, Any] = request.get_metadata()

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


def getserial() -> str:
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""


def main() -> None:
    print("main")

    camera = Picamera2()

    model: str = cast(str, camera.camera_properties["Model"])  # type: ignore
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
    elif model == "imx296":
        print("GS Camera")
        # full frame, 2x2, to set the detector mode to widest angle possible
        fullwidth = 1408  # slightly larger than the detector, to match stride
        fullheight = 1088
        # medium detection resolution, compromise speed vs range
        width = 1408
        height = 1088
    else:
        print("UNKNOWN CAMERA: " + model)
        fullwidth = 100
        fullheight = 100
        width = 100
        height = 100

    camera_config: dict[str, Any] = camera.create_still_configuration(  # type:ignore
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
            "NoiseReductionMode": 0,  # libcamera.controls.draft.NoiseReductionModeEnum.Off,
            "AwbEnable": False,
            "AeEnable": False,
            "ExposureTime": 40000,
            # "AnalogueGain": 1.0
        },
    )

    serial: str = getserial()
    # identity = Camera(serial)

    print("\nREQUESTED CONFIG")
    print(camera_config)  # type:ignore
    camera.align_configuration(camera_config)  # type:ignore
    print("\nALIGNED CONFIG")
    print(camera_config)  # type:ignore
    camera.configure(camera_config)  # type:ignore
    print("\nCONTROLS")
    print(camera.camera_controls)  # type:ignore
    print(serial)
    output = GamePieceFinder(serial, width, height, model)

    camera.start()  # type:ignore
    try:
        while True:
            request: CompletedRequest = camera.capture_request()  # type:ignore
            try:
                output.analyze(request)
            finally:
                request.release()
    finally:
        camera.stop()


main()
