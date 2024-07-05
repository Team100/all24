import dataclasses
import time
from enum import Enum

import cv2
import libcamera
import numpy as np
import sys

import cscore
from cscore import CameraServer
from ntcore import NetworkTableInstance
import ntcore as nt

from picamera2 import Picamera2
from wpimath.geometry import Rotation3d
import math

class Camera(Enum):
    """Keep this synchronized with java team100.config.Camera."""
    # TODO get correct serial numbers for Delta
    # A = "10000000caeaae82"  # "BETA FRONT"
    # B = "1000000013c9c96c"  # "BETA BACK"
    C = "10000000a7c673d9"  # "GAMMA INTAKE"
    SHOOTER = "10000000a7a892c0"  # "DELTA SHOOTER"
    RIGHTAMP = "10000000caeaae82"  # "DELTA AMP-PLACER"
    LEFTAMP = "100000004e0a1fb9"  # "DELTA AMP-PLACER"
    GAME_PIECE = "1000000013c9c96c"  # "DELTA INTAKE"
    G = "10000000a7a892c0"  # ""
    UNKNOWN = None

    @classmethod
    def _missing_(cls, value):
        return Camera.UNKNOWN

class CameraData:
    def __init__(self, id):
        self.camera = Picamera2(id)
        model = self.camera.camera_properties["Model"]
        print("\nMODEL " + model)
        self.id = id
        if model == "imx708_wide":
            print("V3 Wide Camera")
            # full frame is 4608x2592; this is 2x2
            fullwidth = 2304
            fullheight = 1296
            # medium detection resolution, compromise speed vs range
            self.width = 1152
            self.height = 648
        elif model == "imx219":
            print("V2 Camera")
            # full frame, 2x2, to set the detector mode to widest angle possible
            fullwidth = 1664  # slightly larger than the detector, to match stride
            fullheight = 1232
            # medium detection resolution, compromise speed vs range
            self.width = 832
            self.height = 616
        elif model == "imx296":
            print("GS Camera")
            # full frame, 2x2, to set the detector mode to widest angle possible
            fullwidth = 1408   # slightly larger than the detector, to match stride
            fullheight = 1088
            # medium detection resolution, compromise speed vs range
            self.width = 1408
            self.height = 1088
        else:
            print("UNKNOWN CAMERA: " + model)
            fullwidth = 100
            fullheight = 100
            self.width = 100
            self.height = 100

        camera_config = self.camera.create_still_configuration(
            # one buffer to write, one to read, one in between so we don't have to wait
            buffer_count=2,
            main={
                "format": "YUV420",
                "size": (fullwidth, fullheight),
            },
            lores={"format": "YUV420", "size": (self.width, self.height)},
            controls={
                # no duration limit => sacrifice speed for color
                # "FrameDurationLimits": (33333, 33333),  # 41 fps
                # noise reduction takes time
                "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off,
                # "AwbEnable": False,
                # "AeEnable": False,
                "ExposureTime": 40000,
                # "AnalogueGain": 1.0
            },
        )

        print("\nREQUESTED CONFIG")
        print(camera_config)
        self.camera.align_configuration(camera_config)
        print("\nALIGNED CONFIG")
        print(camera_config)
        self.camera.configure(camera_config)
        print("\nCONTROLS")
        print(self.camera.camera_controls)

        if model == "imx708_wide":
            print("V3 WIDE CAMERA")
            fx = 498
            fy = 498
            cx = 584
            cy = 316
            k1 = 0.01
            k2 = -0.0365
        elif model == "imx219":
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
        self.output_stream = CameraServer.putVideo(str(id), self.width, self.height)
        self.fps = 0
        self.camera.start()
        self.frame_time = time.time()
    
    def setFPSPublisher(self, FPSPublisher):
        self.FPSPublisher = FPSPublisher

    def setLatencyPublisher(self, LatencyPublisher):
        self.LatencyPublisher = LatencyPublisher

class GamePieceFinder:
    def __init__(self, serial, camList):
        self.serial = serial
        self.objects = []
        # opencv hue values are 0-180, half the usual number
        self.object_lower = (4,150, 100)
        self.object_higher = (16, 255, 255)
        self.frame_time = 0
        self.theta = 0
        self.initialize_nt(camList)

    def initialize_nt(self, camList):
        """Start NetworkTables with Rio as server, set up publisher."""
        self.inst = NetworkTableInstance.getDefault()
        self.inst.startClient4("gamepiece_finder24")

        # roboRio address. windows machines can impersonate this for simulation.
        self.inst.setServer("10.1.0.2")

        topic_name = "noteVision/" + self.serial
        for camera in camList:
            camera.setFPSPublisher(
                self.inst.getDoubleTopic(topic_name + "/" + str(camera.id) + "/fps").publish()
            )
            camera.setLatencyPublisher(
                self.inst.getDoubleTopic(topic_name + "/" + str(camera.id) + "/latency").publish()
            )
        # work around https://github.com/robotpy/mostrobotpy/issues/60
        self.inst.getStructTopic("bugfix", Rotation3d).publish().set(
            Rotation3d(0,0,0)
        )

        self.vision_nt_struct = self.inst.getStructArrayTopic(
            topic_name + "/Rotation3d", Rotation3d
        ).publish(nt.PubSubOptions(keepDuplicates=True))

    def find_object(self, img_yuv, camera):
        # this says YUV->RGB but it actually makes BGR.
        # github.com/raspberrypi/picamera2/issues/848
        img_bgr = cv2.cvtColor(img_yuv, cv2.COLOR_YUV420p2RGB)
        serial = getserial()
        identity = Camera(serial)
        if identity == Camera.GAME_PIECE:
            img_bgr = img_bgr[65:583,:,:]

        img_bgr = cv2.undistort(img_bgr, camera.mtx,camera.dist)
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
            if cnt_width / cnt_height < 1.2:
                continue
            # reject big bounding box
            if cnt_width > width / 2 or cnt_height > height / 2:
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

            pitchRad = math.atan((cY-height/2)/camera.mtx[1,1])
            yawRad = math.atan((width/2-cX)/camera.mtx[0,0])
            # Puts up angle to the target from the POV of the camera
            rotation = Rotation3d(0, pitchRad, yawRad)
            self.objects.append(rotation)
            self.draw_result(img_bgr, c, cX, cY)
        img_output = cv2.resize(img_bgr, (269,162)) 
        camera.output_stream.putFrame(img_output)

    def draw_result(self, img, cnt, cX, cY):
        # float_formatter = {"float_kind": lambda x: f"{x:4.1f}"}
        cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
        cv2.circle(img, (int(cX), int(cY)), 7, (0, 0, 0), -1)
        # cv2.putText(img, f"t: {np.array2string(wpi_t.flatten(), formatter=float_formatter)}", (int(cX) - 20, int(cY) - 20),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    def analyze(self, request, camera):
        img_yuv = request.make_array("lores")
        metadata = request.get_metadata()

        self.find_object(img_yuv, camera)

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


def getserial():
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""

def main():
    print("main")
    print(Picamera2.global_camera_info())
    camList = []
    if (len(Picamera2.global_camera_info()) == 0):
        print("NO CAMERAS DETECTED, PLEASE TURN OFF PI AND CHECK CAMERA PORT(S)")
    for cameraData in Picamera2.global_camera_info():
        camera = CameraData(cameraData["Num"])
        camList.append(camera)
    serial = getserial()
    print(serial)
    output = GamePieceFinder(serial, camList)
    try:
        while True:
            for camera in camList:
                request = camera.camera.capture_request()
                try:
                    output.analyze(request, camera)
                finally:
                    request.release()
            output.vision_nt_struct.set(output.objects)
            output.objects = []
    finally:
        for camera in camList:
            camera.camera.stop()
main()