import sys
import numpy as np
import cv2
import time
from enum import Enum

from cscore import CameraServer
from ntcore import NetworkTableInstance
import ntcore as nt

from wpimath.geometry import Translation2d
from picamera2 import Picamera2
import libcamera


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


class MouseVision:
    def __init__(self, serial, width, height, model, bwimg):
        self.serial = serial
        self.width = width
        self.height = height
        self.model = model
        self.prev_gray = bwimg
        self.fps = 0
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=10,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )

        self.feature_params = dict(
            maxCorners=20, qualityLevel=0.4 , minDistance=10, blockSize=7
        )

        self.trajectory_len = 2
        self.detect_interval = 1
        self.trajectories = []
        self.frame_idx = 0
        # opencv hue values are 0-180, half the usual number
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

    def initialize_nt(self):
        """Start NetworkTables with Rio as server, set up publisher."""
        self.inst = NetworkTableInstance.getDefault()
        self.inst.startClient4("gamepiece_finder24")

        # roboRio address. windows machines can impersonate this for simulation.
        self.inst.setServer("10.1.0.2")

        topic_name = "mouseVision/" + self.serial
        self.vision_fps = self.inst.getDoubleTopic(topic_name + "/fps").publish()
        self.vision_latency = self.inst.getDoubleTopic(
            topic_name + "/latency"
        ).publish()

        self.vision_nt_struct = self.inst.getStructTopic(
            topic_name + "/Translation2d", Translation2d
        ).publish()

    def analyze(self, request):
        img_yuv = request.make_array("lores")
        frame = cv2.cvtColor(img_yuv, cv2.COLOR_YUV420p2RGB)
        # frame = cv2.resize(frame, (368,544))
        # this  makes a view, very fast (150 ns)
        # start time to calculate FPS
        metadata = request.get_metadata()
        sensor_timestamp = metadata["SensorTimestamp"]
        system_time_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        time_delta_ms = (system_time_ns - sensor_timestamp) // 1000000
        self.vision_latency.set(time_delta_ms)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = frame.copy()
        # Calculate optical flow for a sparse feature set using thje iterative Lucas-Kanade Method
        if len(self.trajectories) > 0:
            img0, img1 = self.prev_gray, frame_gray
            p0 = np.float32(
                [trajectory[-1] for trajectory in self.trajectories]
            ).reshape(-1, 1, 2)
            p1, _st, _err = cv2.calcOpticalFlowPyrLK(
                img0, img1, p0, None, **self.lk_params
            )
            p0r, _st, _err = cv2.calcOpticalFlowPyrLK(
                img1, img0, p1, None, **self.lk_params
            )
            d = abs(p0 - p0r).reshape(-1, 2).max(-1)
            good = d < 1

            new_trajectories = []

            # Get all the trajectories
            for trajectory, (x, y), good_flag in zip(
                self.trajectories, p1.reshape(-1, 2), good
            ):
                if not good_flag:
                    continue
                trajectory.append((x, y))
                if len(trajectory) > self.trajectory_len:
                    del trajectory[0]
                new_trajectories.append(trajectory)
                # Newest detected point
                cv2.circle(img, (int(x), int(y)), 2, (0, 0, 255), -1)

            self.trajectories = new_trajectories

            # Draw all the trajectories
            cv2.polylines(
                img,
                [np.int32(trajectory) for trajectory in self.trajectories],
                False,
                (0, 255, 0),
            )
            cv2.putText(
                img,
                "track count: %d" % len(self.trajectories),
                (20, 50),
                cv2.FONT_HERSHEY_PLAIN,
                0.5,
                (0, 255, 0),
                2,
            )
            dy = 0
            dx = 0
            average = 0
            for trjactory in new_trajectories:
                x = (trjactory[1][0] - trjactory[0][0]) * self.fps
                dx += x
                y = (trjactory[1][1] - trjactory[0][1]) * self.fps
                dy += y
                average += 1
            if len(new_trajectories) > 0:
                averageX = dx / average
                averageY = dy / average
                # Puts it up in robot cords
                self.vision_nt_struct.set(Translation2d(averageY,averageX))
                print("DX: " + str(averageX) + " DY: " + str(averageY))
        # Update interval - When to update and detect new features
        if self.frame_idx % self.detect_interval == 0:
            mask = np.zeros_like(frame_gray)
            mask[:] = 255

            # Lastest point in latest trajectory
            for x, y in [np.int32(trajectory[-1]) for trajectory in self.trajectories]:
                cv2.circle(mask, (x, y), 5, 0, -1)

            # Detect the good features to track
            p = cv2.goodFeaturesToTrack(frame_gray, mask=mask, **self.feature_params)
            if p is not None:
                # If good features can be tracked - add that to the trajectories
                for x, y in np.float32(p).reshape(-1, 2):
                    self.trajectories.append([(x, y)])

        self.frame_idx += 1
        self.prev_gray = frame_gray

        # End time
        # calculate the FPS for current frame detection
        current_time = time.time()
        total_et = current_time - self.frame_time
        self.frame_time = current_time

        self.fps = 1 / total_et
        self.vision_fps.set(self.fps)
        # Show Results
        cv2.putText(
            img,
            f"{self.fps:.2f} FPS",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 0),
            2,
        )
        # output = cv2.resize(img, (self.width,self.height))
        self.output_stream.putFrame(img)


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
    elif model == "imx296":
        print("GS Camera")
        fullwidth = 1456  # slightly larger than the detector, to match stride
        fullheight = 1088
        # medium detection resolution, compromise speed vs range
        width = 64    
        height = 64    
    else:
        print(  "UNKNOWN CAMERA: " + model)
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
            "AnalogueGain": 8.0,
            # try faster shutter to reduce blur.  with 3ms, 3 rad/s seems ok.
            # 3/23/24, reduced to 2ms, even less blur.
            "ExposureTime": 4000,
            # limit auto: go as fast as possible but no slower than 30fps
            # without a duration limit, we slow down in the dark, which is fine
            # "FrameDurationLimits": (5000, 33333),  # 41 fps
            # noise reduction takes time, don't need it.
            "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off,
            # "ScalerCrop": (
            #     int((fullwidth - width) / 2),
            #     int((fullheight - height) / 2),
            #     width,
            #     height,
            # ),
        },
    )

    serial = getserial()

    print("\nREQUESTED CONFIG")
    print(camera_config)
    camera.align_configuration(camera_config)
    print("\nALIGNED CONFIG")
    print(camera_config)
    camera.configure(camera_config)
    print("\nCONTROLS")
    print(camera.camera_controls)
    print(serial)
    camera.start()
    img_yuv = camera.capture_request().make_array("lores")
    frame_gray = cv2.cvtColor(img_yuv, cv2.COLOR_YUV420p2GRAY)
    output = MouseVision(serial, width, height, model, frame_gray)
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