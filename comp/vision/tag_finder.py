# pylint: disable=missing-module-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=missing-class-docstring
# pylint: disable=import-error
import time
from enum import Enum

import cv2
import libcamera
import msgpack
import numpy as np
import ntcore
import os

from cscore import CameraServer
# from ntcore import NetworkTableInstance
from picamera2 import Picamera2
from pupil_apriltags import Detector


class Camera(Enum):
    """Keep this synchronized with java team100.config.Camera."""

    FRONT = "1000000013c9c96c" # "2"
    REAR = "100000004e0a1fb9" # "1"
    LEFT = "10000000a7c673d9" # "4"
    RIGHT = "10000000a7a892c0" # "3"
    UNKNOWN = None

    @classmethod
    def _missing_(cls, value):
        return Camera.UNKNOWN


class TagFinder:
    IMAGE_DIR = 'images'
    def __init__(self, topic_name, width, height):
        self.frame_time = time.time()
        # each camera has its own topic
        self.topic_name = topic_name
        self.width = width
        self.height = height

        # for the driver view
        scale = 0.25
        self.view_width = int(width * scale)
        self.view_height = int(height * scale)

        self.initialize_nt()

        # tag size was wrong before.  full size is 0.2m but
        # https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide
        # this points out that the apriltag library expects tag_size to be
        # the boundary between the outer white boundary and the inner black
        # boundary, which in this case is 0.15 m
        self.tag_size = 0.15
        # self.circle_tag_size = 0.8
        self.at_detector = Detector(families="tag16h5")
        # self.at_circle_detector = Detector(families="tagCircle21h7")
        # self.output_stream = CameraServer.putVideo("Processed", width, height)
        # vertical slice
        # TODO: one slice for squares one for circles
        # for now use the full frame
        #        self.output_stream = CameraServer.putVideo("Processed", width, int(height / 2))
        self.output_stream = CameraServer.putVideo("Processed", width, height)
        self.camera_params = [
            666,
            666,
            width / 2,
            height / 2,
        ]

        # make a place to put example images
        if not os.path.exists(TagFinder.IMAGE_DIR):
            os.mkdir(TagFinder.IMAGE_DIR)
        # to keep track of images to write
        self.img_ts_sec = 0

    def analyze(self, request):
        buffer = request.make_buffer("lores")
        # buffer = request.make_buffer("main")
        # metadata = request.get_metadata()
        # print(metadata)
        # sensor timestamp is the boottime when the first byte was received from the sensor
        # sensor_timestamp = metadata[
        #     "SensorTimestamp"
        # ]
        # system_time_ns = time.clock_gettime_ns(
        #     time.CLOCK_BOOTTIME
        # )
        # time_delta_ns = system_time_ns - sensor_timestamp
        # print(sensor_timestamp, system_time_ns, time_delta_ns//1000000) # ms

        start_time = time.time()
        y_len = self.width * self.height
        # truncate, ignore chrominance
        # this makes a view, takes 300ns
        img = np.frombuffer(buffer, dtype=np.uint8, count=y_len)
        # this also makes a view, takes 150ns
        img = img.reshape((self.height, self.width))
        # slice out the middle, there's never a target near the top or the bottom
        # TODO: one slice for squares one for circles
        # this also makes a view, takes 150ns
        # for now use the full frame
        #        img = img[int(self.height / 4) : int(3 * self.height / 4), : self.width]
        img = img[: self.height, : self.width]
        # for debugging the size:
        # img = np.frombuffer(buffer, dtype=np.uint8)
        # img = img.reshape((int(height*3/2), -1))
        # img = img[: height, ]
        # print(img.shape) # (self.width,self.height) = (616,832)
        # print("MIN", np.amin(img), "MAX", np.amax(img))

        # # not instant, ~300us
        # led_on = np.amax(img) > 200
        # self.vision_nt_led.set(led_on)

        # TODO: add big  tag detection?
        result = self.at_detector.detect(
            img,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size,
        )
        self.draw_result(img, result)

        tags = {}
        tags["tags"] = []

        for result_item in result:
            if result_item.hamming > 0:
                continue

            tags["tags"].append(
                {
                    "id": result_item.tag_id,
                    "pose_t": result_item.pose_t.tolist(),
                    "pose_R": result_item.pose_R.tolist(),
                }
            )

        current_time = time.time()
        # analysis_et = current_time - start_time
        total_et = current_time - self.frame_time

        tags["et"] = total_et
        # print(tags)

        posebytes = msgpack.packb(tags)

        self.vision_nt_msgpack.set(posebytes)

        fps = 1 / total_et
        self.frame_time = current_time
        #self.draw_text(img, f"analysis ET(ms) {1000*analysis_et:.0f}", (5, 25))
        #self.draw_text(img, f"total ET(ms) {1000*total_et:.0f}", (5, 65))
        #self.draw_text(img, f"fps {fps:.1f}", (5, 105))
        self.draw_text(img, f"fps {fps:.1f}", (5, 65))

        # write the current timestamp on the image
        # this timestamp is attached to the NT payloads, see getAtomic()
        now_us = ntcore._now() #pylint:disable=W0212
        self.draw_text(img, f"time(us) {now_us}", (5, 105))

        # shrink the driver view to avoid overloading the radio
        driver_img = cv2.resize(img, (self.view_width, self.view_height))
        self.output_stream.putFrame(driver_img)
        # self.output_stream.putFrame(img)

        # Write some of the files for later analysis
        # To retrieve these files, use:
        # scp pi@10.1.0.11:images/* .
        # These will accumulate forever so remember to clean it out:
        # ssh pi@10.1.0.11 "rm images/img*"
        now_s = now_us // 1000000 # once per second
        if now_s > self.img_ts_sec:
            self.img_ts_sec = now_s
            filename = TagFinder.IMAGE_DIR + "/img" + str(now_s) + ".png"
            cv2.imwrite(filename, img)


    def draw_result(self, image, result):
        for result_item in result:
            if result_item.hamming > 0:
                continue
            (pt_a, pt_b, pt_c, pt_d) = result_item.corners
            pt_a = (int(pt_a[0]), int(pt_a[1]))
            pt_b = (int(pt_b[0]), int(pt_b[1]))
            pt_c = (int(pt_c[0]), int(pt_c[1]))
            pt_d = (int(pt_d[0]), int(pt_d[1]))

            cv2.line(image, pt_a, pt_b, (255, 255, 255), 2)
            cv2.line(image, pt_b, pt_c, (255, 255, 255), 2)
            cv2.line(image, pt_c, pt_d, (255, 255, 255), 2)
            cv2.line(image, pt_d, pt_a, (255, 255, 255), 2)

            (c_x, c_y) = (int(result_item.center[0]), int(result_item.center[1]))
            cv2.circle(image, (c_x, c_y), 10, (255, 255, 255), -1)

            tag_id = result_item.tag_id
            self.draw_text(image, f"id {tag_id}", (c_x, c_y))
            # TODO: differentiate big (circle) and little tags?
            # tag_family = result_item.tag_family.decode("utf-8")
            # self.draw_text(image, f"id {tag_family}", (c_x, c_y + 40))

            # put the pose translation in the image
            # the use of 'item' here is to force a scalar to format
            float_formatter = {"float_kind": lambda x: f"{x:4.1f}"}
            if result_item.pose_t is not None:
                # wpi_axes = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
                # wpi_t = np.matmul(wpi_axes, result_item.pose_t)
                t = result_item.pose_t
                wpi_t = np.array([[t[2][0]], [-t[0][0]], [-t[1][0]]])
                # R = result_item.pose_R
                # wpi_R = np.matmul(wpi_axes, result_item.pose_R)
                # wpi_R = np.array(
                #     [
                #         # [R[0][0], R[0][1], R[0][2]],
                #         # [R[1][0], R[1][1], R[1][2]],
                #         # [R[2][0], R[2][1], R[2][2]]
                #         [R[2][2], -R[2][0], -R[2][1]],
                #         [-R[0][2], R[0][0], R[0][1]],
                #         [-R[1][2], R[1][0], R[1][1]],
                #     ]
                # )

                # this matrix is not necessarily exactly special orthogonal
                # this is sort of a hack to fix it.
                # removed this for now because it's not fast
                # wpi_RV, _ = cv2.Rodrigues(wpi_R)
                # wpi_R, _ = cv2.Rodrigues(wpi_RV)

                # self.draw_text(
                #     image, f"X {result_item.pose_t.item(0):.2f}m", (c_x, c_y + 80)
                # )
                # self.draw_text(
                #     image, f"Y {result_item.pose_t.item(1):.2f}m", (c_x, c_y + 120)
                # )
                # self.draw_text(
                #     image, f"Z {result_item.pose_t.item(2):.2f}m", (c_x, c_y + 160)
                # )

                # translation vector
                self.draw_text(
                    image,
                    f"t: {np.array2string(wpi_t.flatten(), formatter=float_formatter)}",
                    (c_x - 50, c_y + 40),
                )

                # rotation matrix (we don't use this so omit it)
                # self.draw_text(
                #     image,
                #     f"R: [{np.array2string(wpi_R[0], formatter=float_formatter)},",
                #     (c_x, c_y + 80),
                # )
                # self.draw_text(
                #     image,
                #     f"    {np.array2string(wpi_R[1], formatter=float_formatter)},",
                #     (c_x, c_y + 120),
                # )
                # self.draw_text(
                #     image,
                #     f"    {np.array2string(wpi_R[2], formatter=float_formatter)}]",
                #     (c_x, c_y + 160),
                # )

    # these are white with black outline
    def draw_text(self, image, msg, loc):
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 6)
        cv2.putText(image, msg, loc, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)

    def initialize_nt(self):
        """Start NetworkTables with Rio as server, set up publisher."""
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startClient4("tag-finder")
        # this is always the RIO IP address; set a matching static IP on your
        # laptop if you're using this in simulation.
        inst.setServer("10.1.0.2")
        # try faster flushing to minimize vision latency; update rate will be limited by frame rate.
        # be careful that this doesn't flood the rio with too many updates.
        # hmm... 6/22/23, seems like setUpdateRate doesn't exist.
        # inst.setUpdateRate(0.01)
        # Table for vision output information
        self.vision_nt = inst.getTable("Vision")
        self.vision_nt_msgpack = self.vision_nt.getRawTopic(self.topic_name).publish(
            "msgpack"
        )

    # def reconnect_nt(self):
    #     """NT doesn't recover from network disruptions by itself, nor does it
    #     recognize them, so this blindly stops and starts the client.  This is
    #     disruptive, causing flashing in Glass for example ... but better than
    #     zombie mode?"""
    #     inst = ntcore.NetworkTableInstance.getDefault()
    #     inst.stopClient()  # without this, reconnecting doesn't work
    #     inst.startClient4("tag-finder")


def getserial():
    with open("/proc/cpuinfo", "r", encoding="ascii") as cpuinfo:
        for line in cpuinfo:
            if line[0:6] == "Serial":
                return line[10:26]
    return ""


def main():
    print("main")

    # full frame, 2x2, to set the detector mode
    fullwidth = 1664  # slightly larger than the detector, to match stride
    fullheight = 1232
    # fast path, 200fps, narrow crop
    # fullwidth = 640
    # fullheight = 480
    # medium crop
    # fullwidth = 1920
    # fullheight = 1080

    # lores for apriltag detector
    # option 1: big, all the pixels yields 270ms delay, but seems even higher
    # docs say 41 fps is possible
    # width=1664
    # height=1232
    # option 2: medium, two circles, three squares, timer says ~80ms but seems more like 500ms
    # remember to note this delay in the kalman filter input
    # TODO: report the actual delay in network tables
    width = 832
    height = 616
    # option 3: tiny, trade speed for detection distance; two circles, three squraes, ~40ms
    # width=448
    # height=308
    # fast path
    # width=640
    # height=480
    # medium crop
    # width=1920
    # height=1080

    camera = Picamera2()
    # don't use video, it tries to process all the frames?
    # camera_config = camera.create_video_configuration(
    # no need for extra buffers, dropping frames is fine
    #     buffer_count=4,
    #     encode="lores",
    # use single-frame
    camera_config = camera.create_still_configuration(
        # one buffer to write, one to read, one in between so we don't have to wait
        buffer_count=6,
        main={
            "format": "YUV420",
            "size": (fullwidth, fullheight),
        },
        lores={"format": "YUV420", "size": (width, height)},
        controls={
            # these manual controls are useful sometimes but turn them off for now
            # fast shutter means more gain
            # "AnalogueGain": 8.0,
            # the flashing LED makes the Auto-Exposure freak out, so turn it off:
            # "ExposureTime": 5000,
            # go as fast as possible but no slower than 30fps
            "FrameDurationLimits": (5000, 33333),  # 41 fps
            # noise reduction takes time
            "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off,
        },
    )

    topic_name = getserial()  # was: topic_name = "tags"
    identity = Camera(topic_name)
    if identity == Camera.REAR or identity == Camera.FRONT:
        camera_config["transform"] = libcamera.Transform(hflip=1, vflip=1)

    print("REQUESTED")
    print(camera_config)
    camera.align_configuration(camera_config)
    print("ALIGNED")
    print(camera_config)
    camera.configure(camera_config)
    print(camera.camera_controls)

    # Roborio IP: 10.1.0.2
    # Pi IP: 10.1.0.21

    output = TagFinder(topic_name, width, height)
    # the callback blocks the camera thread, so don't do that.
    # camera.pre_callback = output.pre_callback
    camera.start()
    # frame_counter = 0
    try:
        while True:
            # periodically reconnect to NT.
            # without these attempts, network disruption results in
            # permanent disconnection
            # frame_counter += 1
            # disable reconnecting entirely to see if network works
            # if frame_counter > 200:
            #     print("RECONNECTING")
            #     frame_counter = 0
            #     output.reconnect_nt()
            # the most recent frame, maybe from the past
            request = camera.capture_request()
            try:
                output.analyze(request)
            finally:
                # the frame is owned by the camera so remember to release it
                request.release()
            # time.sleep(1)
    finally:
        # not in video mode
        # camera.stop_recording()
        camera.stop()


main()
