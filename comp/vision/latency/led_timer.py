# pylint: disable=missing-module-docstring
# pylint: disable=missing-function-docstring
# pylint: disable=missing-class-docstring

"""
led-timer.py

This is a standalone app for the WPILibPi vision system, specifically
to investigate video pipeline latency, separate from any video
interpretation logic.

To use it, navigate to the "Application" tab of the WPILibPi webserver,
click "Writable" at the top, choose "Uploaded Python File," choose this
file and click "Upload and Save."
"""

import time
import numpy as np
import libcamera
from RPi import GPIO
from cscore import CameraServer
from ntcore import NetworkTableInstance
from picamera2 import MappedArray
from picamera2 import Picamera2


class LEDFinder:
    def __init__(self, width, height):
        self.width = width
        self.height = height

        inst = NetworkTableInstance.getDefault()
        inst.startClient4("led-finder")
        inst.setServer("10.1.0.2")  # RoboRIO IP

        # Table for vision output information
        self.vision_nt = inst.getTable("Vision")
        self.vision_nt_led = self.vision_nt.getBooleanTopic("led").publish()

        # self.output_stream = CameraServer.putVideo("Processed", width, height)
        # vertical slice
        self.output_stream = CameraServer.putVideo(
            "Processed", self.width, int(self.height / 2)
        )
        self.camera_params = [
            357.1,
            357.1,
            self.width / 2,
            self.height / 2,
        ]

    # the callback blocks the camera thread, so don't do that.
    #    def pre_callback(self, request):
    #        with MappedArray(request, "lores") as mapped_array:
    #            self.analyze(mapped_array.array)

    # def analyze(self, buffer):
    def analyze(self, request):
        buffer = request.make_buffer("lores")
        #        buffer = request.make_buffer("main")
        metadata = request.get_metadata()
        # print(metadata)
        # sensor timestamp is the boottime when the first byte was received from the sensor
        sensor_timestamp = metadata["SensorTimestamp"]
        system_time_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        time_delta_ns = system_time_ns - sensor_timestamp
        # print(sensor_timestamp, system_time_ns, time_delta_ns//1000000) # ms
        y_len = self.width * self.height
        # truncate, ignore chrominance
        # this makes a view, takes 300ns
        img = np.frombuffer(buffer, dtype=np.uint8, count=y_len)
        # this also makes a view, takes 150ns
        img = img.reshape((self.height, self.width))
        # slice out the middle, there's never a target near the top or the bottom
        # TODO: one slice for squares one for circles
        # this also makes a view, takes 150ns
        img = img[int(self.height / 4) : int(3 * self.height / 4), : self.width]
        # img = img[:self.height, :self.width]
        # for debugging the size:
        # img = np.frombuffer(buffer, dtype=np.uint8)
        # img = img.reshape((int(height*3/2), -1))
        # img = img[: height, ]
        # print(img.shape) # (self.width,self.height) = (616,832)
        # print("MIN", np.amin(img), "MAX", np.amax(img))

        # not instant, ~300us
        led_on = np.amax(img) > 200
        self.vision_nt_led.set(led_on)

        self.output_stream.putFrame(img)

        # print(led_on)
        if led_on:
            GPIO.output(37, 1)
        else:
            GPIO.output(37, 0)


def main():
    print("main")

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(37, GPIO.OUT)

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
            # fast shutter means more gain
            "AnalogueGain": 8.0,
            # the flashing LED makes the Auto-Exposure freak out, so turn it off:
            "ExposureTime": 5000,
            # go as fast as possible but no slower than 30fps
            # "FrameDurationLimits": (100, 33333),
            "FrameDurationLimits": (24000, 33333),  # 41 fps
            # noise reduction takes time
            "NoiseReductionMode": libcamera.controls.draft.NoiseReductionModeEnum.Off,
        },
    )
    print("REQUESTED")
    print(camera_config)
    camera.align_configuration(camera_config)
    print("ALIGNED")
    print(camera_config)
    camera.configure(camera_config)
    print(camera.camera_controls)

    # Roborio IP: 10.1.0.2
    # Pi IP: 10.1.0.21

    output = LEDFinder(width, height)
    # the callback blocks the camera thread, so don't do that.
    # camera.pre_callback = output.pre_callback
    camera.start()
    try:
        while True:
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
