""" This is the coprocessor main loop.

It takes data from the camera and the gyro, and publishes it
on network tables.
"""

import time

from app import example1
from app.camera import Camera
from app.network import Network
from app.tag_detector import TagDetector
from app2 import example3


def main():
    print("main")
    camera = Camera()
    tag_detector = TagDetector()
    network = Network()

    camera.start()
    try:
        while True:
            # the most recent completed frame, from the recent past
            capture_start = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            request = camera.capture_request()
            capture_end = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            # capture time is how long we wait for the camera, it should be close to zero.
            capture_time_ms = (capture_end - capture_start) // 1000000
            network.log_capture_time(capture_time_ms)
            try:
                tag_detector.analyze(request)
            finally:
                request.release()
    finally:
        camera.stop()

    example1.do_example1()
    example3.do_example3()
