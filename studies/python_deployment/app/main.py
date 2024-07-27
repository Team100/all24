""" This is the coprocessor main loop.

It takes data from the camera and the gyro, and publishes it
on network tables.
"""

import time

from app.camera import Camera, Request, Size
from app.identity import Identity
from app.network import Network
from app.tag_detector import TagDetector


def main() -> None:
    print("main")
    camera: Camera = Camera()
    size: Size = camera.get_size()
    tag_detector: TagDetector = TagDetector(size.width, size.height)
    identity: Identity = Identity.get()
    network: Network = Network(identity)

    camera.start()
    try:
        while True:
            # the most recent completed frame, from the recent past
            capture_start: int = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            request: Request = camera.capture_request()
            capture_end: int = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            # capture time is how long we wait for the camera, it should be close to zero.
            capture_time_ms: int = (capture_end - capture_start) // 1000000
            network.log_capture_time(capture_time_ms)
            try:
                tag_detector.analyze(request)
            finally:
                request.release()
    finally:
        camera.stop()
