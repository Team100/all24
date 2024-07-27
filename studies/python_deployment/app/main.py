""" This is the coprocessor main loop.

It takes data from the camera and the gyro, and publishes it
on network tables.
"""

from app.camera import Camera, Factory, Request, Size
from app.identity import Identity
from app.network import Network
from app.tag_detector import TagDetector
from app.timer import Timer


def main() -> None:
    print("main")
    identity: Identity = Identity.get()
    camera: Camera = Factory.get(identity)
    size: Size = camera.get_size()
    tag_detector: TagDetector = TagDetector(identity, size.width, size.height, camera)
    network: Network = Network(identity)

    camera.start()
    try:
        while True:
            # the most recent completed frame, from the recent past
            capture_start: int = Timer.time_ns()
            request: Request = camera.capture_request()
            capture_end: int = Timer.time_ns()
            # capture time is how long we wait for the camera, it should be close to zero.
            capture_time_ms: int = (capture_end - capture_start) // 1000000
            network.log_capture_time(capture_time_ms)
            try:
                tag_detector.analyze(request)
            finally:
                request.release()
    finally:
        camera.stop()
