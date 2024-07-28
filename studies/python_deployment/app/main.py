""" This is the coprocessor main loop.

It takes data from the camera and the gyro, and publishes it
on network tables.
"""

from app.camera import Camera, Factory, Request, Size
from app.display import Display
from app.gyro import Gyro
from app.identity import Identity
from app.network import Network
from app.tag_detector import TagDetector
from app.timer import Timer


def main() -> None:
    print("main")
    identity: Identity = Identity.get()
    camera: Camera = Factory.get(identity)
    size: Size = camera.get_size()
    display: Display = Display(size.width, size.height)
    network: Network = Network(identity)
    tag_detector: TagDetector = TagDetector(
        identity, size.width, size.height, camera, display, network
    )
    gyro: Gyro = Gyro(network)

    camera.start()
    try:
        while True:
            # the most recent completed frame, from the recent past
            capture_start: int = Timer.time_ns()
            request: Request = camera.capture_request()
            capture_end: int = Timer.time_ns()
            # capture time is how long we wait for the camera, it should be close to zero.
            capture_time_ms: int = (capture_end - capture_start) // 1000000
            network.vision_capture_time_ms.set(capture_time_ms)
            try:
                tag_detector.analyze(request)
                gyro.sample()
            finally:
                request.release()
    finally:
        camera.stop()
