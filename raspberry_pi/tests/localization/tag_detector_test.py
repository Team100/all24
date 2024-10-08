import unittest

from app.camera.camera_protocol import Camera, Request
from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.display import Display
from app.localization.network import Network
from app.localization.tag_detector import TagDetector


class TagDetectorTest(unittest.TestCase):
    def test_tag_detector(self) -> None:
        identity: Identity = Identity.UNKNOWN
        network: Network = Network(identity, 0)
        camera: Camera = FakeCamera()
        display: Display = Display(100, 100, 0)
        tag_detector: TagDetector = TagDetector(
            identity, 100, 100, camera, display, network
        )
        request: Request = camera.capture_request()
        tag_detector.analyze(request)
        # TODO: add an assertion
