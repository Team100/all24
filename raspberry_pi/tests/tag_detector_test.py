import unittest

from app.camera import Camera, Request
from app.camera_factory import CameraFactory
from app.display import Display
from app.fake_camera import FakeRequest
from app.network import Network
from app.tag_detector import TagDetector
from app.config.identity import Identity


class TagDetectorTest(unittest.TestCase):
    def test_tag_detector(self) -> None:
        identity: Identity = Identity.UNKNOWN
        network: Network = Network(identity, 0)
        cameras: list[Camera] = CameraFactory.get(identity)
        display: Display = Display(100, 100, 0)
        tag_detector: TagDetector = TagDetector(
            identity, 100, 100, cameras[0], display, network
        )
        request: Request = FakeRequest("")
        tag_detector.analyze(request)
        # TODO: add an assertion
