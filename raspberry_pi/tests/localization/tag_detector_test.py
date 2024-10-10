import unittest

from app.camera.camera_protocol import Camera, Request
from app.camera.fake_camera import BlindCamera, FakeCamera
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.localization.network import Network
from app.localization.tag_detector import TagDetector


class TagDetectorTest(unittest.TestCase):

    def test_one_tag_found(self) -> None:
        """The fake camera produces one detection."""
        identity: Identity = Identity.UNKNOWN
        network: Network = Network(identity, 0)
        camera: Camera = FakeCamera()
        display: FakeDisplay = FakeDisplay()
        tag_detector: TagDetector = TagDetector(identity, camera, 0, display, network)
        request: Request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(1, len(display.result_items))
        self.assertEqual(1, len(display.poses))
        self.assertEqual(5, len(display.msgs))
        self.assertEqual(5, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertAlmostEqual(283, display.result_items[0].getCenter().x, 0)
        self.assertAlmostEqual(348, display.result_items[0].getCenter().y, 0)
        self.assertAlmostEqual(-0.191, display.poses[0].x, 3)
        self.assertAlmostEqual(0.028, display.poses[0].y, 3)
        self.assertAlmostEqual(0.354, display.poses[0].z, 3)

    def test_zero_tags_found(self) -> None:
        """The blind camera produces zero detections."""
        identity: Identity = Identity.UNKNOWN
        network: Network = Network(identity, 0)
        camera: Camera = BlindCamera()
        display: FakeDisplay = FakeDisplay()
        tag_detector: TagDetector = TagDetector(identity, camera, 0, display, network)
        request: Request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(0, len(display.result_items))
        self.assertEqual(0, len(display.poses))
        self.assertEqual(5, len(display.msgs))
        self.assertEqual(5, len(display.locs))
        self.assertEqual(1, display.frame_count)
