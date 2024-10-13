import unittest

from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.network.fake_network import FakeNetwork
from app.localization.tag_detector import TagDetector


class TagDetectorTest(unittest.TestCase):

    def test_one_tag_found(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        # there are many tags in this file but only the big one
        # is seen by the detector
        # the jpg is very large, so scale it down
        camera = FakeCamera("tag_and_board.jpg", (1100, 620))
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, 0, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(1, len(display.tags))
        self.assertEqual(1, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertAlmostEqual(282, display.tags[0].getCenter().x, 0)
        self.assertAlmostEqual(349, display.tags[0].getCenter().y, 0)
        self.assertAlmostEqual(-0.191, display.poses[0].x, 3)
        self.assertAlmostEqual(0.028, display.poses[0].y, 3)
        self.assertAlmostEqual(0.353, display.poses[0].z, 3)

    def test_zero_tags_found(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        # nothing in this image
        camera = FakeCamera("white_square.jpg")
        display = FakeDisplay()
        tag_detector = TagDetector(identity, camera, 0, display, network)
        request = camera.capture_request()
        tag_detector.analyze(request)

        self.assertEqual(0, len(display.tags))
        self.assertEqual(0, len(display.poses))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)
