import unittest

from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.localization.tag_detector import TagDetector
from app.network.network import Network


class TagDetectorTest(unittest.TestCase):

    def test_one_tag_found(self) -> None:
        identity = Identity.UNKNOWN
        network = Network(identity)
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
        network = Network(identity)
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

    def test_distortion(self) -> None:
        """How much distortion can there be in the image?"""
        identity = Identity.UNKNOWN
        network = Network(identity)

        # no distortion, like above
        camera = FakeCamera("tag_and_board.jpg", (1100, 620), 0)
        display = FakeDisplay()
        TagDetector(identity, camera, 0, display, network).analyze(
            camera.capture_request()
        )
        self.assertEqual(1, len(display.tags))

        # this is about the most possible
        camera = FakeCamera("tag_and_board.jpg", (1100, 620), -5)
        display = FakeDisplay()
        TagDetector(identity, camera, 0, display, network).analyze(
            camera.capture_request()
        )
        self.assertEqual(1, len(display.tags))

        # this is too much distortion
        # I tried increasing QuadThresholdParameters.maxLineFitMSE to 100
        # to try to relax the straight line fit to the curved lines, but that didn't work.
        camera = FakeCamera("tag_and_board.jpg", (1100, 620), -6)
        display = FakeDisplay()
        TagDetector(identity, camera, 0, display, network).analyze(
            camera.capture_request()
        )
        self.assertEqual(0, len(display.tags))
