import unittest

from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.localization.note_detector import NoteDetector
from app.network.fake_network import FakeNetwork


class NoteDetectorTest(unittest.TestCase):

    def test_one_note_found(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        # this has an orange blob that matches the
        # HSV range in the note detector
        camera = FakeCamera("blob.jpg")
        display = FakeDisplay()
        note_detector = NoteDetector(identity, camera, 0, display, network)
        request = camera.capture_request()
        note_detector.analyze(request)

        self.assertEqual(1, len(display.notes))
        self.assertEqual(1, len(display.circles))
        self.assertEqual(1, len(display.msgs))
        self.assertEqual(1, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertEqual(482, display.circles[0][0])
        self.assertEqual(468, display.circles[0][1])

    def test_zero_notes_found(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        # nothing in this image
        camera = FakeCamera("white_square.jpg")
        display = FakeDisplay()
        note_detector = NoteDetector(identity, camera, 0, display, network)
        request = camera.capture_request()
        note_detector.analyze(request)

        self.assertEqual(0, len(display.notes))
        self.assertEqual(0, len(display.circles))
        self.assertEqual(1, len(display.msgs))
        self.assertEqual(1, len(display.locs))
        self.assertEqual(1, display.frame_count)
