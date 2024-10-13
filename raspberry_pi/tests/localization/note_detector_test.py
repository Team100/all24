import unittest

from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.localization.network import Network
from app.localization.note_detector import NoteDetector


class NoteDetectorTest(unittest.TestCase):

    def test_note_detector(self) -> None:

        identity = Identity.UNKNOWN
        network = Network(identity)
        camera = FakeCamera("blob.jpg")
        display = FakeDisplay()
        note_detector = NoteDetector(identity, camera, 0, display, network)
        request = camera.capture_request()
        note_detector.analyze(request)

        self.assertEqual(1, len(display.result_items))
        self.assertEqual(1, len(display.poses))
        self.assertEqual(5, len(display.msgs))
        self.assertEqual(5, len(display.locs))
        self.assertEqual(1, display.frame_count)
