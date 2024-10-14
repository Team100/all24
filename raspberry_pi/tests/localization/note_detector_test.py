import unittest

from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.localization.note_detector import NoteDetector
from app.network.fake_network import FakeNetwork


class NoteDetectorTest(unittest.TestCase):
    KEY = "noteVision/unknown/0/Rotation3d"

    def test_one_note_found(self) -> None:
        identity = Identity.UNKNOWN
        network = FakeNetwork()
        # this has an orange blob that matches the
        # HSV range in the note detector
        # the blob is in the lower right quadrant, so the result
        # should be pitch-down yaw-right.
        camera = FakeCamera("blob.jpg")
        display = FakeDisplay()
        note_detector = NoteDetector(identity, camera, 0, display, network)
        request = camera.capture_request()
        note_detector.analyze(request)

        self.assertEqual(1, len(display.notes))
        self.assertEqual(1, len(display.circles))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertEqual(482, display.circles[0][0])
        self.assertEqual(468, display.circles[0][1])

        self.assertIn(self.KEY, network.notes)
        rots = network.notes[self.KEY]
        self.assertEqual(1, len(rots))
        rot = rots[0]
        # ~zero
        self.assertAlmostEqual(-0.0284, rot.x, 3)
        # pitch down
        self.assertAlmostEqual(0.332, rot.y, 3)
        # yaw right
        self.assertAlmostEqual(-0.169, rot.z, 3)
        rot2d = rot.toRotation2d()
        # right yaw is about 10 deg
        self.assertAlmostEqual(-9.69, rot2d.degrees(), 2)
        q = rot.getQuaternion()
        self.assertAlmostEqual(0, q.X(), 3)
        self.assertAlmostEqual(0.166, q.Y(), 3)
        self.assertAlmostEqual(-0.081, q.Z(), 3)
        self.assertAlmostEqual(0.983, q.W(), 3)

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
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        ## always publish even if empty
        self.assertEqual(1, len(network.notes))
        self.assertIn(self.KEY, network.notes)
        rots = network.notes[self.KEY]
        self.assertEqual(0, len(rots))
