import unittest

import ntcore
import numpy as np
from wpimath.geometry import Rotation3d

from app.camera.fake_camera import FakeCamera
from app.config.identity import Identity
from app.dashboard.fake_display import FakeDisplay
from app.localization.note_detector import NoteDetector
from app.network.network import Network


class NoteDetectorTest(unittest.TestCase):
    KEY = "noteVision/unknown/0/Rotation3d"

    def test_one_note_found(self) -> None:

        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getStructArrayTopic(self.KEY, Rotation3d).subscribe([])

        identity = Identity.UNKNOWN
        network = Network(identity)
        # this has an orange blob that matches the
        # HSV range in the note detector
        # the blob is in the lower right quadrant, so the result
        # should be pitch-down yaw-right.
        # ORANGE TARGET
        # camera = FakeCamera("blob.jpg")
        # GREEN PRACTICE TARGET
        camera = FakeCamera("green_blob.jpg")
        display = FakeDisplay()
        
        # GREEN TARGET VALUES
        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))
        note_detector = NoteDetector(
            identity, camera, 0, display, network, object_lower, object_higher
        )
        request = camera.capture_request()
        note_detector.analyze(request)

        self.assertEqual(1, len(display.notes))
        self.assertEqual(1, len(display.circles))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        self.assertAlmostEqual(482, display.circles[0][0], -1)
        self.assertEqual(468, display.circles[0][1])

        rots = sub.get()
        self.assertEqual(1, len(rots))
        rot = rots[0]
        # NOTE: 0.01 rad resolution is all that can be expected.
        # ~zero
        self.assertAlmostEqual(-0.03, rot.x, 2)
        # pitch down
        self.assertAlmostEqual(0.33, rot.y, 2)
        # yaw right
        self.assertAlmostEqual(-0.17, rot.z, 2)
        rot2d = rot.toRotation2d()
        # right yaw is about 10 deg
        self.assertAlmostEqual(-9.69, rot2d.degrees(), 2)
        q = rot.getQuaternion()
        self.assertAlmostEqual(0, q.X(), 2)
        self.assertAlmostEqual(0.17, q.Y(), 2)
        self.assertAlmostEqual(-0.08, q.Z(), 2)
        self.assertAlmostEqual(0.98, q.W(), 2)

    def test_zero_notes_found(self) -> None:
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        sub = inst.getStructArrayTopic(self.KEY, Rotation3d).subscribe([])

        identity = Identity.UNKNOWN
        network = Network(identity)

        # nothing in this image
        camera = FakeCamera("white_square.jpg")
        display = FakeDisplay()

        # GREEN TARGET VALUES
        object_lower = np.array((40, 50, 100))
        object_higher = np.array((70, 255, 255))
        note_detector = NoteDetector(
            identity, camera, 0, display, network, object_lower, object_higher
        )
        request = camera.capture_request()
        note_detector.analyze(request)

        self.assertEqual(0, len(display.notes))
        self.assertEqual(0, len(display.circles))
        self.assertEqual(2, len(display.msgs))
        self.assertEqual(2, len(display.locs))
        self.assertEqual(1, display.frame_count)

        ## always publish even if empty

        rots = sub.get()

        self.assertEqual(0, len(rots))
