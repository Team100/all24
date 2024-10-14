import unittest

import numpy as np
from wpimath.geometry import Rotation3d


class CrashTest(unittest.TestCase):
    def test_crash1(self) -> None:
        rotation = Rotation3d()
        self.assertEqual(0, rotation.X())
        self.assertEqual(0, rotation.Y())
        self.assertEqual(0, rotation.Z())

    def test_crash2(self) -> None:
        initial = np.array([1, 0, 0], dtype=np.float64)
        final = np.array([1, 0, 0], dtype=np.float64)
        rotation = Rotation3d(initial=initial, final=final)
        self.assertEqual(0, rotation.X())
        self.assertEqual(0, rotation.Y())
        self.assertEqual(0, rotation.Z())
