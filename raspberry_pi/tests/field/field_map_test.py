"""Spot check field map."""

# pylint: disable=C0103,E0611,E1101,R0913


import unittest

from app.field.field_map import FieldMap


class FieldMapTest(unittest.TestCase):
    def test_tag(self) -> None:
        f = FieldMap()

        # at 4,0,1 facing 0 ('into the page' orientation)
        t = f.get(0)
        self.assertAlmostEqual(4, t[0][0], 3)
        self.assertAlmostEqual(0.083, t[0][1], 3)
        self.assertAlmostEqual(0.917, t[0][2], 3)
        self.assertAlmostEqual(4, t[1][0], 3)
        self.assertAlmostEqual(-0.083, t[1][1], 3)
        self.assertAlmostEqual(0.917, t[1][2], 3)
        self.assertAlmostEqual(4, t[2][0], 3)
        self.assertAlmostEqual(-0.083, t[2][1], 3)
        self.assertAlmostEqual(1.083, t[2][2], 3)
        self.assertAlmostEqual(4, t[3][0], 3)
        self.assertAlmostEqual(0.083, t[3][1], 3)
        self.assertAlmostEqual(1.083, t[3][2], 3)

        # at 2,2,1 facing 90 ('into the page' orientation)
        t = f.get(1)
        self.assertAlmostEqual(1.917, t[0][0], 3)
        self.assertAlmostEqual(2, t[0][1], 3)
        self.assertAlmostEqual(0.917, t[0][2], 3)
        self.assertAlmostEqual(2.083, t[1][0], 3)
        self.assertAlmostEqual(2, t[1][1], 3)
        self.assertAlmostEqual(0.917, t[1][2], 3)
        self.assertAlmostEqual(2.083, t[2][0], 3)
        self.assertAlmostEqual(2, t[2][1], 3)
        self.assertAlmostEqual(1.083, t[2][2], 3)
        self.assertAlmostEqual(1.917, t[3][0], 3)
        self.assertAlmostEqual(2, t[3][1], 3)
        self.assertAlmostEqual(1.083, t[3][2], 3)
