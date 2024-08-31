# really to test numeric differentation
# see testPoint.cpp

import math

import unittest
import numpy as np
from gtsam import Point3  # really np.array
from numpy.testing import assert_allclose

from numerical_derivative import numericalDerivative21DoublePoint3Point3
from numerical_derivative import numericalDerivative22DoublePoint3Point3
from numerical_derivative import numericalDerivative21Point3Point3Point3
from numerical_derivative import numericalDerivative22Point3Point3Point3


def dot(p: Point3, q: Point3, H: list[np.array] = None) -> float:
    """H1, H2, OptionalJacobian<1,3>"""
    if H is not None:
        H[0] = np.array([[q[0], q[1], q[2]]])
        H[1] = np.array([[p[0], p[1], p[2]]])
    return np.dot(p, q)


def cross(p: Point3, q: Point3, H: list[np.array] = None) -> Point3:
    """OptionalJacobian<3, 3> H1, H2"""
    if H is not None:
        H[0] = skewSymmetric(q[0] * -1, q[1] * -1, q[2] * -1)
        H[1] = skewSymmetric(p[0], p[1], p[2])
    return np.cross(p, q)


def skewSymmetric(wx: float, wy: float, wz: float) -> np.array:
    return np.array([[0.0, -wz, +wy], [+wz, 0.0, -wx], [-wy, +wx, 0.0]])


class TestPoint3(unittest.TestCase):
    def test_dot(self):
        origin = Point3(0, 0, 0)
        ones = Point3(1, 1, 1)

        self.assertAlmostEqual(origin.dot(Point3(1, 1, 0)), 0)
        self.assertAlmostEqual(ones.dot(Point3(1, 1, 0)), 2)

        p: Point3 = Point3(1, 0.2, 0.3)
        q: Point3 = p + Point3(0.5, 0.2, -3.0)
        r: Point3 = p + Point3(0.8, 0, 0)
        t: Point3 = p + Point3(0, 0.3, -0.4)
        self.assertAlmostEqual(1.130000, p.dot(p))
        self.assertAlmostEqual(0.770000, p.dot(q))
        self.assertAlmostEqual(1.930000, p.dot(r))
        self.assertAlmostEqual(1.070000, p.dot(t))

        # Use numerical derivatives to calculate the expected Jacobians
        H = [np.zeros((1, 3)), np.zeros((1, 3))]

        def f(p: Point3, q: Point3):
            return dot(p, q)

        d = dot(p, q, H)
        self.assertAlmostEqual(0.77, d)
        assert_allclose(numericalDerivative21DoublePoint3Point3(f, p, q), H[0], 1e-9)
        assert_allclose(numericalDerivative22DoublePoint3Point3(f, p, q), H[1], 1e-9)

        d = dot(p, r, H)
        self.assertAlmostEqual(1.93, d)
        assert_allclose(numericalDerivative21DoublePoint3Point3(f, p, r), H[0], 1e-9)
        assert_allclose(numericalDerivative22DoublePoint3Point3(f, p, r), H[1], 1e-9)

        d = dot(p, t, H)
        self.assertAlmostEqual(1.07, d)
        assert_allclose(numericalDerivative21DoublePoint3Point3(f, p, t), H[0], 1e-9)
        assert_allclose(numericalDerivative22DoublePoint3Point3(f, p, t), H[1], 1e-9)

    def test_cross(self):
        H = [np.zeros((3, 3)), np.zeros((3, 3))]

        def f(p: Point3, q: Point3) -> Point3:
            return cross(p, q)

        omega = Point3(0, 1, 0)
        theta = Point3(4, 6, 8)

        cross(omega, theta, H)
        assert_allclose(numericalDerivative21Point3Point3Point3(f, omega, theta), H[0])
        assert_allclose(numericalDerivative22Point3Point3Point3(f, omega, theta), H[1])

    def test_cross2(self):
        p: Point3 = Point3(1, 0.2, 0.3)
        q: Point3 = p + Point3(0.5, 0.2, -3.0)
        r: Point3 = p + Point3(0.8, 0, 0)
        assert_allclose(Point3(0, 0, 0), np.cross(p, p))
        assert_allclose(Point3(-0.66, 3.15, 0.1), np.cross(p, q))
        assert_allclose(Point3(0, 0.24, -0.16), np.cross(p, r))

        # Use numerical derivatives to calculate the expected Jacobians
        H = [np.zeros((3, 3)), np.zeros((3, 3))]

        def f(p: Point3, q: Point3):
            return cross(p, q)

        cross(p, q, H)
        assert_allclose(numericalDerivative21Point3Point3Point3(f, p, q), H[0])
        assert_allclose(numericalDerivative22Point3Point3Point3(f, p, q), H[1])

        cross(p, r, H)
        assert_allclose(numericalDerivative21Point3Point3Point3(f, p, r), H[0])
        assert_allclose(numericalDerivative22Point3Point3Point3(f, p, r), H[1])


if __name__ == "__main__":
    unittest.main()
