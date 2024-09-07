# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0913
# mypy: disable-error-code="import-untyped"
# really to test numeric differentation
# see testPoint2.cpp

import math
from typing import Optional
import unittest
import numpy as np
from gtsam import Point2  # really np.array
from numpy.testing import assert_allclose

from numerical_derivative import numericalDerivative11
from numerical_derivative import numericalDerivative21
from numerical_derivative import numericalDerivative22

x1 = Point2(0, 0)
x2 = Point2(1, 1)
x3 = Point2(1, 1)
l1 = Point2(1, 0)
l2 = Point2(1, 1)
l3 = Point2(2, 2)
l4 = Point2(1, 3)


def norm_proxy(point: Point2) -> float:
    return np.linalg.norm(point) #type:ignore


def distance_proxy(location: Point2, point: Point2) -> float:
    return distance2(location, point)


def norm2(p: Point2, H: Optional[list[np.array]] = None) -> float:
    """
    p: Point2, which is really just np.array
    H: OptionalJacobian<1,2>, really np.array[] always 2d
    """
    r: float = math.sqrt(p[0] * p[0] + p[1] * p[1])
    if H is not None:
        if abs(r) > 1e-10:
            H[0] = np.array([[p[0] / r, p[1] / r]])
        else:
            H[0] = np.array([[1, 1]])  # really infinity, why 1 ?
    return r


def distance2(p: Point2, q: Point2, H: Optional[list[np.array]] = None) -> float:
    """
    H1: OptionalJacobian<1, 2>
    H2: OptionalJacobian<1, 2>
    """
    d: Point2 = q - p
    if H is not None:
        hz = [np.zeros((1, 2))]
        r: float = norm2(d, hz)
        H[0] = hz[0] * -1.0
        H[1] = hz[0]
        return r
    else:
        return np.linalg.norm(d) #type:ignore


class TestPoint2(unittest.TestCase):
    def test_norm(self) -> None:
        p0 = Point2(math.cos(5.0), math.sin(5.0))
        self.assertAlmostEqual(1, np.linalg.norm(p0)) #type:ignore
        p1 = Point2(4, 5)
        p2 = Point2(1, 1)
        self.assertAlmostEqual(5, distance2(p1, p2))
        self.assertAlmostEqual(5, np.linalg.norm((p2 - p1))) #type:ignore

        actualH = [np.zeros((1, 2))]

        # exception, for (0,0) derivative is [Inf,Inf] but we return [1,1]
        actual = norm2(x1, actualH)
        self.assertAlmostEqual(0, actual)
        expectedH = np.array([[1.0, 1.0]])
        assert_allclose(expectedH, actualH[0])

        actual = norm2(x2, actualH)
        self.assertAlmostEqual(math.sqrt(2.0), actual)
        expectedH = numericalDerivative11(norm_proxy, x2)
        assert_allclose(expectedH, actualH[0])

        # analytical
        expectedH = np.array([[x2[0] / actual, x2[1] / actual]])
        assert_allclose(expectedH, actualH[0])

    def test_distance(self) -> None:
        # establish distance is indeed 1
        self.assertAlmostEqual(1, distance2(x1, l1))

        # establish distance is indeed 45 degrees
        self.assertAlmostEqual(math.sqrt(2.0), distance2(x1, l2))

        # Another pair
        actualH = [np.zeros((1, 2)), np.zeros((1, 2))]
        actual23: float = distance2(x2, l3, actualH)
        assert_allclose(actualH[0], np.array([[-math.sqrt(2)/2, -math.sqrt(2)/2]]))
        assert_allclose(actualH[1], np.array([[math.sqrt(2)/2, math.sqrt(2)/2]]))
        self.assertAlmostEqual(math.sqrt(2.0), actual23)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(distance_proxy, x2, l3)
        expectedH2 = numericalDerivative22(distance_proxy, x2, l3)
        assert_allclose(expectedH1, actualH[0])
        assert_allclose(expectedH2, actualH[1])

        # Another test
        actual34: float = distance2(x3, l4, actualH)
        self.assertAlmostEqual(2, actual34)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21(distance_proxy, x3, l4)
        expectedH2 = numericalDerivative22(distance_proxy, x3, l4)
        assert_allclose(expectedH1, actualH[0])
        assert_allclose(expectedH2, actualH[1])


if __name__ == "__main__":
    unittest.main()
