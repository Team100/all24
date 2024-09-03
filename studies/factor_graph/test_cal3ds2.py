# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0621
# really to test numeric differentation
# see testCal3DS2.cpp

import unittest
import numpy as np
from gtsam import Cal3DS2, Point2  # type:ignore
from numpy.testing import assert_almost_equal

from numerical_derivative import numericalDerivative21
from numerical_derivative import numericalDerivative22


K = Cal3DS2(500, 100, 0.1, 320, 240, 1e-3, 2.0 * 1e-3, 3.0 * 1e-3, 4.0 * 1e-3)
p = Point2(2, 3)


def uncalibrate_(k: Cal3DS2, pt: Point2) -> Point2:
    return k.uncalibrate(pt)


def calibrate_(k: Cal3DS2, pt: Point2) -> Point2:
    return k.calibrate(pt)


class TestCal3DS2(unittest.TestCase):

    def test_Duncalibrate1(self) -> None:
        computed = np.zeros((2, 9), order="F")
        K.uncalibrate(p, computed, np.zeros((2, 2), order="F"))
        numerical = numericalDerivative21(uncalibrate_, K, p, 2, 9, 1e-7)
        assert_almost_equal(numerical, computed, 5)
        separate = K.D2d_calibration(p)
        assert_almost_equal(numerical, separate, 5)

    def test_Duncalibrate2(self) -> None:
        computed = np.zeros((2, 2), order="F")
        K.uncalibrate(p, np.zeros((2, 9), order="F"), computed)
        numerical = numericalDerivative22(uncalibrate_, K, p, 2, 2, 1e-7)
        assert_almost_equal(numerical, computed, 5)
        separate = K.D2d_intrinsic(p)
        assert_almost_equal(numerical, separate, 5)

    def test_Dcalibrate(self) -> None:
        pn = Point2(0.5, 0.5)
        pi = K.uncalibrate(pn)
        Dcal = np.zeros((2, 9), order="F")
        Dp = np.zeros((2, 2), order="F")
        K.calibrate(pi, Dcal, Dp)
        numerical1 = numericalDerivative21(calibrate_, K, pi, 2, 9, 1e-7)
        assert_almost_equal(numerical1, Dcal, 5)
        numerical2 = numericalDerivative22(calibrate_, K, pi, 2, 2, 1e-7)
        assert_almost_equal(numerical2, Dp, 5)


if __name__ == "__main__":
    unittest.main()
