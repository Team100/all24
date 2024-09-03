# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913
# really to test numeric differentation
# see testPose2.cpp

import math

import unittest
import numpy as np
from gtsam import Point2, Pose2, Rot2  # type:ignore
from numpy.testing import assert_almost_equal

from numerical_derivative import numericalDerivative11Point2Pose2
from numerical_derivative import numericalDerivative11Pose2Pose2
from numerical_derivative import numericalDerivative21Point2Pose2Point2
from numerical_derivative import numericalDerivative22Point2Pose2Point2
from numerical_derivative import numericalDerivative11Pose2Vector3
from numerical_derivative import numericalDerivative11Vector3Pose2
from numerical_derivative import numericalDerivative21Pose2Pose2Pose2
from numerical_derivative import numericalDerivative22Pose2Pose2Pose2
from numerical_derivative import numericalDerivative21Rot2Pose2Point2
from numerical_derivative import numericalDerivative22Rot2Pose2Point2
from numerical_derivative import numericalDerivative11
from numerical_derivative import numericalDerivative21DoublePose2Point2
from numerical_derivative import numericalDerivative22DoublePose2Point2
from numerical_derivative import numericalDerivative21Rot2Pose2Pose2
from numerical_derivative import numericalDerivative22Rot2Pose2Pose2
from numerical_derivative import numericalDerivative21DoublePose2Pose2
from numerical_derivative import numericalDerivative22DoublePose2Pose2


# some shared test values
x1: Pose2 = Pose2()
x2: Pose2 = Pose2(1, 1, 0)
x3: Pose2 = Pose2(1, 1, math.pi / 4.0)
l1: Point2 = Point2(1, 0)
l2: Point2 = Point2(1, 1)
l3: Point2 = Point2(2, 2)
l4: Point2 = Point2(1, 3)


def bearing_proxy(pose: Pose2, pt: Point2) -> Rot2:
    return pose.bearing(pt)


def bearing_pose_proxy(pose: Pose2, pt: Pose2) -> Rot2:
    return pose.bearing(pt)


def range_proxy(pose: Pose2, point: Point2) -> float:
    return pose.range(point)


def range_pose_proxy(pose: Pose2, point: Pose2) -> float:
    return pose.range(point)


def transformFrom_(pose: Pose2, point: Point2) -> Point2:
    return pose.transformFrom(point)


def transformTo_(pose: Pose2, point: Point2) -> Point2:
    return pose.transformTo(point)


class TestPose2(unittest.TestCase):

    def assertRot2Equals(self, expected: Rot2, actual: Rot2) -> None:
        if expected.equals(actual, 1e-9):
            return
        raise self.failureException(f"{expected} != {actual}")

    def assertPose2Equals(self, expected: Pose2, actual: Pose2) -> None:
        if expected.equals(actual, 1e-9):
            return
        raise self.failureException(f"{expected} != {actual}")

    def test_ExpmapDerivative1(self) -> None:
        actualH = np.zeros((3, 3), order="F")
        w = np.array([0.1, 0.27, -0.3])
        Pose2.Expmap(w, actualH)
        expectedH = numericalDerivative11Pose2Vector3(Pose2.Expmap, w, 1e-2)
        assert_almost_equal(expectedH, actualH, 5)

    def test_ExpmapDerivative2(self) -> None:
        actualH = np.zeros((3, 3), order="F")
        w0 = np.array([0.1, 0.27, 0.0])  # alpha = 0
        Pose2.Expmap(w0, actualH)
        expectedH = numericalDerivative11Pose2Vector3(Pose2.Expmap, w0, 1e-2)
        assert_almost_equal(expectedH, actualH, 5)

    def test_LogmapDerivative1(self) -> None:
        actualH = np.zeros((3, 3), order="F")
        w = np.array([0.1, 0.27, -0.3])
        p: Pose2 = Pose2.Expmap(w)
        assert_almost_equal(w, Pose2.Logmap(p, actualH), 5)
        expectedH = numericalDerivative11Vector3Pose2(Pose2.Logmap, p, 1e-2)
        assert_almost_equal(expectedH, actualH, 5)

    def test_LogmapDerivative2(self) -> None:
        actualH = np.zeros((3, 3), order="F")
        w0 = np.array([0.1, 0.27, 0.0])  # alpha = 0
        p: Pose2 = Pose2.Expmap(w0)
        assert_almost_equal(w0, Pose2.Logmap(p, actualH), 5)
        expectedH = numericalDerivative11Vector3Pose2(Pose2.Logmap, p, 1e-2)
        assert_almost_equal(expectedH, actualH, 5)

    def test_transformTo(self) -> None:
        pose = Pose2(math.pi / 2.0, Point2(1, 2))  # robot at (1,2) looking towards y
        point = Point2(-1, 4)  # landmark at (-1,4)

        # expected
        expected = Point2(2, 2)
        expectedH1 = np.array(
            [
                [-1.0, 0.0, 2.0],
                [0.0, -1.0, -2.0],
            ]
        )
        expectedH2 = np.array(
            [
                [0.0, 1.0],
                [-1.0, 0.0],
            ]
        )

        # actual
        actualH1 = np.zeros((2, 3), order="F")
        actualH2 = np.zeros((2, 2), order="F")
        actual: Point2 = pose.transformTo(point, actualH1, actualH2)
        assert_almost_equal(expected, actual, 5)

        assert_almost_equal(expectedH1, actualH1, 5)
        numericalH1 = numericalDerivative21Point2Pose2Point2(transformTo_, pose, point)
        assert_almost_equal(numericalH1, actualH1, 5)

        assert_almost_equal(expectedH2, actualH2, 5)
        numericalH2 = numericalDerivative22Point2Pose2Point2(transformTo_, pose, point)
        assert_almost_equal(numericalH2, actualH2, 5)

    def test_transformFrom(self) -> None:
        pose = Pose2(1.0, 0.0, math.pi / 2.0)
        pt = Point2(2.0, 1.0)
        H1 = np.zeros((2, 3), order="F")
        H2 = np.zeros((2, 2), order="F")
        actual: Point2 = pose.transformFrom(pt, H1, H2)

        expected = Point2(0.0, 2.0)
        assert_almost_equal(expected, actual, 5)

        H1_expected = np.array(
            [
                [0.0, -1.0, -2.0],
                [1.0, 0.0, -1.0],
            ]
        )
        H2_expected = np.array(
            [
                [0.0, -1.0],
                [1.0, 0.0],
            ]
        )

        numericalH1 = numericalDerivative21Point2Pose2Point2(transformFrom_, pose, pt)
        assert_almost_equal(H1_expected, H1, 5)
        assert_almost_equal(H1_expected, numericalH1, 5)

        numericalH2 = numericalDerivative22Point2Pose2Point2(transformFrom_, pose, pt)
        assert_almost_equal(H2_expected, H2, 5)
        assert_almost_equal(H2_expected, numericalH2, 5)

    def test_compose_a(self) -> None:

        pose1 = Pose2(math.pi / 4.0, Point2(math.sqrt(0.5), math.sqrt(0.5)))
        pose2 = Pose2(math.pi / 2.0, Point2(0.0, 2.0))

        actualDcompose1 = np.zeros((3, 3), order="F")
        actualDcompose2 = np.zeros((3, 3), order="F")
        actual: Pose2 = pose1.compose(pose2, actualDcompose1, actualDcompose2)

        expected = Pose2(
            3.0 * math.pi / 4.0, Point2(-math.sqrt(0.5), 3.0 * math.sqrt(0.5))
        )
        self.assertPose2Equals(expected, actual)

        expectedH1 = np.array(
            [
                [0.0, 1.0, 0.0],
                [-1.0, 0.0, 2.0],
                [0.0, 0.0, 1.0],
            ]
        )
        expectedH2 = np.eye(3)

        numericalH1 = numericalDerivative21Pose2Pose2Pose2(Pose2.compose, pose1, pose2)
        numericalH2 = numericalDerivative22Pose2Pose2Pose2(Pose2.compose, pose1, pose2)
        assert_almost_equal(expectedH1, actualDcompose1, 5)
        assert_almost_equal(numericalH1, actualDcompose1, 5)
        assert_almost_equal(expectedH2, actualDcompose2, 5)
        assert_almost_equal(numericalH2, actualDcompose2, 5)

        point = Point2(math.sqrt(0.5), 3.0 * math.sqrt(0.5))
        expected_point = Point2(-1.0, -1.0)
        actual_point1: Point2 = (pose1.compose(pose2)).transformTo(point)
        actual_point2: Point2 = pose2.transformTo(pose1.transformTo(point))
        assert_almost_equal(expected_point, actual_point1, 5)
        assert_almost_equal(expected_point, actual_point2, 5)

    def test_compose_b(self) -> None:
        pose1 = Pose2(Rot2.fromAngle(math.pi / 10.0), Point2(0.75, 0.5))
        pose2 = Pose2(
            Rot2.fromAngle(math.pi / 4.0 - math.pi / 10.0),
            Point2(0.701289620636, 1.34933052585),
        )

        pose_expected = Pose2(Rot2.fromAngle(math.pi / 4.0), Point2(1.0, 2.0))

        pose_actual_op: Pose2 = pose1.compose(pose2)
        actualDcompose1 = np.zeros((3, 3), order="F")
        actualDcompose2 = np.zeros((3, 3), order="F")
        pose_actual_fcn: Pose2 = pose1.compose(pose2, actualDcompose1, actualDcompose2)

        numericalH1 = numericalDerivative21Pose2Pose2Pose2(Pose2.compose, pose1, pose2)
        numericalH2 = numericalDerivative22Pose2Pose2Pose2(Pose2.compose, pose1, pose2)
        assert_almost_equal(numericalH1, actualDcompose1, 5)
        assert_almost_equal(numericalH2, actualDcompose2, 5)

        self.assertPose2Equals(pose_expected, pose_actual_op)
        self.assertPose2Equals(pose_expected, pose_actual_fcn)

    def test_compose_c(self) -> None:
        pose1 = Pose2(Rot2.fromAngle(math.pi / 4.0), Point2(1.0, 1.0))
        pose2 = Pose2(
            Rot2.fromAngle(math.pi / 4.0), Point2(math.sqrt(0.5), math.sqrt(0.5))
        )

        pose_expected = Pose2(Rot2.fromAngle(math.pi / 2.0), Point2(1.0, 2.0))

        pose_actual_op: Pose2 = pose1.compose(pose2)
        actualDcompose1 = np.zeros((3, 3), order="F")
        actualDcompose2 = np.zeros((3, 3), order="F")
        pose_actual_fcn: Pose2 = pose1.compose(pose2, actualDcompose1, actualDcompose2)

        numericalH1 = numericalDerivative21Pose2Pose2Pose2(Pose2.compose, pose1, pose2)
        numericalH2 = numericalDerivative22Pose2Pose2Pose2(Pose2.compose, pose1, pose2)
        assert_almost_equal(numericalH1, actualDcompose1, 5)
        assert_almost_equal(numericalH2, actualDcompose2, 5)

        self.assertPose2Equals(pose_expected, pose_actual_op)
        self.assertPose2Equals(pose_expected, pose_actual_fcn)

    def test_inverse(self) -> None:
        # origin = Point2(0, 0)
        t = Point2(1, 2)
        gTl = Pose2(math.pi / 2.0, t)  # robot at (1,2) looking towards y

        identity = Pose2.Identity()
        lTg = gTl.inverse()
        self.assertPose2Equals(identity, lTg.compose(gTl))
        self.assertPose2Equals(identity, gTl.compose(lTg))

        l = Point2(4, 5)
        g = Point2(-4, 6)
        assert_almost_equal(g, gTl.transformFrom(l), 5)
        assert_almost_equal(l, lTg.transformFrom(g), 5)

        # Check derivative
        numericalH = numericalDerivative11Pose2Pose2(Pose2.inverse, lTg)
        actualDinverse = np.zeros((3, 3), order="F")
        lTg.inverse(actualDinverse)
        assert_almost_equal(numericalH, actualDinverse, 5)

    def test_translation(self) -> None:
        pose = Pose2(3.5, -8.2, 4.2)

        actualH = np.zeros((2, 3), order="F")
        assert_almost_equal(np.array([3.5, -8.2]), pose.translation(actualH), 5)

        def f(T: Pose2) -> Point2:
            return T.translation()

        numericalH = numericalDerivative11Point2Pose2(f, pose)
        assert_almost_equal(numericalH, actualH, 6)

    def test_rotation(self) -> None:
        pose = Pose2(3.5, -8.2, 4.2)

        actualH = np.zeros((1, 3), order="F")
        self.assertRot2Equals(Rot2(4.2), pose.rotation(actualH))

        def f(T: Pose2) -> Rot2:
            return T.rotation()

        numericalH = numericalDerivative11(f, pose, 1, 3)
        assert_almost_equal(numericalH, actualH, 6)

    def test_between(self) -> None:
        gT1 = Pose2(math.pi / 2.0, Point2(1, 2))  # robot at (1,2) looking towards y
        gT2 = Pose2(math.pi, Point2(-1, 4))  # robot at (-1,4) looking at negative x

        actualH1 = np.zeros((3, 3), order="F")
        actualH2 = np.zeros((3, 3), order="F")

        expected = Pose2(math.pi / 2.0, Point2(2, 2))
        actual1: Pose2 = gT1.between(gT2)
        actual2: Pose2 = gT1.between(gT2, actualH1, actualH2)
        self.assertPose2Equals(expected, actual1)
        self.assertPose2Equals(expected, actual2)

        expectedH1 = np.array(
            [
                [0.0, -1.0, -2.0],
                [1.0, 0.0, -2.0],
                [0.0, 0.0, -1.0],
            ]
        )
        numericalH1 = numericalDerivative21Pose2Pose2Pose2(Pose2.between, gT1, gT2)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(numericalH1, actualH1, 5)
        # Assert H1 = -AdjointMap(between(p2,p1)) as in doc/math.lyx
        assert_almost_equal(-gT2.between(gT1).AdjointMap(), actualH1, 5)

        expectedH2 = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        numericalH2 = numericalDerivative22Pose2Pose2Pose2(Pose2.between, gT1, gT2)
        assert_almost_equal(expectedH2, actualH2, 5)
        assert_almost_equal(numericalH2, actualH2, 5)

    def test_between2(self) -> None:

        p2 = Pose2(math.pi / 2.0, Point2(1, 2))  # robot at (1,2) looking towards y
        p1 = Pose2(math.pi, Point2(-1, 4))  # robot at (-1,4) loooking at negative x

        actualH1 = np.zeros((3, 3), order="F")
        actualH2 = np.zeros((3, 3), order="F")
        p1.between(p2, actualH1, actualH2)
        numericalH1 = numericalDerivative21Pose2Pose2Pose2(Pose2.between, p1, p2)
        assert_almost_equal(numericalH1, actualH1, 5)
        numericalH2 = numericalDerivative22Pose2Pose2Pose2(Pose2.between, p1, p2)
        assert_almost_equal(numericalH2, actualH2, 5)

    # arbitrary, non perpendicular angles to be extra safe
    def test_between3(self):

        p2 = Pose2(math.pi / 3.0, Point2(1, 2))
        p1 = Pose2(math.pi / 6.0, Point2(-1, 4))

        actualH1 = np.zeros((3, 3), order="F")
        actualH2 = np.zeros((3, 3), order="F")

        p1.between(p2, actualH1, actualH2)
        numericalH1 = numericalDerivative21Pose2Pose2Pose2(Pose2.between, p1, p2)
        assert_almost_equal(numericalH1, actualH1, 5)
        numericalH2 = numericalDerivative22Pose2Pose2Pose2(Pose2.between, p1, p2)
        assert_almost_equal(numericalH2, actualH2, 5)

    def test_bearing(self) -> None:

        actualH1 = np.zeros((1, 3), order="F")
        actualH2 = np.zeros((1, 2), order="F")

        # establish bearing is indeed zero
        self.assertRot2Equals(Rot2(), x1.bearing(l1))

        # establish bearing is indeed 45 degrees
        self.assertRot2Equals(Rot2.fromAngle(math.pi / 4.0), x1.bearing(l2))

        # establish bearing is indeed 45 degrees even if shifted
        actual23: Rot2 = x2.bearing(l3, actualH1, actualH2)
        self.assertRot2Equals(Rot2.fromAngle(math.pi / 4.0), actual23)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21Rot2Pose2Point2(bearing_proxy, x2, l3)
        assert_almost_equal(expectedH1, actualH1, 5)
        expectedH2 = numericalDerivative22Rot2Pose2Point2(bearing_proxy, x2, l3)
        assert_almost_equal(expectedH2, actualH2, 5)

        # establish bearing is indeed 45 degrees even if rotated
        actual34: Rot2 = x3.bearing(l4, actualH1, actualH2)
        self.assertRot2Equals(Rot2.fromAngle(math.pi / 4.0), actual34)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21Rot2Pose2Point2(bearing_proxy, x3, l4)
        expectedH2 = numericalDerivative22Rot2Pose2Point2(bearing_proxy, x3, l4)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)

    def test_bearing_pose(self) -> None:

        xl1 = Pose2(1, 0, math.pi / 2.0)
        xl2 = Pose2(1, 1, math.pi)
        xl3 = Pose2(2.0, 2.0, -math.pi / 2.0)
        xl4 = Pose2(1, 3, 0)

        actualH1 = np.zeros((1, 3), order="F")
        actualH2 = np.zeros((1, 3), order="F")

        # establish bearing is indeed zero
        self.assertRot2Equals(Rot2(), x1.bearing(xl1))

        # establish bearing is indeed 45 degrees
        self.assertRot2Equals(Rot2.fromAngle(math.pi / 4.0), x1.bearing(xl2))

        # establish bearing is indeed 45 degrees even if shifted
        actual23: Rot2 = x2.bearing(xl3, actualH1, actualH2)
        self.assertRot2Equals(Rot2.fromAngle(math.pi / 4.0), actual23)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21Rot2Pose2Pose2(bearing_pose_proxy, x2, xl3)
        expectedH2 = numericalDerivative22Rot2Pose2Pose2(bearing_pose_proxy, x2, xl3)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)

        # establish bearing is indeed 45 degrees even if rotated
        actual34: Rot2 = x3.bearing(xl4, actualH1, actualH2)
        self.assertRot2Equals(Rot2.fromAngle(math.pi / 4.0), actual34)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21Rot2Pose2Pose2(bearing_pose_proxy, x3, xl4)
        expectedH2 = numericalDerivative22Rot2Pose2Pose2(bearing_pose_proxy, x3, xl4)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)

    def test_range(self) -> None:

        actualH1 = np.zeros((1, 3), order="F")
        actualH2 = np.zeros((1, 2), order="F")

        # establish range is indeed zero
        self.assertAlmostEqual(1, x1.range(l1), 9)

        # establish range is indeed 45 degrees
        self.assertAlmostEqual(math.sqrt(2.0), x1.range(l2), 9)

        # Another pair (x2 = Pose(1,1,0); l3 = Point(2,2))
        actual23: float = x2.range(l3, actualH1, actualH2)
        self.assertAlmostEqual(math.sqrt(2.0), actual23, 9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21DoublePose2Point2(range_proxy, x2, l3)
        expectedH2 = numericalDerivative22DoublePose2Point2(range_proxy, x2, l3)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)

        # Another test
        actual34: float = x3.range(l4, actualH1, actualH2)
        self.assertAlmostEqual(2, actual34, 9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21DoublePose2Point2(range_proxy, x3, l4)
        expectedH2 = numericalDerivative22DoublePose2Point2(range_proxy, x3, l4)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)

    def test_range_pose(self) -> None:

        xl1 = Pose2(1, 0, math.pi / 2.0)
        xl2 = Pose2(1, 1, math.pi)
        xl3 = Pose2(2.0, 2.0, -math.pi / 2.0)
        xl4 = Pose2(1, 3, 0)

        actualH1 = np.zeros((1, 3), order="F")
        actualH2 = np.zeros((1, 3), order="F")

        # establish range is indeed zero
        self.assertAlmostEqual(1, x1.range(xl1))

        # establish range is indeed 45 degrees
        self.assertAlmostEqual(math.sqrt(2.0), x1.range(xl2))

        # Another pair
        actual23: float = x2.range(xl3, actualH1, actualH2)
        self.assertAlmostEqual(math.sqrt(2.0), actual23)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21DoublePose2Pose2(range_pose_proxy, x2, xl3)
        expectedH2 = numericalDerivative22DoublePose2Pose2(range_pose_proxy, x2, xl3)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)

        # Another test
        actual34: float = x3.range(xl4, actualH1, actualH2)
        self.assertAlmostEqual(2, actual34)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21DoublePose2Pose2(range_pose_proxy, x3, xl4)
        expectedH2 = numericalDerivative22DoublePose2Pose2(range_pose_proxy, x3, xl4)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)


if __name__ == "__main__":
    unittest.main()
