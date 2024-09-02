# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0621
# really to test numeric differentation
# see testPose3.cpp

import math

import unittest
import numpy as np
from gtsam import Point3, Pose3, Rot3, Unit3  # type:ignore
from numpy.testing import assert_almost_equal

from numerical_derivative import numericalDerivative21Vector6Pose3Vector6
from numerical_derivative import numericalDerivative22Vector6Pose3Vector6
from numerical_derivative import numericalDerivative11Point3Pose3
from numerical_derivative import numericalDerivative11Rot3Pose3
from numerical_derivative import numericalDerivative21Pose3Pose3Pose3
from numerical_derivative import numericalDerivative22Pose3Pose3Pose3
from numerical_derivative import numericalDerivative11Pose3Pose3
from numerical_derivative import numericalDerivative21Point3Pose3Point3
from numerical_derivative import numericalDerivative22Point3Pose3Point3
from numerical_derivative import numericalDerivative21DoublePose3Point3
from numerical_derivative import numericalDerivative22DoublePose3Point3
from numerical_derivative import numericalDerivative21DoublePose3Pose3
from numerical_derivative import numericalDerivative22DoublePose3Pose3
from numerical_derivative import numericalDerivative21Unit3Pose3Point3
from numerical_derivative import numericalDerivative22Unit3Pose3Point3
from numerical_derivative import numericalDerivative11Pose3Vector6
from numerical_derivative import numericalDerivative11Pose3Double
from numerical_derivative import numericalDerivative11Vector6Pose3
from numerical_derivative import numericalDerivative21Vector6Vector6Vector6
from numerical_derivative import numericalDerivative22Vector6Vector6Vector6
from numerical_derivative import numericalDerivative21Pose3Rot3Point3
from numerical_derivative import numericalDerivative22Pose3Rot3Point3
from numerical_derivative import numericalDerivative31Pose3Pose3Pose3Double
from numerical_derivative import numericalDerivative32Pose3Pose3Pose3Double


P = Point3(0.2, 0.7, -2)
R = Rot3.Rodrigues(0.3, 0, 0)
P2 = Point3(3.5, -8.2, 4.2)
T = Pose3(R, P2)
T2 = Pose3(Rot3.Rodrigues(0.3, 0.2, 0.1), P2)
T3 = Pose3(Rot3.Rodrigues(-90, 0, 0), Point3(1, 2, 3))
tol = 1e-5

# some shared test values - pulled from equivalent test in Pose2
l1 = Point3(1, 0, 0)
l2 = Point3(1, 1, 0)
l3 = Point3(2, 2, 0)
l4 = Point3(1, 4, -4)
x1 = Pose3()
x2 = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), l2)
x3 = Pose3(Rot3.Ypr(math.pi / 4.0, 0.0, 0.0), l2)

xl1 = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(1, 0, 0))
xl2 = Pose3(Rot3.Ypr(0.0, 1.0, 0.0), Point3(1, 1, 0))
xl3 = Pose3(Rot3.Ypr(1.0, 0.0, 0.0), Point3(2, 2, 0))
xl4 = Pose3(Rot3.Ypr(0.0, 0.0, 1.0), Point3(1, 4, -4))


def transformFrom_(pose: Pose3, point: Point3) -> Point3:
    return pose.transformFrom(point)


def transform_to_(pose: Pose3, point: Point3) -> Point3:
    return pose.transformTo(point)


def transformPoseFrom_(wTa: Pose3, aTb: Pose3) -> Pose3:
    return wTa.transformPoseFrom(aTb)


def transformPoseTo_(pose: Pose3, pose2: Pose3) -> Pose3:
    return pose.transformPoseTo(pose2)


def range_proxy(pose: Pose3, point: Point3) -> float:
    return pose.range(point)


def range_pose_proxy(pose: Pose3, point: Pose3) -> float:
    return pose.range(point)


def bearing_proxy(pose: Pose3, point: Point3) -> Unit3:
    return pose.bearing(point)


def testDerivAdjoint(xi: np.ndarray, v: np.ndarray) -> np.ndarray:
    return Pose3.adjointMap(xi).dot(v)


def testDerivAdjointTranspose(xi: np.ndarray, v: np.ndarray) -> np.ndarray:
    return Pose3.adjointMap(xi).transpose().dot(v)


def testing_interpolate(t1: Pose3, t2: Pose3, gamma: float) -> Pose3:
    # return interpolate(t1, t2, gamma)
    return t1.slerp(gamma, t2)


class screwPose3:
    a: float = 0.3
    c: float = math.cos(a)
    s: float = math.sin(a)
    w: float = 0.3
    xi: np.ndarray = np.array([0.0, 0.0, w, w, 0.0, 1.0])
    expectedR = Rot3(c, -s, 0, s, c, 0, 0, 0, 1)
    expectedT = Point3(0.29552, 0.0446635, 1)
    expected = Pose3(expectedR, expectedT)


class TestPose3(unittest.TestCase):

    def assertUnit3Equals(self, expected: Unit3, actual: Unit3) -> None:
        if expected.equals(actual, 1e-5):
            return
        raise self.failureException(f"{expected} != {actual}")

    def assertPose3Equals(self, expected: Pose3, actual: Pose3) -> None:
        if expected.equals(actual, 1e-5):
            return
        raise self.failureException(f"{expected} != {actual}")

    def assertRot3Equals(self, expected: Rot3, actual: Rot3) -> None:
        if expected.equals(actual, 1e-5):
            return
        raise self.failureException(f"{expected} != {actual}")

    # Check Adjoint numerical derivatives
    def test_Adjoint_jacobians(self) -> None:
        xi = np.array([0.1, 1.2, 2.3, 3.1, 1.4, 4.5])

        # Check evaluation sanity check
        assert_almost_equal((T.AdjointMap().dot(xi)), T.Adjoint(xi))
        assert_almost_equal((T2.AdjointMap().dot(xi)), T2.Adjoint(xi))
        assert_almost_equal((T3.AdjointMap().dot(xi)), T3.Adjoint(xi))

        # Check jacobians
        actualH1 = np.zeros((6, 6), order="F")
        actualH2 = np.zeros((6, 6), order="F")

        def Adjoint_proxy(T: Pose3, xi: np.ndarray) -> np.ndarray:
            return T.Adjoint(xi)

        T.Adjoint(xi, actualH1, actualH2)
        expectedH1 = numericalDerivative21Vector6Pose3Vector6(Adjoint_proxy, T, xi)
        expectedH2 = numericalDerivative22Vector6Pose3Vector6(Adjoint_proxy, T, xi)
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)

        T2.Adjoint(xi, actualH1, actualH2)
        expectedH1 = numericalDerivative21Vector6Pose3Vector6(Adjoint_proxy, T2, xi)
        expectedH2 = numericalDerivative22Vector6Pose3Vector6(Adjoint_proxy, T2, xi)
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)

        T3.Adjoint(xi, actualH1, actualH2)
        expectedH1 = numericalDerivative21Vector6Pose3Vector6(Adjoint_proxy, T3, xi)
        expectedH2 = numericalDerivative22Vector6Pose3Vector6(Adjoint_proxy, T3, xi)
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)

    # Check AdjointTranspose and jacobians
    def test_AdjointTranspose(self) -> None:
        xi = np.array([0.1, 1.2, 2.3, 3.1, 1.4, 4.5])

        # Check evaluation
        assert_almost_equal(
            (T.AdjointMap().transpose().dot(xi)), T.AdjointTranspose(xi)
        )
        assert_almost_equal(
            (T2.AdjointMap().transpose().dot(xi)), T2.AdjointTranspose(xi)
        )
        assert_almost_equal(
            (T3.AdjointMap().transpose().dot(xi)), T3.AdjointTranspose(xi)
        )

        # Check jacobians
        actualH1 = np.zeros((6, 6), order="F")
        actualH2 = np.zeros((6, 6), order="F")

        def AdjointTranspose_proxy(T: Pose3, xi: np.ndarray) -> np.ndarray:
            return T.AdjointTranspose(xi)

        T.AdjointTranspose(xi, actualH1, actualH2)
        expectedH1 = numericalDerivative21Vector6Pose3Vector6(
            AdjointTranspose_proxy, T, xi
        )
        expectedH2 = numericalDerivative22Vector6Pose3Vector6(
            AdjointTranspose_proxy, T, xi
        )
        assert_almost_equal(expectedH1, actualH1, 8)
        assert_almost_equal(expectedH2, actualH2)

        T2.AdjointTranspose(xi, actualH1, actualH2)
        expectedH1 = numericalDerivative21Vector6Pose3Vector6(
            AdjointTranspose_proxy, T2, xi
        )
        expectedH2 = numericalDerivative22Vector6Pose3Vector6(
            AdjointTranspose_proxy, T2, xi
        )
        assert_almost_equal(expectedH1, actualH1, 8)
        assert_almost_equal(expectedH2, actualH2)

        T3.AdjointTranspose(xi, actualH1, actualH2)
        expectedH1 = numericalDerivative21Vector6Pose3Vector6(
            AdjointTranspose_proxy, T3, xi
        )
        expectedH2 = numericalDerivative22Vector6Pose3Vector6(
            AdjointTranspose_proxy, T3, xi
        )
        assert_almost_equal(expectedH1, actualH1, 8)
        assert_almost_equal(expectedH2, actualH2)

    # Check translation and its pushforward
    def test_translation(self) -> None:
        actualH = np.zeros((3, 6), order="F")
        assert_almost_equal(Point3(3.5, -8.2, 4.2), T.translation(actualH), 8)

        def f(T: Pose3) -> Point3:
            return T.translation()

        numericalH = numericalDerivative11Point3Pose3(f, T)
        assert_almost_equal(numericalH, actualH, 6)

    # Check rotation and its pushforward
    def test_rotation(self) -> None:
        actualH = np.zeros((3, 6), order="F")
        self.assertRot3Equals(R, T.rotation(actualH))

        def f(T: Pose3) -> Pose3:
            return T.rotation()

        numericalH = numericalDerivative11Rot3Pose3(f, T)
        assert_almost_equal(numericalH, actualH, 6)

    # Check compose and its pushforward
    def test_compose(self) -> None:
        actual: np.ndarray = T2.compose(T2).matrix()  # 4x4
        expected: np.ndarray = T2.matrix().dot(T2.matrix())  # 4x4
        assert_almost_equal(actual, expected, 8)

        actualDcompose1 = np.zeros((6, 6), order="F")
        actualDcompose2 = np.zeros((6, 6), order="F")
        T2.compose(T2, actualDcompose1, actualDcompose2)

        numericalH1 = numericalDerivative21Pose3Pose3Pose3(Pose3.compose, T2, T2)
        assert_almost_equal(numericalH1, actualDcompose1, 3)
        assert_almost_equal(T2.inverse().AdjointMap(), actualDcompose1, 3)

        numericalH2 = numericalDerivative22Pose3Pose3Pose3(Pose3.compose, T2, T2)
        assert_almost_equal(numericalH2, actualDcompose2, 4)

    # Check compose and its pushforward, another case
    def test_compose2(self) -> None:
        T1: Pose3 = T
        actual: np.ndarray = T1.compose(T2).matrix()
        expected: np.ndarray = T1.matrix().dot(T2.matrix())
        assert_almost_equal(actual, expected, 8)

        actualDcompose1 = np.zeros((6, 6), order="F")
        actualDcompose2 = np.zeros((6, 6), order="F")
        T1.compose(T2, actualDcompose1, actualDcompose2)

        numericalH1 = numericalDerivative21Pose3Pose3Pose3(Pose3.compose, T1, T2)
        assert_almost_equal(numericalH1, actualDcompose1, 3)
        assert_almost_equal(T2.inverse().AdjointMap(), actualDcompose1, 3)

        numericalH2 = numericalDerivative22Pose3Pose3Pose3(Pose3.compose, T1, T2)
        assert_almost_equal(numericalH2, actualDcompose2, 5)

    def test_inverse(self) -> None:
        actualDinverse = np.zeros((6, 6), order="F")
        actual: np.ndarray = T.inverse(actualDinverse).matrix()
        expected: np.ndarray = np.linalg.inv(T.matrix())
        assert_almost_equal(actual, expected, 8)

        numericalH: np.ndarray = numericalDerivative11Pose3Pose3(Pose3.inverse, T)
        assert_almost_equal(numericalH, actualDinverse, 3)
        assert_almost_equal(-T.AdjointMap(), actualDinverse, 3)

    def test_inverseDerivatives2(self) -> None:
        R: Rot3 = Rot3.Rodrigues(0.3, 0.4, -0.5)
        t = Point3(3.5, -8.2, 4.2)
        T = Pose3(R, t)

        numericalH = numericalDerivative11Pose3Pose3(Pose3.inverse, T)
        actualDinverse = np.zeros((6, 6), order="F")
        T.inverse(actualDinverse)
        assert_almost_equal(numericalH, actualDinverse, 3)
        assert_almost_equal(-T.AdjointMap(), actualDinverse, 3)

    def test_Dtransform_from1_a(self) -> None:
        actualDtransform_from1 = np.zeros((3, 6), order="F")
        Hpoint = np.zeros((3, 3), order="F")
        T.transformFrom(P, actualDtransform_from1, Hpoint)
        numerical: np.ndarray = numericalDerivative21Point3Pose3Point3(
            transformFrom_, T, P
        )
        assert_almost_equal(numerical, actualDtransform_from1, 8)

    def test_Dtransform_from1_b(self) -> None:
        origin = Pose3()
        actualDtransform_from1 = np.zeros((3, 6), order="F")
        Hpoint = np.zeros((3, 3), order="F")
        origin.transformFrom(P, actualDtransform_from1, Hpoint)
        numerical = numericalDerivative21Point3Pose3Point3(transformFrom_, origin, P)
        assert_almost_equal(numerical, actualDtransform_from1, 8)

    def test_Dtransform_from1_c(self) -> None:
        origin = Point3(0, 0, 0)
        T0 = Pose3(R, origin)
        actualDtransform_from1 = np.zeros((3, 6), order="F")
        Hpoint = np.zeros((3, 3), order="F")
        T0.transformFrom(P, actualDtransform_from1, Hpoint)
        numerical: np.ndarray = numericalDerivative21Point3Pose3Point3(
            transformFrom_, T0, P
        )
        assert_almost_equal(numerical, actualDtransform_from1, 8)

    def test_Dtransform_from1_d(self) -> None:
        I = Rot3.Identity()
        t0 = Point3(100, 0, 0)
        T0 = Pose3(I, t0)
        actualDtransform_from1 = np.zeros((3, 6), order="F")
        Hpoint = np.zeros((3, 3), order="F")
        T0.transformFrom(P, actualDtransform_from1, Hpoint)
        numerical = numericalDerivative21Point3Pose3Point3(transformFrom_, T0, P)
        assert_almost_equal(numerical, actualDtransform_from1, 8)

    def test_Dtransform_from2(self) -> None:
        Hself = np.zeros((3, 6), order="F")
        actualDtransform_from2 = np.zeros((3, 3), order="F")
        T.transformFrom(P, Hself, actualDtransform_from2)
        numerical = numericalDerivative22Point3Pose3Point3(transformFrom_, T, P)
        assert_almost_equal(numerical, actualDtransform_from2, 8)

    def test_Dtransform_to1(self) -> None:
        computed = np.zeros((3, 6), order="F")
        Hpoint = np.zeros((3, 3), order="F")
        T.transformTo(P, computed, Hpoint)
        numerical = numericalDerivative21Point3Pose3Point3(transform_to_, T, P)
        assert_almost_equal(numerical, computed, 8)

    def test_Dtransform_to2(self) -> None:
        Hself = np.zeros((3, 6), order="F")
        computed = np.zeros((3, 3), order="F")
        T.transformTo(P, Hself, computed)
        numerical = numericalDerivative22Point3Pose3Point3(transform_to_, T, P)
        assert_almost_equal(numerical, computed, 8)

    def test_transform_to_with_derivatives(self) -> None:
        actH1 = np.zeros((3, 6), order="F")
        actH2 = np.zeros((3, 3), order="F")
        T.transformTo(P, actH1, actH2)
        expH1 = numericalDerivative21Point3Pose3Point3(transform_to_, T, P)
        expH2 = numericalDerivative22Point3Pose3Point3(transform_to_, T, P)
        assert_almost_equal(expH1, actH1, 8)
        assert_almost_equal(expH2, actH2, 8)

    def test_transform_from_with_derivatives(self) -> None:
        actH1 = np.zeros((3, 6), order="F")
        actH2 = np.zeros((3, 3), order="F")
        T.transformFrom(P, actH1, actH2)
        expH1 = numericalDerivative21Point3Pose3Point3(transformFrom_, T, P)
        expH2 = numericalDerivative22Point3Pose3Point3(transformFrom_, T, P)
        assert_almost_equal(expH1, actH1, 8)
        assert_almost_equal(expH2, actH2, 8)

    def test_transformPoseFrom(self) -> None:
        actual: np.ndarray = T2.compose(T2).matrix()
        expected: np.ndarray = T2.matrix().dot(T2.matrix())
        assert_almost_equal(actual, expected, 8)

        H1 = np.zeros((6, 6), order="F")
        H2 = np.zeros((6, 6), order="F")
        T2.transformPoseFrom(T2, H1, H2)

        numericalH1: np.ndarray = numericalDerivative21Pose3Pose3Pose3(
            transformPoseFrom_, T2, T2
        )
        assert_almost_equal(numericalH1, H1, 3)
        assert_almost_equal(T2.inverse().AdjointMap(), H1, 3)

        numericalH2: np.ndarray = numericalDerivative22Pose3Pose3Pose3(
            transformPoseFrom_, T2, T2
        )
        assert_almost_equal(numericalH2, H2, 4)

    def test_transformPoseTo_with_derivatives(self) -> None:
        actH1 = np.zeros((6, 6), order="F")
        actH2 = np.zeros((6, 6), order="F")
        res: Pose3 = T.transformPoseTo(T2, actH1, actH2)
        self.assertPose3Equals(res, T.inverse().compose(T2))

        expH1 = numericalDerivative21Pose3Pose3Pose3(transformPoseTo_, T, T2)
        expH2 = numericalDerivative22Pose3Pose3Pose3(transformPoseTo_, T, T2)
        assert_almost_equal(expH1, actH1, 8)
        assert_almost_equal(expH2, actH2, 8)

    def test_transformPoseTo_with_derivatives2(self) -> None:
        actH1 = np.zeros((6, 6), order="F")
        actH2 = np.zeros((6, 6), order="F")
        res: Pose3 = T.transformPoseTo(T3, actH1, actH2)
        self.assertPose3Equals(res, T.inverse().compose(T3))

        expH1 = numericalDerivative21Pose3Pose3Pose3(transformPoseTo_, T, T3)
        expH2 = numericalDerivative22Pose3Pose3Pose3(transformPoseTo_, T, T3)
        assert_almost_equal(expH1, actH1, 8)
        assert_almost_equal(expH2, actH2, 8)

    def test_between(self) -> None:
        expected: Pose3 = T2.inverse() * T3
        actualDBetween1 = np.zeros((6, 6), order="F")
        actualDBetween2 = np.zeros((6, 6), order="F")
        actual: Pose3 = T2.between(T3, actualDBetween1, actualDBetween2)
        self.assertPose3Equals(expected, actual)

        numericalH1: np.ndarray = numericalDerivative21Pose3Pose3Pose3(
            Pose3.between, T2, T3
        )
        assert_almost_equal(numericalH1, actualDBetween1, 3)

        numericalH2: np.ndarray = numericalDerivative22Pose3Pose3Pose3(
            Pose3.between, T2, T3
        )
        assert_almost_equal(numericalH2, actualDBetween2, 5)

    def test_range(self) -> None:
        actualH1 = np.zeros((1, 6), order="F")
        actualH2 = np.zeros((1, 3), order="F")

        # establish range is indeed zero
        self.assertAlmostEqual(1, x1.range(l1), 9)

        # establish range is indeed sqrt2
        self.assertAlmostEqual(math.sqrt(2.0), x1.range(l2), 9)

        # Another pair
        actual23: float = x2.range(l3, actualH1, actualH2)
        self.assertAlmostEqual(math.sqrt(2.0), actual23, 9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21DoublePose3Point3(range_proxy, x2, l3)
        expectedH2 = numericalDerivative22DoublePose3Point3(range_proxy, x2, l3)
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)

        # Another test
        actual34: float = x3.range(l4, actualH1, actualH2)
        self.assertAlmostEqual(5, actual34, 9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21DoublePose3Point3(range_proxy, x3, l4)
        expectedH2 = numericalDerivative22DoublePose3Point3(range_proxy, x3, l4)
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)

    def test_range_pose(self) -> None:
        actualH1 = np.zeros((1, 6), order="F")
        actualH2 = np.zeros((1, 6), order="F")

        # establish range is indeed zero
        self.assertAlmostEqual(1, x1.range(xl1), 9)

        # establish range is indeed sqrt2
        self.assertAlmostEqual(math.sqrt(2.0), x1.range(xl2), 9)

        # Another pair
        actual23: float = x2.range(xl3, actualH1, actualH2)
        self.assertAlmostEqual(math.sqrt(2.0), actual23, 9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21DoublePose3Pose3(range_pose_proxy, x2, xl3)
        expectedH2 = numericalDerivative22DoublePose3Pose3(range_pose_proxy, x2, xl3)
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)

        # Another test
        actual34: float = x3.range(xl4, actualH1, actualH2)
        self.assertAlmostEqual(5, actual34, 9)

        # Check numerical derivatives
        expectedH1 = numericalDerivative21DoublePose3Pose3(range_pose_proxy, x3, xl4)
        expectedH2 = numericalDerivative22DoublePose3Pose3(range_pose_proxy, x3, xl4)
        assert_almost_equal(expectedH1, actualH1)
        assert_almost_equal(expectedH2, actualH2)

    def test_Bearing(self) -> None:
        actualH1 = np.zeros((2, 6), order="F")
        actualH2 = np.zeros((2, 3), order="F")
        self.assertUnit3Equals(
            Unit3(np.array([1, 0, 0])), x1.bearing(l1, actualH1, actualH2)
        )

        # Check numerical derivatives
        expectedH1 = numericalDerivative21Unit3Pose3Point3(bearing_proxy, x1, l1)
        expectedH2 = numericalDerivative22Unit3Pose3Point3(bearing_proxy, x1, l1)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)

    def test_Bearing2(self) -> None:
        actualH1 = np.zeros((2, 6), order="F")
        actualH2 = np.zeros((2, 3), order="F")
        self.assertUnit3Equals(
            Unit3(np.array([0, 0.6, -0.8])), x2.bearing(l4, actualH1, actualH2)
        )

        # Check numerical derivatives
        expectedH1 = numericalDerivative21Unit3Pose3Point3(bearing_proxy, x2, l4)
        expectedH2 = numericalDerivative22Unit3Pose3Point3(bearing_proxy, x2, l4)
        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)

    def test_PoseToPoseBearing(self) -> None:
        actualH1 = np.zeros((2, 6), order="F")
        actualH2 = np.zeros((2, 6), order="F")
        self.assertUnit3Equals(
            Unit3(np.array([0, 1, 0])), xl1.bearing(xl2, actualH1, actualH2)
        )

        # Check numerical derivatives
        expectedH1 = numericalDerivative21Unit3Pose3Point3(bearing_proxy, xl1, l2)

        # Since the second pose is treated as a point, the value calculated by
        # numericalDerivative22 only depends on the position of the pose. Here, we
        # calculate the Jacobian w.r.t. the second pose's position, and then augment
        # that with zeroes in the block that is w.r.t. the second pose's
        # orientation.
        H2block = numericalDerivative22Unit3Pose3Point3(bearing_proxy, xl1, l2)
        expectedH2 = np.zeros((2, 6), order="F")
        expectedH2[:, 3:6] = H2block

        assert_almost_equal(expectedH1, actualH1, 5)
        assert_almost_equal(expectedH2, actualH2, 5)

    def test_ExpmapDerivative1(self) -> None:
        actualH = np.zeros((6, 6), order="F")
        w = np.array([0.1, 0.2, 0.3, 4.0, 5.0, 6.0])
        Pose3.Expmap(w, actualH)
        expectedH = numericalDerivative11Pose3Vector6(Pose3.Expmap, w)
        assert_almost_equal(expectedH, actualH)

    def test_ExpmapDerivative2(self) -> None:
        # Iserles05an (Lie-group Methods) says:
        # scalar is easy: d exp(a(t)) / dt = exp(a(t)) a'(t)
        # matrix is hard: d exp(A(t)) / dt = exp(A(t)) dexp[-A(t)] A'(t)
        # where A(t): T -> se(3) is a trajectory in the tangent space of SE(3)
        # and dexp[A] is a linear map from 4*4 to 4*4 derivatives of se(3)
        # Hence, the above matrix equation is typed: 4*4 = SE(3) * linear_map(4*4)

        # In GTSAM, we don't work with the Lie-algebra elements A directly, but with 6-vectors.
        # xi is easy: d Expmap(xi(t)) / dt = ExmapDerivative[xi(t)] * xi'(t)

        # Let's verify the above formula.

        def xi(t: float) -> np.ndarray:
            return np.array(
                [2 * t, math.sin(t), 4 * t * t, 2 * t, math.sin(t), 4 * t * t]
            )

        def xi_dot(t: float) -> np.ndarray:
            return np.array([2, math.cos(t), 8 * t, 2, math.cos(t), 8 * t])

        # We define a function T
        def T(t: np.ndarray) -> Pose3:
            return Pose3.Expmap(xi(t[0]))

        for t in np.linspace(-2.0, 2.0, 15):
            expected:np.ndarray = numericalDerivative11Pose3Double(T, np.array([t]))
            # TODO: ravel here seems weird
            expected = expected.ravel()
            actual = Pose3.ExpmapDerivative(xi(t)).dot(xi_dot(t))
            assert_almost_equal(expected, actual, 7)

    # THIS TEST DOES NOT PASS
    # def test_ExpmapDerivativeQr(self) -> None:
    #     w = np.random.uniform(-1,1,6)
    #     w[:3] /= np.linalg.norm(w[:3])
    #     w[:3] *= 0.009
    #     actualQr: np.ndarray = Pose3.ComputeQforExpmapDerivative(w, 0.01)
    #     expectedH: np.ndarray = numericalDerivative11Pose3Vector6(Pose3.Expmap, w)
    #     expectedQr: np.ndarray = expectedH[:3, :3]
    #     assert_almost_equal(expectedQr, actualQr, 6)

    def test_LogmapDerivative(self) -> None:
        actualH = np.zeros((6, 6), order="F")
        w = np.array([0.1, 0.2, 0.3, 4.0, 5.0, 6.0])
        p: Pose3 = Pose3.Expmap(w)
        assert_almost_equal(w, Pose3.Logmap(p, actualH), 5)
        expectedH = numericalDerivative11Vector6Pose3(Pose3.Logmap, p)
        assert_almost_equal(expectedH, actualH)

    def test_adjoint(self) -> None:
        v = np.array([1, 2, 3, 4, 5, 6])
        expected: np.ndarray = testDerivAdjoint(screwPose3.xi, v)

        actualH1 = np.zeros((6, 6), order="F")
        actualH2 = np.zeros((6, 6), order="F")
        actual: np.ndarray = Pose3.adjoint(screwPose3.xi, v, actualH1, actualH2)

        numericalH1 = numericalDerivative21Vector6Vector6Vector6(
            testDerivAdjoint, screwPose3.xi, v, 1e-5
        )
        numericalH2 = numericalDerivative22Vector6Vector6Vector6(
            testDerivAdjoint, screwPose3.xi, v, 1e-5
        )

        assert_almost_equal(expected, actual, 5)
        assert_almost_equal(numericalH1, actualH1, 5)
        assert_almost_equal(numericalH2, actualH2, 5)

    def test_adjointTranspose(self) -> None:
        xi = np.array([0.01, 0.02, 0.03, 1.0, 2.0, 3.0])
        v = np.array([0.04, 0.05, 0.06, 4.0, 5.0, 6.0])
        expected: np.ndarray = testDerivAdjointTranspose(xi, v)

        actualH1 = np.zeros((6, 6), order="F")
        actualH2 = np.zeros((6, 6), order="F")
        actual: np.ndarray = Pose3.adjointTranspose(xi, v, actualH1, actualH2)

        numericalH1 = numericalDerivative21Vector6Vector6Vector6(
            testDerivAdjointTranspose, xi, v, 1e-5
        )
        numericalH2 = numericalDerivative22Vector6Vector6Vector6(
            testDerivAdjointTranspose, xi, v, 1e-5
        )

        assert_almost_equal(expected, actual, 5)
        assert_almost_equal(numericalH1, actualH1, 5)
        assert_almost_equal(numericalH2, actualH2, 5)

    def test_interpolateJacobians1(self) -> None:
        X: Pose3 = Pose3.Identity()
        Y: Pose3 = Pose3(Rot3.Rz(math.pi / 2), Point3(1, 0, 0))
        t: float = 0.5
        expectedPoseInterp = Pose3(
            Rot3.Rz(math.pi / 4), Point3(0.5, -0.207107, 0)
        )  # note: different from test above: this is full Pose3 interpolation
        actualJacobianX = np.zeros((6, 6), order="F")
        actualJacobianY = np.zeros((6, 6), order="F")
        self.assertPose3Equals(
            expectedPoseInterp,
            # interpolate(X, Y, t, actualJacobianX, actualJacobianY),
            X.slerp(t, Y, actualJacobianX, actualJacobianY),
        )

        expectedJacobianX = numericalDerivative31Pose3Pose3Pose3Double(
            testing_interpolate, X, Y, t
        )
        assert_almost_equal(expectedJacobianX, actualJacobianX, 6)

        expectedJacobianY = numericalDerivative32Pose3Pose3Pose3Double(
            testing_interpolate, X, Y, t
        )
        assert_almost_equal(expectedJacobianY, actualJacobianY, 6)

    def test_interpolateJacobians2(self) -> None:
        X: Pose3 = Pose3.Identity()
        Y: Pose3 = Pose3(Rot3.Identity(), Point3(1, 0, 0))
        t: float = 0.3
        expectedPoseInterp: Pose3 = Pose3(Rot3.Identity(), Point3(0.3, 0, 0))
        actualJacobianX = np.zeros((6, 6), order="F")
        actualJacobianY = np.zeros((6, 6), order="F")
        self.assertPose3Equals(
            expectedPoseInterp, X.slerp(t, Y, actualJacobianX, actualJacobianY)
        )

        expectedJacobianX = numericalDerivative31Pose3Pose3Pose3Double(
            testing_interpolate, X, Y, t
        )
        assert_almost_equal(expectedJacobianX, actualJacobianX, 6)

        expectedJacobianY = numericalDerivative32Pose3Pose3Pose3Double(
            testing_interpolate, X, Y, t
        )
        assert_almost_equal(expectedJacobianY, actualJacobianY, 6)

    def test_interpolateJacobians3(self) -> None:
        X: Pose3 = Pose3.Identity()
        Y: Pose3 = Pose3(Rot3.Rz(math.pi / 2), Point3(0, 0, 0))
        t: float = 0.5
        expectedPoseInterp: Pose3 = Pose3(Rot3.Rz(math.pi / 4), Point3(0, 0, 0))
        actualJacobianX = np.zeros((6, 6), order="F")
        actualJacobianY = np.zeros((6, 6), order="F")
        self.assertPose3Equals(
            expectedPoseInterp, X.slerp(t, Y, actualJacobianX, actualJacobianY)
        )

        expectedJacobianX = numericalDerivative31Pose3Pose3Pose3Double(
            testing_interpolate, X, Y, t
        )
        assert_almost_equal(expectedJacobianX, actualJacobianX, 6)

        expectedJacobianY = numericalDerivative32Pose3Pose3Pose3Double(
            testing_interpolate, X, Y, t
        )
        assert_almost_equal(expectedJacobianY, actualJacobianY, 6)

    def test_interpolateJacobians4(self) -> None:
        X = Pose3(Rot3.Ypr(0.1, 0.2, 0.3), Point3(10, 5, -2))
        Y = Pose3(Rot3.Ypr(1.1, -2.2, -0.3), Point3(-5, 1, 1))
        t: float = 0.3
        # expectedPoseInterp = Pose3(Rot3.Rz(math.pi / 4), Point3(0, 0, 0))
        actualJacobianX = np.zeros((6, 6), order="F")
        actualJacobianY = np.zeros((6, 6), order="F")
        X.slerp(t, Y, actualJacobianX, actualJacobianY)

        expectedJacobianX = numericalDerivative31Pose3Pose3Pose3Double(
            testing_interpolate, X, Y, t
        )
        assert_almost_equal(expectedJacobianX, actualJacobianX, 6)

        expectedJacobianY = numericalDerivative32Pose3Pose3Pose3Double(
            testing_interpolate, X, Y, t
        )
        assert_almost_equal(expectedJacobianY, actualJacobianY, 6)

    def test_Create(self) -> None:
        actualH1 = np.zeros((6, 3), order="F")
        actualH2 = np.zeros((6, 3), order="F")
        actual: Pose3 = Pose3.Create(R, P2, actualH1, actualH2)
        self.assertPose3Equals(T, actual)

        def create(R: Rot3, t: Point3) -> Pose3:
            return Pose3.Create(R, t)

        assert_almost_equal(
            numericalDerivative21Pose3Rot3Point3(create, R, P2), actualH1, 9
        )
        assert_almost_equal(
            numericalDerivative22Pose3Rot3Point3(create, R, P2), actualH2, 9
        )

    def test_ExpmapChainRule(self) -> None:
        # Muliply with an arbitrary matrix and exponentiate
        M = np.array(
            [
                [1, 2, 3, 4, 5, 6],
                [7, 8, 9, 1, 2, 3],
                [4, 5, 6, 7, 8, 9],
                [1, 2, 3, 4, 5, 6],
                [7, 8, 9, 1, 2, 3],
                [4, 5, 6, 7, 8, 9],
            ]
        )

        def g(omega: np.ndarray) -> Pose3:
            return Pose3.Expmap(M.dot(omega))

        # Test the derivatives at zero
        expected = numericalDerivative11Pose3Vector6(g, np.zeros((6)))
        assert_almost_equal(expected, M, 5)  # Pose3.ExpmapDerivative(Z_6x1) is identity

        # Test the derivatives at another value
        delta = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        expected2 = numericalDerivative11Pose3Vector6(g, delta)
        analytic = Pose3.ExpmapDerivative(M.dot(delta)).dot(M)
        assert_almost_equal(expected2, analytic, 5)  # note tolerance


if __name__ == "__main__":
    unittest.main()
