# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0621
# really to test numeric differentation
# see testPinholeCamera.cpp

import unittest
import numpy as np
from gtsam import PinholeCameraCal3_S2, Cal3_S2  # type:ignore
from gtsam import Point2, Point3, Pose3, Rot3, Unit3  # type:ignore
from numpy.testing import assert_almost_equal

from numerical_derivative import numericalDerivative11
from numerical_derivative import numericalDerivative31Point2Pose3Point3Cal3_S2
from numerical_derivative import numericalDerivative32Point2Pose3Point3Cal3_S2
from numerical_derivative import numericalDerivative33Point2Pose3Point3Cal3_S2
from numerical_derivative import numericalDerivative31Point2Pose3Unit3Cal3_S2
from numerical_derivative import numericalDerivative32Point2Pose3Unit3Cal3_S2
from numerical_derivative import numericalDerivative33Point2Pose3Unit3Cal3_S2
from numerical_derivative import numericalDerivative21
from numerical_derivative import numericalDerivative22


K = Cal3_S2(625, 625, 0, 0, 0)

pose = Pose3(Rot3(np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])), Point3(0, 0, 0.5))
camera = PinholeCameraCal3_S2(pose, K)

pose1 = Pose3(Rot3(), Point3(0, 1, 0.5))
camera1 = PinholeCameraCal3_S2(pose1, K)

point1 = Point3(-0.08, -0.08, 0.0)
point2 = Point3(-0.08, 0.08, 0.0)
point3 = Point3(0.08, 0.08, 0.0)
point4 = Point3(0.08, -0.08, 0.0)

point1_inf = Unit3(np.array([-0.16, -0.16, -1.0]))
point2_inf = Unit3(np.array([-0.16, 0.16, -1.0]))
point3_inf = Unit3(np.array([0.16, 0.16, -1.0]))
point4_inf = Unit3(np.array([0.16, -0.16, -1.0]))


def project3(pose: Pose3, point: Point3, cal: Cal3_S2) -> Point2:
    return PinholeCameraCal3_S2(pose, cal).project(point)


def projectInfinity3(pose: Pose3, point3D: Unit3, cal: Cal3_S2) -> Point2:
    return PinholeCameraCal3_S2(pose, cal).project(point3D)


def project4(camera: PinholeCameraCal3_S2, point: Point3) -> Point2:
    return camera.project2(point)


def range0(camera: PinholeCameraCal3_S2, point: Point3) -> float:
    return camera.range(point)


def range1(camera: PinholeCameraCal3_S2, pose: Pose3) -> float:
    return camera.range(pose)


def distance3(a: Point3, b: Point3) -> float:
    return np.linalg.norm(a - b)  # type:ignore


class TestPinholeCamera(unittest.TestCase):

    def assertPose3Equals(self, expected: Pose3, actual: Pose3) -> None:
        if expected.equals(actual, 1e-5):
            return
        raise self.failureException(f"{expected} != {actual}")

    def test_Create(self) -> None:
        actualH1 = np.zeros((11, 6), order="F")
        actualH2 = np.zeros((11, 5), order="F")
        self.assertTrue(
            camera.equals(
                PinholeCameraCal3_S2.Create(pose, K, actualH1, actualH2), 1e-9
            )
        )

        # Check derivative
        def f(x1: Pose3, x2: Cal3_S2) -> PinholeCameraCal3_S2:
            return PinholeCameraCal3_S2.Create(
                x1, x2, np.zeros((11, 6), order="F"), np.zeros((11, 5), order="F")
            )

        numericalH1 = numericalDerivative21(f, pose, K, 11, 6)
        assert_almost_equal(numericalH1, actualH1)
        numericalH2 = numericalDerivative22(f, pose, K, 11, 5)
        assert_almost_equal(numericalH2, actualH2)

    def test_Pose(self) -> None:
        actualH = np.zeros((6, 11), order="F")
        self.assertPose3Equals(pose, camera.getPose(actualH))

        # Check derivative
        def f(x1: PinholeCameraCal3_S2) -> Pose3:
            return PinholeCameraCal3_S2.getPose(x1, np.zeros((6, 11), order="F"))

        numericalH = numericalDerivative11(f, camera, 6, 11)
        assert_almost_equal(numericalH, actualH)

    def test_Dproject(self) -> None:
        Dpose = np.zeros((2, 6), order="F")
        Dpoint = np.zeros((2, 3), order="F")
        Dcal = np.zeros((2, 5), order="F")
        result = camera.project(point1, Dpose, Dpoint, Dcal)
        numerical_pose = numericalDerivative31Point2Pose3Point3Cal3_S2(
            project3, pose, point1, K
        )
        Hexpected2 = numericalDerivative32Point2Pose3Point3Cal3_S2(
            project3, pose, point1, K
        )
        numerical_cal = numericalDerivative33Point2Pose3Point3Cal3_S2(
            project3, pose, point1, K
        )
        assert_almost_equal(Point2(-100, 100), result)
        assert_almost_equal(numerical_pose, Dpose)
        assert_almost_equal(Hexpected2, Dpoint)
        assert_almost_equal(numerical_cal, Dcal)

    def test_Dproject_Infinity(self) -> None:
        Dpose = np.zeros((2, 6), order="F")
        Dpoint = np.zeros((2, 2), order="F")
        Dcal = np.zeros((2, 5), order="F")
        point3D = Unit3(
            np.array([point1[0], point1[1], -10.0])
        )  # a point in front of the camera1

        # test Projection
        actual = camera.project(point3D, Dpose, Dpoint, Dcal)
        expected = Point2(-5.0, 5.0)
        assert_almost_equal(actual, expected)

        # test Jacobians
        numerical_pose = numericalDerivative31Point2Pose3Unit3Cal3_S2(
            projectInfinity3, pose, point3D, K
        )
        Hexpected2: np.ndarray = numericalDerivative32Point2Pose3Unit3Cal3_S2(
            projectInfinity3, pose, point3D, K
        )
        numerical_point2x2 = Hexpected2[
            :2, :2
        ]  # only the direction to the point matters
        numerical_cal = numericalDerivative33Point2Pose3Unit3Cal3_S2(
            projectInfinity3, pose, point3D, K
        )
        assert_almost_equal(numerical_pose, Dpose)
        assert_almost_equal(numerical_point2x2, Dpoint)
        assert_almost_equal(numerical_cal, Dcal)

    def test_Dproject2(self) -> None:
        Dcamera = np.zeros((2, 11), order="F")
        Dpoint = np.zeros((2, 3), order="F")
        result = camera.project2(point1, Dcamera, Dpoint)
        Hexpected1 = numericalDerivative21(
            project4, camera, point1, 2, 11
        )
        Hexpected2 = numericalDerivative22(
            project4, camera, point1, 2, 3
        )
        assert_almost_equal(result, Point2(-100, 100))
        assert_almost_equal(Hexpected1, Dcamera)
        assert_almost_equal(Hexpected2, Dpoint)

    # Add a test with more arbitrary rotation
    def test_Dproject3(self) -> None:
        pose1 = Pose3(Rot3.Ypr(0.1, -0.1, 0.4), Point3(0, 0, -10))
        camera = PinholeCameraCal3_S2(pose1)
        Dpose = np.zeros((2, 11), order="F")
        Dpoint = np.zeros((2, 3), order="F")
        camera.project2(point1, Dpose, Dpoint)
        numerical_pose = numericalDerivative21(
            project4, camera, point1, 2, 11
        )
        numerical_point = numericalDerivative22(
            project4, camera, point1, 2, 3
        )
        assert_almost_equal(numerical_pose, Dpose)
        assert_almost_equal(numerical_point, Dpoint)

    def test_range0(self) -> None:
        D1 = np.zeros((1, 11), order="F")
        D2 = np.zeros((1, 3), order="F")
        result = camera.range(point1, D1, D2)
        Hexpected1 = numericalDerivative21(
            range0, camera, point1, 1, 11
        )
        Hexpected2 = numericalDerivative22(
            range0, camera, point1, 1, 3
        )
        self.assertAlmostEqual(distance3(point1, camera.pose().translation()), result)
        assert_almost_equal(Hexpected1, D1)
        assert_almost_equal(Hexpected2, D2)

    def test_range1(self) -> None:
        D1 = np.zeros((1, 11), order="F")
        D2 = np.zeros((1, 6), order="F")
        result = camera.range(pose1, D1, D2)
        Hexpected1 = numericalDerivative21(
            range1, camera, pose1, 1, 11
        )
        Hexpected2 = numericalDerivative22(
            range1, camera, pose1, 1, 6
        )
        self.assertAlmostEqual(1, result)
        assert_almost_equal(Hexpected1, D1)
        assert_almost_equal(Hexpected2, D2)


if __name__ == "__main__":
    unittest.main()
