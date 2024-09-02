# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0621
# really to test numeric differentation
# see testPinholeCamera.cpp

import math

import unittest
import numpy as np
from gtsam import CalibratedCamera,PinholeCameraCal3_S2,Cal3_S2,Cal3Bundler, PinholeCameraCal3Bundler,Point2, Point3, Pose3, Rot3, Unit3, Vector3 # type:ignore
from numpy.testing import assert_almost_equal








K = Cal3_S2(625, 625, 0, 0, 0)

pose = Pose3(Rot3(Vector3(1, -1, -1).asDiagonal()), Point3(0, 0, 0.5))
camera = PinholeCameraCal3_S2 (pose, K)

pose1 = Pose3 (Rot3(),Point3(0, 1, 0.5))
camera1 = PinholeCameraCal3_S2(pose1, K)

point1 = Point3(-0.08,-0.08, 0.0)
point2 = Point3(-0.08, 0.08, 0.0)
point3 = Point3( 0.08, 0.08, 0.0)
point4= Point3( 0.08,-0.08, 0.0)

point1_inf = Unit3(-0.16,-0.16, -1.0)
point2_inf = Unit3(-0.16, 0.16, -1.0)
point3_inf = Unit3( 0.16, 0.16, -1.0)
point4_inf = Unit3( 0.16,-0.16, -1.0)

def  project3(pose: Pose3,   point: Point3,   cal: Cal3_S2) -> Point2:
    return PinholeCameraCal3_S2(pose,cal).project(point)

def  projectInfinity3( pose:  Pose3,   point3D: Unit3,   cal: Cal3_S2) -> Point2:
    return PinholeCameraCal3_S2(pose,cal).project(point3D)

def  project4( camera: PinholeCameraCal3_S2,   point: Point3) -> Point2:
    return camera.project2(point)

def  range0(  camera: PinholeCameraCal3_S2,   point: Point3) -> float:
    return camera.range(point)

def range1( camera:  PinholeCameraCal3_S2,  pose:  Pose3) -> float:
    return camera.range(pose)


K2 = Cal3Bundler(625, 1e-3, 1e-3)
camera2 = PinholeCameraCal3Bundler(pose1, K2)
def range2(camera: PinholeCameraCal3_S2,  camera2: PinholeCameraCal3Bundler) -> float:
    return camera.range<Cal3Bundler>(camera2)


camera3 = CalibratedCamera(pose1)
def range3( camera: PinholeCameraCal3_S2, camera3: CalibratedCamera) -> float:
    return camera.range(camera3)


class TestPinholeCamera(unittest.TestCase):

    def assertPose3Equals(self, expected: Pose3, actual: Pose3) -> None:
        if expected.equals(actual, 1e-5):
            return
        raise self.failureException(f"{expected} != {actual}")

    def test_Create(self) -> None:
        actualH1 = np.zeros((1,1), order='F')
        actualH2 = np.zeros((1,1), order='F')
        self.assertTrue(camera.equals(PinholeCameraCal3_S2.Create(pose,K, actualH1, actualH2)))

        # Check derivative
        std::function<Camera(Pose3, Cal3_S2)> f =  #
            std::bind(Camera::Create, std::placeholders::_1, std::placeholders::_2,
                    nullptr, nullptr)
        numericalH1 = numericalDerivative21<Camera,Pose3,Cal3_S2>(f,pose,K)
        assert_almost_equal(numericalH1, actualH1)
        numericalH2 = numericalDerivative22<Camera,Pose3,Cal3_S2>(f,pose,K)
        assert_almost_equal(numericalH2, actualH2)


    def test_Pose(self) -> None:
        actualH = np.zeros((1,1), order='F') 
        self.assertPose3Equals(pose, camera.getPose(actualH))

        # Check derivative
        std::function<Pose3(Camera)> f =  #
            std::bind(&Camera::getPose, std::placeholders::_1, nullptr)
        numericalH = numericalDerivative11<Pose3,Camera>(f,camera)
        assert_almost_equal(numericalH, actualH)



    def test_Dproject(self) -> None:
        Dpose = np.zeros((1,1), order='F') 
        Dpoint = np.zeros((1,1), order='F') 
        Dcal = np.zeros((1,1), order='F') 
        result = camera.project(point1, Dpose, Dpoint, Dcal)
        numerical_pose  = numericalDerivative31(project3, pose, point1, K)
        Hexpected2 = numericalDerivative32(project3, pose, point1, K)
        numerical_cal   = numericalDerivative33(project3, pose, point1, K)
        assert_almost_equal(Point2(-100,  100), result)
        assert_almost_equal(numerical_pose,  Dpose)
        assert_almost_equal(Hexpected2, Dpoint)
        assert_almost_equal(numerical_cal,   Dcal)



    def test_Dproject_Infinity(self) -> None:
        Dpose = np.zeros((1,1), order='F') 
        Dpoint = np.zeros((1,1), order='F')
        Dcal = np.zeros((1,1), order='F')
        point3D = Unit3(point1.x(), point1.y(), -10.0) # a point in front of the camera1

        # test Projection
        actual = camera.project(point3D, Dpose, Dpoint, Dcal)
        expected = Point2(-5.0, 5.0)
        assert_almost_equal(actual, expected)

        # test Jacobians
        numerical_pose     = numericalDerivative31(projectInfinity3, pose, point3D, K)
        Hexpected2    = numericalDerivative32(projectInfinity3, pose, point3D, K)
        numerical_point2x2 = Hexpected2.block(0,0,2,2) # only the direction to the point matters
        numerical_cal      = numericalDerivative33(projectInfinity3, pose, point3D, K)
        assert_almost_equal(numerical_pose,     Dpose)
        assert_almost_equal(numerical_point2x2, Dpoint)
        assert_almost_equal(numerical_cal,      Dcal)




    def test_Dproject2(self) -> None:
        Dcamera  = np.zeros((1,1), order='F')
        Dpoint  = np.zeros((1,1), order='F')
        result = camera.project2(point1, Dcamera, Dpoint)
        Hexpected1 = numericalDerivative21(project4, camera, point1)
        Hexpected2  = numericalDerivative22(project4, camera, point1)
        assert_almost_equal(result, Point2(-100,  100) )
        assert_almost_equal(Hexpected1, Dcamera)
        assert_almost_equal(Hexpected2,  Dpoint)


    # Add a test with more arbitrary rotation
    def test_Dproject3(self) -> None:
        pose1 = Pose3(Rot3::Ypr(0.1, -0.1, 0.4), Point3(0, 0, -10))
        camera = PinholeCameraCal3_S2(pose1)
        Dpose  = np.zeros((1,1), order='F')
        Dpoint  = np.zeros((1,1), order='F')
        camera.project2(point1, Dpose, Dpoint)
        numerical_pose  = numericalDerivative21(project4, camera, point1)
        numerical_point = numericalDerivative22(project4, camera, point1)
        assert_almost_equal(numerical_pose,  Dpose)
        assert_almost_equal(numerical_point, Dpoint)



    def test_range0(self) -> None:
        D1  = np.zeros((1,1), order='F')
        D2  = np.zeros((1,1), order='F')
        result = camera.range(point1, D1, D2)
        Hexpected1 = numericalDerivative21(range0, camera, point1)
        Hexpected2 = numericalDerivative22(range0, camera, point1)
        self.assertAlmostEqual(distance3(point1, camera.pose().translation()), result)
        assert_almost_equal(Hexpected1, D1)
        assert_almost_equal(Hexpected2, D2)


    def test_range1(self) -> None:
        D1  = np.zeros((1,1), order='F')
        D2  = np.zeros((1,1), order='F')
        result = camera.range(pose1, D1, D2)
        Hexpected1 = numericalDerivative21(range1, camera, pose1)
        Hexpected2 = numericalDerivative22(range1, camera, pose1)
        self.assertAlmostEqual(1, result)
        assert_almost_equal(Hexpected1, D1)
        assert_almost_equal(Hexpected2, D2)


    def test_range2(self) -> None:
        D1  = np.zeros((1,1), order='F') 
        D2  = np.zeros((1,1), order='F')
        result = camera.range<Cal3Bundler>(camera2, D1, D2)
        Hexpected1 = numericalDerivative21(range2, camera, camera2)
        Hexpected2 = numericalDerivative22(range2, camera, camera2)
        self.assertAlmostEqual(1, result)
        assert_almost_equal(Hexpected1, D1)
        assert_almost_equal(Hexpected2, D2)


    def test_range3(self) -> None:
        D1 = np.zeros((1,1), order='F') 
        D2 = np.zeros((1,1), order='F')
        result = camera.range(camera3, D1, D2)
        Hexpected1 = numericalDerivative21(range3, camera, camera3)
        Hexpected2 = numericalDerivative22(range3, camera, camera3)
        self.assertAlmostEqual(1, result)
        assert_almost_equal(Hexpected1, D1)
        assert_almost_equal(Hexpected2, D2)



if __name__ == "__main__":
    unittest.main()

