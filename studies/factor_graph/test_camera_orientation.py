# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0621
# really to test numeric differentation
# see testCal3DS2.cpp

import unittest
import numpy as np
from gtsam import Cal3DS2
from gtsam import PinholeCameraCal3DS2
from gtsam import Point3, Pose3, Rot3

# camera "zero" is facing +z; this turns it to face +x
CAM_COORD = Pose3(Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])), Point3(0, 0, 0))
CX = 50
CY = 50
FX = 50
FY = 50
# no distortion
K = Cal3DS2(FX, FY, 0.0, CX, CY, 0.0, 0.0, 0.0, 0.0)


class TestCameraOrientation(unittest.TestCase):
    def test_composition(self) -> None:
        robot_pose = Pose3(Rot3.Identity(), Point3(0,0,0))
        offset = Pose3(Rot3.Yaw(1), Point3(0, 1, 0))
        camera = robot_pose.compose(offset)
        print(camera)

    def test_center(self) -> None:
        # camera at origin facing down X
        camera_in_world = Pose3(Rot3.Identity(), Point3(0, 0, 0))
        camera_pose = camera_in_world.compose(CAM_COORD)
        camera = PinholeCameraCal3DS2(camera_pose, K)
        # target 1m ahead
        landmark = Point3(1, 0, 0)
        pixel = camera.project(landmark)
        # target should be in the center
        self.assertAlmostEqual(50, pixel[0])
        self.assertAlmostEqual(50, pixel[1])

    def test_target_left(self) -> None:
        # camera at origin facing down X
        camera_in_world = Pose3(Rot3.Identity(), Point3(0, 0, 0))
        camera_pose = camera_in_world.compose(CAM_COORD)
        camera = PinholeCameraCal3DS2(camera_pose, K)
        # target 1m ahead, 0.5 m to the left
        landmark = Point3(1, 0.5, 0)
        pixel = camera.project(landmark)
        # target should be to the left, which means a smaller x number (x-positive-right)
        self.assertAlmostEqual(25, pixel[0])
        self.assertAlmostEqual(50, pixel[1])

    def test_target_up(self) -> None:
        # camera at origin facing down X
        camera_in_world = Pose3(Rot3.Identity(), Point3(0, 0, 0))
        camera_pose = camera_in_world.compose(CAM_COORD)
        camera = PinholeCameraCal3DS2(camera_pose, K)
        # target 1m ahead, 0.5 m up
        landmark = Point3(1, 0, 0.5)
        pixel = camera.project(landmark)
        # target should be up, which means a smaller y number (y-positive-down)
        self.assertAlmostEqual(50, pixel[0])
        self.assertAlmostEqual(25, pixel[1])

    def test_camera_left(self) -> None:
        # camera facing down X, a bit to the left (+y)
        camera_in_world = Pose3(Rot3.Identity(), Point3(0, 0.5, 0))
        camera_pose = camera_in_world.compose(CAM_COORD)
        camera = PinholeCameraCal3DS2(camera_pose, K)
        # target 1m ahead
        landmark = Point3(1, 0, 0)
        pixel = camera.project(landmark)
        # target should be a bit to the right (+x)
        self.assertAlmostEqual(75, pixel[0])
        self.assertAlmostEqual(50, pixel[1])

    def test_camera_up(self) -> None:
        # camera facing down X, a bit up (+z)
        camera_in_world = Pose3(Rot3.Identity(), Point3(0, 0, 0.5))
        camera_pose = camera_in_world.compose(CAM_COORD)
        camera = PinholeCameraCal3DS2(camera_pose, K)
        # target 1m ahead
        landmark = Point3(1, 0, 0)
        pixel = camera.project(landmark)
        # target should be a bit lower (+y)
        self.assertAlmostEqual(50, pixel[0])
        self.assertAlmostEqual(75, pixel[1])

    def test_pan_left(self) -> None:
        # camera at origin rotated to the left
        camera_in_world = Pose3(Rot3.Yaw(0.2), Point3(0, 0, 0))
        camera_pose = camera_in_world.compose(CAM_COORD)
        camera = PinholeCameraCal3DS2(camera_pose, K)
        # target 1m ahead
        landmark = Point3(1, 0, 0)
        pixel = camera.project(landmark)
        # target should be to the right
        self.assertAlmostEqual(60.1355, pixel[0], 3)
        self.assertAlmostEqual(50, pixel[1])

    def test_truck_and_pan(self) -> None:
        # camera truck left pan (further than above) right, so target is near center again
        camera_in_world = Pose3(Rot3.Yaw(-0.45), Point3(0, 0.5, 0))
        camera_pose = camera_in_world.compose(CAM_COORD)
        camera = PinholeCameraCal3DS2(camera_pose, K)
        # target 1m ahead
        landmark = Point3(1, 0, 0)
        pixel = camera.project(landmark)
        # target should be in the center
        self.assertAlmostEqual(50.6824, pixel[0], 3)
        self.assertAlmostEqual(50, pixel[1])


if __name__ == "__main__":
    unittest.main()
