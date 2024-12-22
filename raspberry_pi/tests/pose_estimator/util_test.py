# pylint: disable=C0103,E1101


import math
import unittest

import gtsam
import numpy as np

from app.pose_estimator.util import pose2_to_pose2d, pose3_to_pose3d, to_cal


class UtilTest(unittest.TestCase):
    def test_pose3_to_pose3d_0(self) -> None:
        gp = gtsam.Pose3(gtsam.Rot3(), np.array([0, 0, 0]))
        wp = pose3_to_pose3d(gp)
        self.assertAlmostEqual(0, wp.rotation().X())
        self.assertAlmostEqual(0, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        self.assertAlmostEqual(0, wp.translation().X())
        self.assertAlmostEqual(0, wp.translation().Y())
        self.assertAlmostEqual(0, wp.translation().Z())

    def test_pose3_to_pose3d_1(self) -> None:
        gp = gtsam.Pose3(gtsam.Rot3(), np.array([1, 2, 3]))
        wp = pose3_to_pose3d(gp)
        self.assertAlmostEqual(0, wp.rotation().X())
        self.assertAlmostEqual(0, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        self.assertAlmostEqual(1, wp.translation().X())
        self.assertAlmostEqual(2, wp.translation().Y())
        self.assertAlmostEqual(3, wp.translation().Z())

    def test_pose3_to_pose3d_2(self) -> None:
        gp = gtsam.Pose3(gtsam.Rot3().Roll(math.pi / 2), np.array([0, 0, 0]))
        wp = pose3_to_pose3d(gp)
        self.assertAlmostEqual(math.pi / 2, wp.rotation().X())
        self.assertAlmostEqual(0, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        self.assertAlmostEqual(0, wp.translation().X())
        self.assertAlmostEqual(0, wp.translation().Y())
        self.assertAlmostEqual(0, wp.translation().Z())

    def test_pose3_to_pose3d_4(self) -> None:
        # grsam ypr is is intrinsic rotation:
        # yaw 90 degrees left, pitch 90 degrees down, roll 90 degrees right
        # the corresponding extrinsic rpy should be
        # pitch 90 degree down and that's all
        gp = gtsam.Pose3(
            gtsam.Rot3().Ypr(math.pi / 2, math.pi / 2, math.pi / 2), np.array([0, 0, 0])
        )
        wp = pose3_to_pose3d(gp)
        self.assertAlmostEqual(0, wp.rotation().X())
        self.assertAlmostEqual(math.pi / 2, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        self.assertAlmostEqual(0, wp.translation().X())
        self.assertAlmostEqual(0, wp.translation().Y())
        self.assertAlmostEqual(0, wp.translation().Z())

    def test_pose3_to_pose3d_5(self) -> None:
        # roll right and move +z, where do you end up?
        gp = gtsam.Pose3(gtsam.Rot3().Roll(math.pi / 2), np.array([0, 0, 1]))
        wp = pose3_to_pose3d(gp)
        print(wp)
        # roll is roll
        self.assertAlmostEqual(math.pi / 2, wp.rotation().X())
        self.assertAlmostEqual(0, wp.rotation().Y())
        self.assertAlmostEqual(0, wp.rotation().Z())
        # z is z, i.e. translation is applied first (or, equivalently, it's extrinsic)
        self.assertAlmostEqual(0, wp.translation().X())
        self.assertAlmostEqual(0, wp.translation().Y())
        self.assertAlmostEqual(1, wp.translation().Z())

    def test_pose2_to_pose2d(self) -> None:
        gp = gtsam.Pose2(0, 0, 0)
        wp = pose2_to_pose2d(gp)
        self.assertAlmostEqual(0, wp.X())
        self.assertAlmostEqual(0, wp.Y())
        self.assertAlmostEqual(0, wp.rotation().radians())
        gp = gtsam.Pose2(1, 2, 3)
        wp = pose2_to_pose2d(gp)
        self.assertAlmostEqual(1, wp.X())
        self.assertAlmostEqual(2, wp.Y())
        self.assertAlmostEqual(3, wp.rotation().radians())

    def test_cal(self) -> None:
        # gc = gtsam.Cal3DS2(100, 150, 0, 50, 75, 1, 2, 3, 4)
        gc = gtsam.Cal3DS2(100, 150, 0, 50, 75, 1, 2)
        wc = to_cal(gc)
        self.assertAlmostEqual(100, wc.fx)
        self.assertAlmostEqual(150, wc.fy)
        self.assertAlmostEqual(0, wc.s)
        self.assertAlmostEqual(50, wc.u0)
        self.assertAlmostEqual(75, wc.v0)
        self.assertAlmostEqual(1, wc.k1)
        self.assertAlmostEqual(2, wc.k2)
        # self.assertAlmostEqual(3, wc.p1)
        # self.assertAlmostEqual(4, wc.p2)

    def test_compose_h(self) -> None:
        """Figure out what the composition Jacobians do"""
        # compose two identities
        p0 = gtsam.Pose3()
        p1 = gtsam.Pose3()
        H1 = np.zeros((6, 6), order="F")
        H2 = np.zeros((6, 6), order="F")
        p0.compose(p1, H1, H2)

        # adjoint of the inverse
        print("H1", H1)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1],
                    ]
                ),
                H1,
            )
        )
        # always identity
        print("H2", H2)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1],
                    ]
                ),
                H2,
            )
        )

    def test_compose_h2(self) -> None:
        """Figure out what the composition Jacobians do"""
        # compose identity with pure translation
        p0 = gtsam.Pose3()
        p1 = gtsam.Pose3(gtsam.Rot3(), np.array([1, 0, 0]))
        H1 = np.zeros((6, 6), order="F")
        H2 = np.zeros((6, 6), order="F")
        p0.compose(p1, H1, H2)

        # adjoint of the inverse
        print("H1", H1)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 1, 0, 1, 0],
                        [0, -1, 0, 0, 0, 1],
                    ]
                ),
                H1,
            )
        )
        # always identity
        print("H2", H2)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1],
                    ]
                ),
                H2,
            )
        )

    def test_compose_h3(self) -> None:
        """Figure out what the composition Jacobians do"""
        # compose identity with pure rotation
        p0 = gtsam.Pose3()
        p1 = gtsam.Pose3(gtsam.Rot3().Yaw(1), np.array([0, 0, 0]))
        H1 = np.zeros((6, 6), order="F")
        H2 = np.zeros((6, 6), order="F")
        p0.compose(p1, H1, H2)

        # adjoint of the inverse
        print("H1", H1)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.540, 0.841, 0.000, 0.000, 0.000, 0.000],
                        [-0.841, 0.540, 0.000, 0.000, 0.000, 0.000],
                        [0.000, 0.000, 1.000, 0.000, 0.000, 0.000],
                        [0.000, 0.000, 0.000, 0.540, 0.841, 0.000],
                        [0.000, 0.000, 0.000, -0.841, 0.540, 0.000],
                        [0.000, 0.000, 0.000, 0.000, 0.000, 1.000],
                    ]
                ),
                H1,
                atol=0.001,
            )
        )
        # always identity
        print("H2", H2)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1],
                    ]
                ),
                H2,
            )
        )

    def test_compose_h4(self) -> None:
        """Figure out what the composition Jacobians do"""
        # compose identity with translation + rotation
        p0 = gtsam.Pose3()
        p1 = gtsam.Pose3(gtsam.Rot3().Yaw(1), np.array([1, 0, 0]))
        H1 = np.zeros((6, 6), order="F")
        H2 = np.zeros((6, 6), order="F")
        p0.compose(p1, H1, H2)

        # adjoint of the inverse
        print("H1", H1)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.540, 0.841, 0, 0, 0, 0],
                        [-0.841, 0.540, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0.841, 0.540, 0.841, 0],
                        [0, 0, 0.540, -0.841, 0.540, 0],
                        [0, -1, 0, 0, 0, 1],
                    ]
                ),
                H1,
                atol=0.001
            )
        )
        # always identity
        print("H2", H2)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1],
                    ]
                ),
                H2,
            )
        )


    def test_compose_2h(self) -> None:
        """Figure out what the composition Jacobians do"""
        # compose two identities
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2()
        H1 = np.zeros((3, 3), order="F")
        H2 = np.zeros((3, 3), order="F")
        p0.compose(p1, H1, H2)

        # adjoint of the inverse
        print("H1", H1)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1],

                    ]
                ),
                H1,
            )
        )
        # always identity
        print("H2", H2)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1],
                    ]
                ),
                H2,
            )
        )

    def test_compose_2h2(self) -> None:
        """Figure out what the composition Jacobians do"""
        # compose identity with pure translation
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2(1, 0, 0)
        H1 = np.zeros((3, 3), order="F")
        H2 = np.zeros((3, 3), order="F")
        p0.compose(p1, H1, H2)

        # adjoint of the inverse
        # since we moved +x, dtheta produces dy.
        print("H1", H1)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0],
                        [0, 1, 1],
                        [0, 0, 1],
                    ]
                ),
                H1,
            )
        )
        # always identity
        print("H2", H2)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1],
                    ]
                ),
                H2,
            )
        )

    def test_compose_2h3(self) -> None:
        """Figure out what the composition Jacobians do"""
        # compose identity with pure rotation
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2(0, 0, 1)
        H1 = np.zeros((3, 3), order="F")
        H2 = np.zeros((3, 3), order="F")
        p0.compose(p1, H1, H2)

        # adjoint of the inverse
        # pure rotation
        print("H1", H1)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.540, 0.841, 0],
                        [-0.841, 0.540, 0],
                        [0, 0, 1],
                    ]
                ),
                H1,
                atol=0.001,
            )
        )
        # always identity
        print("H2", H2)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1],
                    ]
                ),
                H2,
            )
        )

    def test_compose_2h4(self) -> None:
        """Figure out what the composition Jacobians do"""
        # compose identity with translation + rotation
        p0 = gtsam.Pose2()
        p1 = gtsam.Pose2(1, 0, 1)
        H1 = np.zeros((3, 3), order="F")
        H2 = np.zeros((3, 3), order="F")
        p0.compose(p1, H1, H2)

        # adjoint of the inverse
        # same theta-y coupling but since we rotated a bit, it's not just y.
        print("H1", H1)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [0.540, 0.841, 0.841],
                        [-0.841, 0.540, 0.540],
                        [0, 0, 1],
                    ]
                ),
                H1,
                atol=0.001
            )
        )
        # always identity
        print("H2", H2)
        self.assertTrue(
            np.allclose(
                np.array(
                    [
                        [1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1],
                    ]
                ),
                H2,
            )
        )