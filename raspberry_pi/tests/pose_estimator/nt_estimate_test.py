"""Exercise the network tables estimator."""

# pylint: disable=W0212


import time
from typing import cast
import unittest

import ntcore
from wpimath.geometry import Pose2d, Rotation2d

from app.config.camera_config import CameraConfig
from app.config.identity import Identity
from app.field.field_map import FieldMap
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)
from app.network.structs import Blip25, PoseEstimate25
from app.network.network import Network
from app.pose_estimator.nt_estimate import NTEstimate


class NTEstimateTest(unittest.TestCase):

    def test_real_nt_est_blips(self) -> None:
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        pub = inst.getStructArrayTopic("blip25", Blip25).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        pub_prior = inst.getStructTopic("prior", Pose2d).publish()
        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        field_map = FieldMap()
        cam = CameraConfig(Identity.UNKNOWN)
        net = Network(Identity.UNKNOWN)
        est = NTEstimate(field_map, cam, net)

        # for this test, we want to start at a known location.
        # this is the same as the CircleSimulator starting point.
        pub_prior.set(Pose2d(2, 0.02, Rotation2d(0.05)), 0)

        estimate = None
        for i in range(10):
            time.sleep(0.02)
            time_us = ntcore._now()
            pub.set(
                [
                    Blip25(
                        tag_id=0,
                        # these values are from nt_sim_nt_estimate_test output
                        llx=399.517,
                        lly=219.329,
                        lrx=431.516,
                        lry=218.937,
                        urx=430.735,
                        ury=189.683,
                        ulx=399.529,
                        uly=190.269,
                    ),
                ],
                time_us,
            )
            est.step()
            estimate = sub.get()
            # print("ESTIMATE", estimate)

        self.assertIsNotNone(estimate)
        estimate = cast(PoseEstimate25, estimate)

        # with just one sighting the resulting pose is quite sensitive
        # to the exact values above.
        self.assertAlmostEqual(2.0, estimate.x, 3)
        self.assertAlmostEqual(0.02, estimate.y, 3)
        self.assertAlmostEqual(0.06, estimate.theta, 3)
        self.assertAlmostEqual(0.008, estimate.x_sigma, 3)
        self.assertAlmostEqual(0.068, estimate.y_sigma, 3)
        self.assertAlmostEqual(0.068, estimate.theta_sigma, 3)
        self.assertAlmostEqual(0, estimate.dx, 3)
        self.assertAlmostEqual(0, estimate.dy, 3)
        self.assertAlmostEqual(0, estimate.dtheta, 3)
        self.assertAlmostEqual(0, estimate.dt, 3)

    def test_real_nt_est_odo(self) -> None:
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        pub = inst.getStructTopic("odometry", SwerveModulePositions).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        pub_prior = inst.getStructTopic("prior", Pose2d).publish()

        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        field_map = FieldMap()
        cam = CameraConfig(Identity.UNKNOWN)
        net = Network(Identity.UNKNOWN)
        est = NTEstimate(field_map, cam, net)
        # for this test, we want to start at a known location.
        pub_prior.set(Pose2d(), 0)
        estimate = None
        for i in range(10):
            time.sleep(0.02)
            # print("NTEstTest.test_real_nt_est() i ", i)
            time_us = ntcore._now()
            # print("NTEstTest.test_real_nt_est() time_us ", time_us)

            pub.set(
                SwerveModulePositions(
                    SwerveModulePosition100(
                        0.1 * i, OptionalRotation2d(True, Rotation2d(0))
                    ),
                    SwerveModulePosition100(
                        0.1 * i, OptionalRotation2d(True, Rotation2d(0))
                    ),
                    SwerveModulePosition100(
                        0.1 * i, OptionalRotation2d(True, Rotation2d(0))
                    ),
                    SwerveModulePosition100(
                        0.1 * i, OptionalRotation2d(True, Rotation2d(0))
                    ),
                ),
                time_us,
            )
            est.step()
            estimate = sub.get()
            print(estimate)

        self.assertIsNotNone(estimate)
        estimate = cast(PoseEstimate25, estimate)

        # so what are we left with?
        # ten steps of 0.1 each,
        # relative to the ridiculously-wide prior mean
        # because there is no other grounding
        self.assertAlmostEqual(0.9, estimate.x, 3)
        self.assertAlmostEqual(0, estimate.y, 3)
        self.assertAlmostEqual(0, estimate.theta, 3)

        # prior above was 0.05, each odo is 0.01
        # we're moving in x but there's more y sigma
        # because there's also theta sigma
        self.assertAlmostEqual(0.058, estimate.x_sigma, 3)
        self.assertAlmostEqual(0.075, estimate.y_sigma, 3)
        self.assertAlmostEqual(0.058, estimate.theta_sigma, 3)

        # we should get back the odometry we sent
        self.assertAlmostEqual(0.1, estimate.dx, 3)
        self.assertAlmostEqual(0, estimate.dy, 3)
        self.assertAlmostEqual(0, estimate.dtheta, 3)
        # sometimes we miss one?
        # TODO: huh?
        # self.assertAlmostEqual(20000, estimate.dt, 3)

    def test_real_nt_est_gyro(self) -> None:
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        pub = inst.getStructTopic("gyro", Rotation2d).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        field_map = FieldMap()
        cam = CameraConfig(Identity.UNKNOWN)
        net = Network(Identity.UNKNOWN)
        est = NTEstimate(field_map, cam, net)
        estimate = None
        for i in range(10):
            time.sleep(0.02)
            time_us = ntcore._now()

            pub.set(
                Rotation2d(0.1 * i),
                time_us,
            )
            est.step()
            estimate = sub.get()
            print(estimate)

        self.assertIsNotNone(estimate)
        estimate = cast(PoseEstimate25, estimate)
        # so what are we left with?
        # the x and y are the ridiculously-loose prior
        # since the gyro has nothing to say about it
        self.assertAlmostEqual(8, estimate.x, 3)
        self.assertAlmostEqual(4, estimate.y, 3)
        self.assertAlmostEqual(0.9, estimate.theta, 3)
        # prior was enormous
        self.assertAlmostEqual(160, estimate.x_sigma, 3)
        self.assertAlmostEqual(80, estimate.y_sigma, 3)
        # prior was 0.001
        self.assertAlmostEqual(0.001, estimate.theta_sigma, 3)
