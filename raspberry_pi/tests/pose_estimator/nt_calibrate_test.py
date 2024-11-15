"""Exercise the network tables estimator."""

# pylint: disable=W0212


import time
from typing import cast
import unittest

import ntcore
from wpimath.geometry import Rotation2d, Pose2d

from app.config.identity import Identity
from app.field.field_map import FieldMap
from app.kinodynamics.swerve_module_position import (OptionalRotation2d,
                                                     SwerveModulePosition100,
                                                     SwerveModulePositions)
from app.network.fake_network import FakeNetwork
from app.network.network_protocol import Blip25, CameraCalibration, PoseEstimate25
from app.network.real_network import RealNetwork
from app.pose_estimator.nt_calibrate import NTCalibrate


class NTCalibrateTest(unittest.TestCase):
    def test_real_nt_est_blips(self) -> None:
        # print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        pub = inst.getStructArrayTopic("foo/1", Blip25).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        field_map = FieldMap()
        net = RealNetwork(Identity.UNKNOWN)
        est = NTCalibrate(field_map, net)
        estimate = None
        for i in range(10):
            time.sleep(0.02)
            time_us = ntcore._now()
            # this is not enough data to learn anything.
            pub.set(
                [
                    Blip25(0, 190, 210, 210, 210, 210, 190, 190, 190),
                    Blip25(0, 190, 210, 210, 210, 210, 190, 190, 190),
                ],
                time_us,
            )
            est.step()
            estimate = sub.get()
            # print(estimate)
        if estimate is not None:
            # so what are we left with?
            # the calibrator is bad at doing anything
            # with so few data points, so this is
            # garbage.  note the enormous tolerance.
            # ?????
            self.assertAlmostEqual(2.9, estimate.x, 0)
            self.assertAlmostEqual(-2.4, estimate.y, 0)
            self.assertAlmostEqual(-0.3, estimate.theta, 0)
            # ???
            self.assertAlmostEqual(0.3, estimate.x_sigma, 0)
            # ???
            self.assertAlmostEqual(0.6, estimate.y_sigma, 0)
            # ???
            self.assertAlmostEqual(0.2, estimate.theta_sigma, 0)
            #
            self.assertAlmostEqual(0, estimate.dx, 0)
            self.assertAlmostEqual(0, estimate.dy, 0)
            self.assertAlmostEqual(0, estimate.dtheta, 0)
            self.assertAlmostEqual(0, estimate.dt, 0)

    def test_fake_nt_est_blips(self) -> None:
        field_map = FieldMap()
        net = FakeNetwork()
        est = NTCalibrate(field_map, net)
        start_time_us = ntcore._now()
        for _ in range(10):
            time.sleep(0.02)
            time_us = ntcore._now() - start_time_us
            net.received_blip25s["blip25"] = [
                (
                    time_us,
                    [
                        Blip25(0, 190, 210, 210, 210, 210, 190, 190, 190),
                        Blip25(0, 190, 210, 210, 210, 210, 190, 190, 190),
                    ],
                )
            ]
            est.step()
        print(net.estimate)

        # these are garbage values
        self.assertAlmostEqual(2.9, net.estimate.x, 0)
        #
        self.assertAlmostEqual(-2.4, net.estimate.y, 0)
        #
        self.assertAlmostEqual(0, net.estimate.theta, 0)
        #
        self.assertAlmostEqual(0.0, net.estimate.x_sigma, 0)
        #
        self.assertAlmostEqual(0.6, net.estimate.y_sigma, 0)
        #
        self.assertAlmostEqual(0.2, net.estimate.theta_sigma, 0)
        #
        self.assertAlmostEqual(0, net.estimate.dx, 0)
        self.assertAlmostEqual(0, net.estimate.dy, 0)
        self.assertAlmostEqual(0, net.estimate.dtheta, 0)
        self.assertAlmostEqual(0, net.estimate.dt, 0)

    def test_real_nt_est_odo(self) -> None:
        """Here we're just driving forward at a constant speed,
        and it works fine because odometry is pretty tight.
        """
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        pub = inst.getStructTopic("odometry", SwerveModulePositions).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        pub_prior = inst.getStructTopic("prior", Pose2d).publish()
        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        sub_cal = inst.getStructTopic("calib", CameraCalibration).subscribe(None)
        field_map = FieldMap()
        net = RealNetwork(Identity.UNKNOWN)
        est = NTCalibrate(field_map, net)

        # for this test, we want to start at a known location.
        pub_prior.set(Pose2d(), 0)
        
        estimate = None
        cal = None
        for i in range(10):
            time.sleep(0.02)
            time_us = ntcore._now()
            # print(i, time_us)

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
            cal = sub_cal.get()
            # print(cal)
        self.assertIsNotNone(estimate)
        estimate = cast(PoseEstimate25, estimate)
        # so what are we left with?
        # ten steps of 0.1 each,
        # relative to the ridiculously-wide prior mean
        # because there is no other grounding
        self.assertAlmostEqual(0.9, estimate.x, 3)
        self.assertAlmostEqual(0, estimate.y, 3)
        self.assertAlmostEqual(0, estimate.theta, 3)
        # prior was 0.3, each odo is 0.01
        self.assertAlmostEqual(0.301, estimate.x_sigma, 3)
        self.assertAlmostEqual(0.315, estimate.y_sigma, 3)
        # prior was 0.1
        self.assertAlmostEqual(0.104, estimate.theta_sigma, 3)
        # we should get back the odometry we sent
        self.assertAlmostEqual(0.1, estimate.dx, 3)
        self.assertAlmostEqual(0, estimate.dy, 3)
        self.assertAlmostEqual(0, estimate.dtheta, 3)
        self.assertAlmostEqual(20000, estimate.dt, 3)
        # there is a calibration but it's just the default
        # since there's no camera data.
        self.assertIsNotNone(cal)

    def test_real_nt_est_gyro(self) -> None:
        """The calibrator is pretty good at yaw
        since the gyro factor is very demanding."""
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        # TODO: consolidate these names
        pub = inst.getStructTopic("gyro", Rotation2d).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        sub_cal = inst.getStructTopic("calib", CameraCalibration).subscribe(None)
        field_map = FieldMap()
        net = RealNetwork(Identity.UNKNOWN)
        est = NTCalibrate(field_map, net)
        estimate: PoseEstimate25 | None = None
        cal = None
        for i in range(10):
            time.sleep(0.02)
            time_us = ntcore._now()
            # print(i, time_us)

            pub.set(
                Rotation2d(0.1 * i),
                time_us,
            )
            est.step()
            estimate = sub.get()
            # print(estimate)
            cal = sub_cal.get()
            # print(cal)
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
        # prior was 0.1
        self.assertAlmostEqual(0.0001, estimate.theta_sigma, 2)
        # there is a calibration but it's just the default
        # since there's no camera data.
        self.assertIsNotNone(cal)

