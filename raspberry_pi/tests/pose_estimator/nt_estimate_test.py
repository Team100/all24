"""Exercise the network tables estimator."""

# pylint: disable=W0212


import time
import unittest

import ntcore
from wpimath.geometry import Rotation2d

from app.config.identity import Identity
from app.network.fake_network import FakeNetwork
from app.network.network_protocol import Blip25, PoseEstimate25
from app.network.real_network import RealNetwork
from app.field.field_map import FieldMap
from app.pose_estimator.nt_estimate import NTEstimate
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)


class NTEstimateTest(unittest.TestCase):
    def test_real_nt_est_blips(self) -> None:
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        pub = inst.getStructArrayTopic("foo/1", Blip25).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        field_map = FieldMap()
        net = RealNetwork(Identity.UNKNOWN)
        est = NTEstimate(field_map, net)
        estimate = None
        for i in range(10):
            time.sleep(0.02)
            time_us = ntcore._now()
            pub.set(
                [
                    Blip25(0, 190, 210, 210, 210, 210, 190, 190, 190),
                    Blip25(0, 190, 210, 210, 210, 210, 190, 190, 190),
                ],
                time_us,
            )
            est.step()
            estimate = sub.get()
            print(estimate)
        if estimate is not None:
            # so what are we left with?
            # right in front of the tag, as expected.
            self.assertAlmostEqual(1.351, estimate.x, 3)
            self.assertAlmostEqual(0, estimate.y, 3)
            self.assertAlmostEqual(0, estimate.theta, 3)
            # good at estimating range
            self.assertAlmostEqual(0.041, estimate.x_sigma, 3)
            # not good at estimating bearing
            self.assertAlmostEqual(1.169, estimate.y_sigma, 3)
            # not good at estimating yaw
            self.assertAlmostEqual(0.707, estimate.theta_sigma, 3)
            # no odometry
            self.assertAlmostEqual(0, estimate.dx, 3)
            self.assertAlmostEqual(0, estimate.dy, 3)
            self.assertAlmostEqual(0, estimate.dtheta, 3)
            self.assertAlmostEqual(0, estimate.dt, 3)

    def test_fake_nt_est_blips(self) -> None:
        field_map = FieldMap()
        net = FakeNetwork()
        est = NTEstimate(field_map, net)
        start_time_us = ntcore._now()
        for _ in range(10):
            time.sleep(0.02)
            time_us = ntcore._now() - start_time_us
            net.received_blip25s["foo"] = [
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

        # so what are we left with?
        # right in front of the tag, as expected.
        self.assertAlmostEqual(1.351, net.estimate.x, 3)
        # this is not exctly 0 due to the wide prior
        self.assertAlmostEqual(0, net.estimate.y, 2)
        # this is not exctly 0 due to the wide prior
        self.assertAlmostEqual(0, net.estimate.theta, 2)
        # good at estimating range
        self.assertAlmostEqual(0.041, net.estimate.x_sigma, 3)
        # not good at estimating bearing
        self.assertAlmostEqual(1.169, net.estimate.y_sigma, 3)
        # not good at estimating yaw
        self.assertAlmostEqual(0.707, net.estimate.theta_sigma, 3)
        # no odometry
        self.assertAlmostEqual(0, net.estimate.dx, 3)
        self.assertAlmostEqual(0, net.estimate.dy, 3)
        self.assertAlmostEqual(0, net.estimate.dtheta, 3)
        self.assertAlmostEqual(0, net.estimate.dt, 3)

    def test_real_nt_est_odo(self) -> None:
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        pub = inst.getStructTopic("bar", SwerveModulePositions).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        field_map = FieldMap()
        net = RealNetwork(Identity.UNKNOWN)
        est = NTEstimate(field_map, net)
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
        if estimate is not None:
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
            # sometimes we miss one?
            # TODO: huh?
            # self.assertAlmostEqual(20000, estimate.dt, 3)

    def test_real_nt_est_gyro(self) -> None:
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        pub = inst.getStructTopic("baz", Rotation2d).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        sub = inst.getStructTopic("pose", PoseEstimate25).subscribe(None)
        field_map = FieldMap()
        net = RealNetwork(Identity.UNKNOWN)
        est = NTEstimate(field_map, net)
        estimate = None
        for i in range(10):
            time.sleep(0.02)
            # print("NTEstTest.test_real_nt_est() i ", i)
            time_us = ntcore._now()
            # print("NTEstTest.test_real_nt_est() time_us ", time_us)

            pub.set(
                Rotation2d(0.1 * i),
                time_us,
            )
            est.step()
            estimate = sub.get()
            print(estimate)
        if estimate is not None:
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
            self.assertAlmostEqual(0.0001, estimate.theta_sigma, 3)
