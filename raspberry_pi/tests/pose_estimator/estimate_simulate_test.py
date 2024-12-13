# pylint: disable=C0301,E0611,E1101,R0903,R0914

import time
import unittest

import gtsam
import numpy as np
from gtsam import Pose2  # type:ignore
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

from app.field.field_map import FieldMap
from app.pose_estimator.estimate import Estimate
from tests.pose_estimator.simulation.circle_simulator import CircleSimulator

ACTUALLY_PRINT = False

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class EstimateSimulateTest(unittest.TestCase):
    def test_odo_only(self) -> None:
        """Odometry only, using the native factor.
        This is very fast, 0.1s on my machine for a 1s window,
        0.03s for a 0.1s window"""
        sim = CircleSimulator(FieldMap())
        est = Estimate(0.1)
        est.init()

        # sim starts at (2,0)
        prior_mean = gtsam.Pose2(2, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        state = gtsam.Pose2()
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

        # this should just record the positions and timestamp
        est.odometry(0, sim.positions, odometry_noise)

        print()
        print(
            "      t,    GT X,    GT Y,  GT ROT,   EST X,   EST Y, EST ROT,   ERR X,   ERR Y, ERR ROT"
        )
        for i in range(1, 100):
            t0 = time.time_ns()
            t0_us = 20000 * (i - 1)
            t1_us = 20000 * i
            # updates gt to t1
            sim.step(0.02)
            est.add_state(t1_us, state)
            est.odometry(t1_us, sim.positions, odometry_noise)
            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result_size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.mean_pose2(X(t1_us))
            # use the previous estimate as the new estimate.
            state = p
            est_x = p.x()
            est_y = p.y()
            est_theta = p.theta()

            err_x = est_x - gt_x
            err_y = est_y - gt_y
            err_theta = est_theta - gt_theta

            print(
                f"{t:7.4f}, {gt_x:7.4f}, {gt_y:7.4f}, {gt_theta:7.4f}, {est_x:7.4f}, {est_y:7.4f}, {est_theta:7.4f}, {err_x:7.4f}, {err_y:7.4f}, {err_theta:7.4f}"
            )

    def test_odo_and_gyro(self) -> None:

        sim = CircleSimulator(FieldMap())
        est = Estimate(0.1)
        est.init()

        # sim starts at (2,0)
        prior_mean = gtsam.Pose2(2, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        state = gtsam.Pose2()
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

        # this should just record the positions and timestamp
        est.odometry(0, sim.positions, odometry_noise)

        print()
        print(
            "      t,    GT X,    GT Y,  GT ROT,   EST X,   EST Y, EST ROT,   ERR X,   ERR Y, ERR ROT"
        )
        for i in range(1, 100):
            t0 = time.time_ns()
            t0_us = 20000 * (i - 1)
            t1_us = 20000 * i
            # updates gt to t1
            sim.step(0.02)
            est.add_state(t1_us, state)
            est.odometry(t1_us, sim.positions, odometry_noise)
            est.gyro(t1_us, sim.gt_theta)
            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result_size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.mean_pose2(X(t1_us))
            # use the previous estimate as the new estimate.
            state = p
            est_x = p.x()
            est_y = p.y()
            est_theta = p.theta()

            err_x = est_x - gt_x
            err_y = est_y - gt_y
            err_theta = est_theta - gt_theta

            print(
                f"{t:7.4f}, {gt_x:7.4f}, {gt_y:7.4f}, {gt_theta:7.4f}, {est_x:7.4f}, {est_y:7.4f}, {est_theta:7.4f}, {err_x:7.4f}, {err_y:7.4f}, {err_theta:7.4f}"
            )

    def test_camera_batched(self) -> None:
        """Camera with batch factors.
        using 0.1s window
        using 2s of data
        Python: 0.3s (15% of realtime)
        C++: 0.04s (2% of realtime)
        """
        sim = CircleSimulator(FieldMap())
        est = Estimate(0.1)
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        state = gtsam.Pose2()

        if ACTUALLY_PRINT:
            print()
            print(
                "      t,    GT X,    GT Y,  GT ROT,   EST X,   EST Y, EST ROT,   ERR X,   ERR Y, ERR ROT"
            )
        for i in range(1, 100):
            t0 = time.time_ns()
            t0_us = 20000 * i
            # updates gt to t0
            sim.step(0.02)

            if len(sim.gt_pixels) == 0:
                continue

            est.add_state(t0_us, state)

            all_pixels = np.concatenate(sim.gt_pixels)
            est.apriltag_for_smoothing_batch(
                sim.landmarks, all_pixels, t0_us, sim.camera_offset, sim.calib
            )

            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result_size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.mean_pose2(X(t0_us))
            # use the previous estimate as the new estimate.
            state = p
            est_x = p.x()
            est_y = p.y()
            est_theta = p.theta()

            err_x = est_x - gt_x
            err_y = est_y - gt_y
            err_theta = est_theta - gt_theta
            if ACTUALLY_PRINT:

                print(
                    f"{t:7.4f}, {gt_x:7.4f}, {gt_y:7.4f}, {gt_theta:7.4f}, {est_x:7.4f}, {est_y:7.4f}, {est_theta:7.4f}, {err_x:7.4f}, {err_y:7.4f}, {err_theta:7.4f}"
                )

    def test_batched_camera_and_odometry_and_gyro(self) -> None:
        """Camera and odometry and gyro.
        using 0.1s window
        using 2s of data
        Batching python: 0.33s (i.e. 16% of realtime)
        Non-batching c++: 0.07s (i.e. 4% of realtime)
        """
        sim = CircleSimulator(FieldMap())
        est = Estimate(0.1)
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, noiseModel.Diagonal.Sigmas(np.array([100, 100, 100])))

        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
        # this should just record the positions and timestamp
        est.odometry(0, sim.positions, odometry_noise)

        state = gtsam.Pose2()

        if ACTUALLY_PRINT:
            print()
            print(
                "      t,    GT X,    GT Y,  GT ROT,   EST X,   EST Y, EST ROT,   ERR X,   ERR Y, ERR ROT"
            )
        # gt_theta_0 = sim.gt_theta
        for i in range(1, 100):
            t0 = time.time_ns()

            # t0_us = 20000 * (i - 1)
            t1_us = 20000 * i
            # updates gt to t1
            sim.step(0.02)
            est.add_state(t1_us, state)
            est.odometry(t1_us, sim.positions, odometry_noise)
            # dtheta = sim.gt_theta - gt_theta_0
            est.gyro(t1_us, sim.gt_theta)
            # gt_theta_0 = sim.gt_theta

            if len(sim.gt_pixels) > 0:
                all_pixels = np.concatenate(sim.gt_pixels)
                est.apriltag_for_smoothing_batch(
                    sim.landmarks, all_pixels, t1_us, sim.camera_offset, sim.calib
                )

            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result_size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.mean_pose2(X(t1_us))
            # use the previous estimate as the new estimate.
            state = p
            est_x = p.x()
            est_y = p.y()
            est_theta = p.theta()

            err_x = est_x - gt_x
            err_y = est_y - gt_y
            err_theta = est_theta - gt_theta

            if ACTUALLY_PRINT:
                print(
                    f"{t:7.4f}, {gt_x:7.4f}, {gt_y:7.4f}, {gt_theta:7.4f}, {est_x:7.4f}, {est_y:7.4f}, {est_theta:7.4f}, {err_x:7.4f}, {err_y:7.4f}, {err_theta:7.4f}"
                )
