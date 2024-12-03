"""Contains simulation/estimation tests that use Parking Lot factors."""

# pylint: disable=C0301,E0611,E1101,R0903,R0914

import time
import unittest

import gtsam
import numpy as np
from gtsam import Pose2  # type:ignore
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

from app.field.field_map import FieldMap
from app.pose_estimator.parking_lot.parking_lot import ParkingLot
from tests.pose_estimator.simulation.circle_simulator import CircleSimulator

ACTUALLY_PRINT = False

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([3, 3, 1]))


class EstimateSimulateTest(unittest.TestCase):
    def test_odo_only_custom(self) -> None:
        """Odometry only, using the python custom factor.
        This is very slow, 1.4s on my machine for a 1s window,
        0.2s for a 0.1s window
        """
        sim = CircleSimulator(FieldMap())
        est = ParkingLot()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        # this should just record the positions and timestamp
        est.odometry_custom(0, sim.positions)

        state = gtsam.Pose2()

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
            est.odometry_custom(t1_us, sim.positions)
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

    def test_camera_only(self) -> None:
        """Camera only.
        note we're using the previous estimate as the initial estimate for the
        next state.  if you don't do that (e.g. initial always at origin) then
        it gets really confused, producing big errors and taking a long time.
        this is not fast: 0.9s for a 0.1s window."""
        sim = CircleSimulator(FieldMap())
        est = ParkingLot()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        state = gtsam.Pose2()

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
                # skip out-of-frame iterations
                continue
            est.add_state(t0_us, state)
            est.apriltag_for_smoothing(
                sim.l0, sim.gt_pixels[0], t0_us, sim.camera_offset, sim.calib
            )
            est.apriltag_for_smoothing(
                sim.l1, sim.gt_pixels[1], t0_us, sim.camera_offset, sim.calib
            )
            est.apriltag_for_smoothing(
                sim.l2, sim.gt_pixels[2], t0_us, sim.camera_offset, sim.calib
            )
            est.apriltag_for_smoothing(
                sim.l3, sim.gt_pixels[3], t0_us, sim.camera_offset, sim.calib
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

            print(
                f"{t:7.4f}, {gt_x:7.4f}, {gt_y:7.4f}, {gt_theta:7.4f}, {est_x:7.4f}, {est_y:7.4f}, {est_theta:7.4f}, {err_x:7.4f}, {err_y:7.4f}, {err_theta:7.4f}"
            )

    def test_camera_and_odometry(self) -> None:
        """Camera and odometry.
        with lots of noise, the estimator
        guesses the mirror image path
        somehow (-y instead of y, more rot to compensate).
        tightening up the noise model fixes it.
        also using the previous state as the estimate for the next state
        fixes it."""
        sim = CircleSimulator(FieldMap())
        est = ParkingLot()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
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

            if len(sim.gt_pixels) > 0:
                est.apriltag_for_smoothing(
                    sim.l0, sim.gt_pixels[0], t1_us, sim.camera_offset, sim.calib
                )
                est.apriltag_for_smoothing(
                    sim.l1, sim.gt_pixels[1], t1_us, sim.camera_offset, sim.calib
                )
                est.apriltag_for_smoothing(
                    sim.l2, sim.gt_pixels[2], t1_us, sim.camera_offset, sim.calib
                )
                est.apriltag_for_smoothing(
                    sim.l3, sim.gt_pixels[3], t1_us, sim.camera_offset, sim.calib
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

            print(
                f"{t:7.4f}, {gt_x:7.4f}, {gt_y:7.4f}, {gt_theta:7.4f}, {est_x:7.4f}, {est_y:7.4f}, {est_theta:7.4f}, {err_x:7.4f}, {err_y:7.4f}, {err_theta:7.4f}"
            )

    def test_camera_and_odometry_and_gyro(self) -> None:
        """Camera and odometry and gyro.
        This is very slow, 1.3s to run 2s of data with a 0.1s window.
        I think this means these factors need to be written in C++.
        Somewhat surprising, this actually does a pretty good job without
        a lag window at all, i.e. lag of 0.001 s, so just one state, and
        it runs in about 4x real time (0.5s for 2s of samples).
        """
        sim = CircleSimulator(FieldMap())
        est = ParkingLot()
        est.init()

        # use a good guess for initial state, to eliminate the transient error.
        # prior_mean = gtsam.Pose2(sim.gt_x, sim.gt_y, sim.gt_theta)
        # est.prior(0, prior_mean, PRIOR_NOISE)

        # or use a very high noise level to say "ignore this prior"
        prior_mean = gtsam.Pose2(0, 0, 0)
        est.prior(0, prior_mean, noiseModel.Diagonal.Sigmas(np.array([100, 100, 100])))
        est.add_state(0, prior_mean)

        state = gtsam.Pose2()
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

        # this should just record the positions and timestamp
        est.odometry(0, sim.positions, odometry_noise)

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
                est.apriltag_for_smoothing(
                    sim.l0, sim.gt_pixels[0], t1_us, sim.camera_offset, sim.calib
                )
                est.apriltag_for_smoothing(
                    sim.l1, sim.gt_pixels[1], t1_us, sim.camera_offset, sim.calib
                )
                est.apriltag_for_smoothing(
                    sim.l2, sim.gt_pixels[2], t1_us, sim.camera_offset, sim.calib
                )
                est.apriltag_for_smoothing(
                    sim.l3, sim.gt_pixels[3], t1_us, sim.camera_offset, sim.calib
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

            print(
                f"{t:7.4f}, {gt_x:7.4f}, {gt_y:7.4f}, {gt_theta:7.4f}, {est_x:7.4f}, {est_y:7.4f}, {est_theta:7.4f}, {err_x:7.4f}, {err_y:7.4f}, {err_theta:7.4f}"
            )
