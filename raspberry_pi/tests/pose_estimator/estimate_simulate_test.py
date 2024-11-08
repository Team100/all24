# pylint: disable=C0301,E0611,E1101,R0903,R0914

import time
import unittest

import gtsam
import numpy as np
from gtsam import Pose2  # type:ignore
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

from app.pose_estimator.estimate import Estimate
from app.pose_estimator.field_map import FieldMap
from tests.pose_estimator.circle_simulator import CircleSimulator
from tests.pose_estimator.line_simulator import LineSimulator

ACTUALLY_PRINT = False

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class EstimateSimulateTest(unittest.TestCase):
    def test_odo_only(self) -> None:
        """Odometry only, using the native factor.
        This is very fast, 0.1s on my machine for a 1s window,
        0.03s for a 0.1s window"""
        sim = CircleSimulator(FieldMap())
        est = Estimate()
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
            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.result.atPose2(X(t1_us))
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

    def test_odo_only_custom(self) -> None:
        """Odometry only, using the python custom factor.
        This is very slow, 1.4s on my machine for a 1s window,
        0.2s for a 0.1s window
        """
        sim = CircleSimulator(FieldMap())
        est = Estimate()
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
                print(f"{et/1e9} {est.result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.result.atPose2(X(t1_us))
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

    def test_accel_only(self) -> None:
        """Acceleration only.
        This factor is surprisingly slow, and maybe not that useful?
        """
        sim = LineSimulator()
        est = Estimate()
        # adds a state at zero
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        state = gtsam.Pose2()

        # for accel we need another state
        sim.step(0.02)
        est.add_state(20000, state)
        est.prior(20000, gtsam.Pose2(0.0002, 0, 0), PRIOR_NOISE)

        print()
        print(
            "      t,    GT X,    GT Y,  GT ROT,   EST X,   EST Y, EST ROT,   ERR X,   ERR Y, ERR ROT"
        )

        for i in range(2, 100):
            t0 = time.time_ns()
            t0_us = 20000 * (i - 2)
            t1_us = 20000 * (i - 1)
            t2_us = 20000 * i
            # updates gt to t2
            sim.step(0.02)
            est.add_state(t2_us, state)
            # TODO: don't want a prior here
            # but without it, the system is underdetermined
            est.prior(t2_us, gtsam.Pose2(sim.gt_x, sim.gt_y, sim.gt_theta), PRIOR_NOISE)
            # est.odometry(t0_us, t1_us, sim.positions)
            est.accelerometer(t0_us, t1_us, t2_us, sim.gt_ax, sim.gt_ay)
            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.result.atPose2(X(t2_us))
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
        est = Estimate()
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
                print(f"{et/1e9} {est.result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.result.atPose2(X(t0_us))
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
        this is much faster than multiple factors, 0.4s for a 0.1s window"""
        sim = CircleSimulator(FieldMap())
        est = Estimate()
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
            est.add_state(t0_us, state)

            all_pixels = np.concatenate(sim.gt_pixels)
            est.apriltag_for_smoothing_batch(
                sim.landmarks, all_pixels, t0_us, sim.camera_offset, sim.calib
            )

            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.result.atPose2(X(t0_us))
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
        est = Estimate()
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
                print(f"{et/1e9} {est.result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.result.atPose2(X(t1_us))
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
        est = Estimate()
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
                print(f"{et/1e9} {est.result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.result.atPose2(X(t1_us))
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

    def test_batched_camera_and_odometry_and_gyro(self) -> None:
        """Camera and odometry and gyro.
        Batching vision speeds things up significantly,
        0.6s for 2s of data in a 0.1s window
        """
        sim = CircleSimulator(FieldMap())
        est = Estimate()
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, noiseModel.Diagonal.Sigmas(np.array([100, 100, 100])))
        
        odometry_noise = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
        # this should just record the positions and timestamp
        est.odometry(0, sim.positions, odometry_noise)

        state = gtsam.Pose2()

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

            all_pixels = np.concatenate(sim.gt_pixels)

            est.apriltag_for_smoothing_batch(
                sim.landmarks, all_pixels, t1_us, sim.camera_offset, sim.calib
            )

            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est.result.atPose2(X(t1_us))
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
