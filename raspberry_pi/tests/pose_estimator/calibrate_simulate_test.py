# pylint: disable=C0301,E0611,E1101,R0903,R0914

import time
import unittest

import gtsam
import numpy as np
from gtsam import Pose2  # type:ignore
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import C, K, X  # type:ignore

from app.pose_estimator.calibrate import Calibrate
from app.field.field_map import FieldMap
from tests.pose_estimator.simulation.circle_simulator import CircleSimulator

ACTUALLY_PRINT = False

# pose prior
PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([2, 2, 2]))

# real odometry is quite accurate (1cm here)
ODOMETRY_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))



class CalibrateSimulateTest(unittest.TestCase):
    def test_odo_only(self) -> None:
        """Odometry only, using the native factor.
        This is very fast, 0.1s on my machine for a 1s window,
        0.03s for a 0.1s window"""
        sim = CircleSimulator(FieldMap())
        est = Calibrate(0.1)
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
        est = Calibrate(0.1)
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
        this is much faster than multiple factors, 0.4s for a 0.1s window
        Note the solver here is depending *only* on vision input, which means
        the tag geometry itself is the signal.  It's kinda surprising that
        this works at all.  It would work better with tags spaced further
        apart, and/or multiple cameras, or, as below, with odometry.
        It takes a long time to learn the fx and fy values, like >20s.
        but k1 and k2 seem pretty close within 10s.
        For the 1000-iteration (20s) case with the python batched factor,
        runtime is about 20s (i.e. 100% realtime, totally not ok).
        For the c++ not-batched factor the runtime is about 5s (still
        not great).
        """
        np.set_printoptions(suppress=True, precision=2)

        sim = CircleSimulator(FieldMap())
        # larger window makes each step quite a bit slower
        # but it doesn't seem to change the convergence rate per step.
        # a very short window also seems to hurt the convergence rate.
        est = Calibrate(0.1)
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        state = gtsam.Pose2()

        if ACTUALLY_PRINT:
            print()
            print(
                "     t, "
                "  GT X,   GT Y,   GT Θ, "
                " Est X,  Est Y,  Est Θ, "
                " sig X,  sig Y,  sig Θ, "
                " Err X,  Err Y,  Err Θ, "
                "   F_X,    F_Y,    P_X, "
                "   P_Y,    K_0,    K_1, "
                "   C_X,    C_Y,    C_Z, "
                "   C_R,    C_P,    C_Y"
            )
        for i in range(1, 1000):
            t0 = time.time_ns()
            t0_us = 20000 * i
            # updates gt to t0
            sim.step(0.02)

            if len(sim.gt_pixels) == 0:
                # print("out of frame")
                continue

            est.add_state(t0_us, state)
            all_pixels = np.concatenate(sim.gt_pixels)

            # print(sim.landmarks)
            # print(all_pixels)

            est.apriltag_for_calibration_batch(sim.landmarks, all_pixels, t0_us)
            est.keep_calib_hot(t0_us)

            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result_size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            p: Pose2 = est.mean_pose2(X(t0_us))
            # use the previous estimate as the new estimate.
            state = p
            est_x = p.x()
            est_y = p.y()
            est_theta = p.theta()

            err_x = est_x - gt_x
            err_y = est_y - gt_y
            err_theta = est_theta - gt_theta

            # the sigma values are very large (over 1m) even when the actual error
            # is very small (10 cm), when the target is the furthest away, "square"
            # to the camera, so the uncertainty in yaw is maximum.
            # this matches the intuition that it's hard (even for a human) to tell
            # in this situation.
            s0 = est.sigma(X(t0_us))

            c0 = est.mean_pose3(C(0))

            k0 = est.mean_cal3DS2(K(0))

            f = '6.2f'

            if ACTUALLY_PRINT:
                print(
                    f"{t:{f}}, "
                    f"{gt_x:{f}}, {gt_y:{f}}, {gt_theta:{f}}, "
                    f"{est_x:{f}}, {est_y:{f}}, {est_theta:{f}}, "
                    f"{s0[0]:{f}}, {s0[1]:{f}}, {s0[2]:{f}}, "
                    f"{err_x:{f}}, {err_y:{f}}, {err_theta:{f}}, "
                    f"{k0.fx():{f}}, {k0.fy():{f}}, {k0.px():{f}}, "
                    f"{k0.py():{f}}, {k0.k1():{f}}, {k0.k2():{f}}, "
                    f"{c0.x():{f}}, {c0.y():{f}}, {c0.z():{f}}, "
                    f"{c0.rotation().roll():{f}}, {c0.rotation().pitch():{f}}, {c0.rotation().yaw():{f}}"
                )

    def test_batched_camera_and_odometry_and_gyro(self) -> None:
        """Camera and odometry and gyro.
        Adding odometry removes the large sigma(y) at the far-and-square
        pose (mentioned above); sigmas are now a few cm.
        Adding the gyro locks the theta error to zero.
        It takes a long time to learn fx and fy, as the model above also does.
        k1 and k2 are pretty close within 2 sec (!)
        The prediction errors are below 1cm and 0.5 deg all the time.
        for the python batched factor for 20s, the runtime is about 15s
        for the c++ not-batched factor for 20s, the runtime is about 1.3s
        (much better)
        """
        sim = CircleSimulator(FieldMap())
        est = Calibrate(0.1)
        est.init()

        prior_mean = gtsam.Pose2(0, 0, 0)
        est.add_state(0, prior_mean)
        est.prior(0, prior_mean, PRIOR_NOISE)

        # this should just record the positions and timestamp
        est.odometry(0, sim.positions, ODOMETRY_NOISE)

        state = gtsam.Pose2()

        if ACTUALLY_PRINT:
            print()
            print(
                "     t, "
                "  GT X,   GT Y,   GT Θ, "
                " Est X,  Est Y,  Est Θ, "
                " sig X,  sig Y,  sig Θ, "
                " Err X,  Err Y,  Err Θ, "
                "   F_X,    F_Y,    P_X, "
                "   P_Y,    K_0,    K_1, "
                "   C_X,    C_Y,    C_Z, "
                "   C_R,    C_P,    C_Y"
            )
        for i in range(1, 1000):
            t0 = time.time_ns()
            t0_us = 20000 * i
            # updates gt to t0
            sim.step(0.02)

            est.add_state(t0_us, state)

            est.odometry(t0_us, sim.positions, ODOMETRY_NOISE)
            
            # the gyro factor really confuses the model. (??)
            est.gyro(t0_us, sim.gt_theta)


            if len(sim.gt_pixels) > 0:
                all_pixels = np.concatenate(sim.gt_pixels)
                est.apriltag_for_calibration_batch(sim.landmarks, all_pixels, t0_us)

            est.keep_calib_hot(t0_us)
            est.update()
            t1 = time.time_ns()
            et = t1 - t0
            if ACTUALLY_PRINT:
                print(f"{et/1e9} {est.result_size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            p: Pose2 = est.mean_pose2(X(t0_us))
            # use the previous estimate as the new estimate.
            state = p
            est_x = p.x()
            est_y = p.y()
            est_theta = p.theta()

            err_x = est_x - gt_x
            err_y = est_y - gt_y
            err_theta = est_theta - gt_theta

            s0 = est.sigma(X(t0_us))

            c0 = est.mean_pose3(C(0))

            k0 = est.mean_cal3DS2(K(0))

            f = '6.2f'

            if ACTUALLY_PRINT:
                print(
                    f"{t:{f}}, "
                    f"{gt_x:{f}}, {gt_y:{f}}, {gt_theta:{f}}, "
                    f"{est_x:{f}}, {est_y:{f}}, {est_theta:{f}}, "
                    f"{s0[0]:{f}}, {s0[1]:{f}}, {s0[2]:{f}}, "
                    f"{err_x:{f}}, {err_y:{f}}, {err_theta:{f}}, "
                    f"{k0.fx():{f}}, {k0.fy():{f}}, {k0.px():{f}}, "
                    f"{k0.py():{f}}, {k0.k1():{f}}, {k0.k2():{f}}, "
                    f"{c0.x():{f}}, {c0.y():{f}}, {c0.z():{f}}, "
                    f"{c0.rotation().roll():{f}}, {c0.rotation().pitch():{f}}, {c0.rotation().yaw():{f}}"
                )
