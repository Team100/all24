# pylint: disable=C0301,E0611,E1101,R0903,R0914


import time
import unittest

import gtsam
import numpy as np
from gtsam import Pose2  # type:ignore
from gtsam import noiseModel  # type:ignore
from gtsam.symbol_shorthand import X  # type:ignore

from app.pose_estimator.parking_lot.parking_lot import ParkingLot
from tests.pose_estimator.simulation.line_simulator import LineSimulator

ACTUALLY_PRINT = False

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class EstimateSimulateAccelTest(unittest.TestCase):

    def test_accel_only(self) -> None:
        """Acceleration only.
        This factor is surprisingly slow, and maybe not that useful?
        """
        sim = LineSimulator()
        est = ParkingLot()
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
                print(f"{et/1e9} {est._result.size()}")
            t = i * 0.02
            gt_x = sim.gt_x
            gt_y = sim.gt_y
            gt_theta = sim.gt_theta

            # using just odometry without noise, the error
            # is exactly zero, all the time. :-)
            p: Pose2 = est._result.atPose2(X(t2_us))
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