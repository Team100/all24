# pylint: disable=W0212


import time
import unittest

import ntcore

from app.config.identity import Identity
from app.network.fake_network import FakeNetwork
from app.network.network_protocol import Blip25
from app.network.real_network import RealNetwork
from app.pose_estimator.estimate import Estimate
from app.pose_estimator.field_map import FieldMap
from app.pose_estimator.nt_estimate import NTEstimate


class NTEstTest(unittest.TestCase):
    def test_real_nt_est(self) -> None:
        print()
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startServer()
        pub = inst.getStructArrayTopic("foo/1", Blip25).publish(
            ntcore.PubSubOptions(keepDuplicates=True)
        )
        field_map = FieldMap()
        net = RealNetwork(Identity.UNKNOWN)
        est = NTEstimate(field_map, net)
        for i in range(10):
            time.sleep(0.02)
            # print("NTEstTest.test_real_nt_est() i ", i)
            time_us = ntcore._now()
            # print("NTEstTest.test_real_nt_est() time_us ", time_us)
            pub.set(
                [
                    Blip25(0, 190, 210, 210, 210, 210, 190, 190, 190),
                    Blip25(0, 190, 210, 210, 210, 210, 190, 190, 190),
                ],
                time_us,
            )
            est.step()

    def test_fake_nt_est(self) -> None:
        field_map = FieldMap()
        net = FakeNetwork()
        est = NTEstimate(field_map, net)
        for _ in range(50):
            est.step()
