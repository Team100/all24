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
        pub = inst.getStructTopic("foo/1", Blip25).publish()
        field_map = FieldMap()
        net = RealNetwork(Identity.UNKNOWN)
        est = NTEstimate(field_map, net)
        for i in range(10):
            time.sleep(1)
            print("set ", i)
            pub.set(
                Blip25(190, 210, 210, 210, 210, 190, 190, 190),
                ntcore._now(),
            )
            # t = inst.getTopics()
            # print("0 ", t[0].getName())
            # print("1 ", t[1].getName())
            # print(inst.getStructArrayTopic("foo", Blip25).subscribe([]).get())
            est.step()

    def test_fake_nt_est(self) -> None:
        field_map = FieldMap()
        net = FakeNetwork()
        est = NTEstimate(field_map, net)
        for _ in range(50):
            est.step()
