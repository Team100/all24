"""Publish simulated measurements on Network Tables,
and compare to the estimations from Network Tables.

For now this uses the CircleSimulator only."""

from app.network.network_protocol import Blip25, Network
from tests.pose_estimator.circle_simulator import CircleSimulator


class NTSim:
    def __init__(self, net: Network) -> None:
        self.net = net
        self.blip_sender = net.get_blip25_sender("foo")
        self.sim = CircleSimulator()

    def step(self, dt_s: float) -> None:
        """Step the simulation dt_s and publish the
        measurements to the network."""
        self.sim.step(dt_s)
        p = self.sim.gt_pixels
        b = Blip25(
            1,
            p[0][0],
            p[0][1],
            p[1][0],
            p[1][1],
            p[2][0],
            p[2][1],
            p[3][0],
            p[3][1],
        )
        self.blip_sender.send([b], 0)
