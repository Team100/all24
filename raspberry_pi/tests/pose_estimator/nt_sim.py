"""Publish simulated measurements on Network Tables,
and compare to the estimations from Network Tables.

For now this uses the CircleSimulator only."""

# pylint: disable=R0903

from wpimath.geometry import Rotation2d

from app.field.field_map import FieldMap
from app.network.structs import Blip25
from app.network.network import Network
from tests.pose_estimator.simulation.circle_simulator import CircleSimulator

# TODO: more than one tag
TAG_ID = 0


class NTSim:
    def __init__(self, net: Network) -> None:
        self.net = net
        # TODO: real naming scheme
        self.blip_sender = net.get_blip25_sender("blip25")
        self.odometry_sender = net.get_odometry_sender("odometry")
        self.gyro_sender = net.get_gyro_sender("gyro")
        self.sim = CircleSimulator(FieldMap())

    def step(self, dt_s: float) -> None:
        """Step the simulation dt_s and publish the
        measurements to the network."""
        self.sim.step(dt_s)

        self._send_blips()
        self._send_odometry()
        self._send_gyro()

    def _send_blips(self) -> None:
        p = self.sim.gt_pixels
        # sometimes the tag is out of frame, so there's nothing to send.
        if len(p) == 0:
            return
        b = Blip25(
            TAG_ID,
            p[0][0],
            p[0][1],
            p[1][0],
            p[1][1],
            p[2][0],
            p[2][1],
            p[3][0],
            p[3][1],
        )
        # print("BLIP", b)
        self.blip_sender.send([b], 0)

    def _send_odometry(self) -> None:
        p = self.sim.positions
        # print("ODOMETRY", p)
        self.odometry_sender.send(p, 0)

    def _send_gyro(self) -> None:
        p = self.sim.gt_theta
        # print("GYRO", p)
        self.gyro_sender.send(Rotation2d(p), 0)
