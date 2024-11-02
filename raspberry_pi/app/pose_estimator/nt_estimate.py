"""Read measurements from Network Tables, run the
smoother, and publish the results on Network Tables."""

# pylint: disable=C0301,E0611,E1101,R0903,R0914

# TODO: remove gtsam
import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from wpimath.geometry import Pose2d

from app.network.network_protocol import Network
from app.pose_estimator.estimate import Estimate

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class NTEstimate:
    def __init__(self, net: Network) -> None:
        self.net = net
        self.blip_receiver = net.get_blip25_receiver("foo")
        self.est = Estimate()
        # current estimate, used for initial value for next time
        # TODO: remove gtsam types
        self.state = gtsam.Pose2()
        self.est.init()
        prior_mean = gtsam.Pose2(0, 0, 0)
        self.est.add_state(0, prior_mean)
        self.est.prior(0, prior_mean, PRIOR_NOISE)

    def step(self) -> None:
        """Collect any pending measurements from
        the network and add them to the sim."""
        frames = self.blip_receiver.get()
        for frame in frames:
            timestamp_us = frame[0]
            blips = frame[1]
            for blip in blips:
                id = blip.id
