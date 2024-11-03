"""Read measurements from Network Tables, run the
smoother, and publish the results on Network Tables."""

# pylint: disable=C0301,E0611,E1101,R0903,R0914

# TODO: remove gtsam
import gtsam
import numpy as np
from gtsam import noiseModel  # type:ignore
from wpimath.geometry import Pose2d

from app.camera.camera_protocol import Camera
from app.config.camera_config import CameraConfig
from app.config.identity import Identity
from app.network.network_protocol import Network
from app.pose_estimator.estimate import Estimate
from app.pose_estimator.field_map import FieldMap

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))


class NTEstimate:
    def __init__(self, field_map: FieldMap, net: Network) -> None:
        self.field_map = field_map
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
        # TODO: read the camera identity from the blip
        cam = CameraConfig(Identity.UNKNOWN)
        sights = self.blip_receiver.get()
        print("sights ", sights)
        for sight in sights:
            timestamp_us = sight[0]
            tag_id = sight[1]
            blip = sight[2]
            pixels = blip.measurement()
            corners = self.field_map.get(tag_id)
            self.est.apriltag_for_smoothing_batch(
                corners, pixels, timestamp_us, cam.camera_offset, cam.calib
            )

