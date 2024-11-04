"""Read measurements from Network Tables, run the
smoother, and publish the results on Network Tables."""

# pylint: disable=C0301,E0611,E1101,R0903,R0914

import math
from typing import cast

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
        self.pose_sender = net.get_pose_sender("pose")
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
        # print("NTEstimate.step() sights ", sights)
        for sight in sights:
            # timestamp in us from the epoch (i.e. a big number)
            timestamp_us = sight[0]
            # print("NTEstimate.step() timestamp ", timestamp_us)
            # discretize at 50 Hz
            time_slice = math.ceil(timestamp_us/20000) * 20000
            # print("NTEstimate.step() time slice ", time_slice)
            blip_list = sight[1]
            for blip in blip_list:
                pixels = blip.measurement()
                corners = self.field_map.get(blip.tag_id)
                self.est.add_state(time_slice, self.state)
                self.est.apriltag_for_smoothing_batch(
                    corners, pixels, time_slice, cam.camera_offset, cam.calib
                )
        self.est.update()
        print("NTEstimate.step() result ", self.est.result)
        results: tuple[int, gtsam.Pose2, np.ndarray] = cast(tuple[int, gtsam.Pose2, np.ndarray], self.est.get_result())
        poses: list[Pose2d] = []
        # pose = self.est.result.atPose2()
        # self.pose_sender.send()

