"""Read measurements from Network Tables, run the
smoother, and publish the results on Network Tables."""

# pylint: disable=C0301,E0611,E1101,R0903,R0914,W0212

import math
from typing import cast

# TODO: remove gtsam
import gtsam
import ntcore
import numpy as np
from gtsam import noiseModel  # type:ignore
from wpimath.geometry import Pose2d

from app.config.camera_config import CameraConfig
from app.config.identity import Identity
from app.network.network_protocol import Network, PoseEstimate25
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
            # TODO: push this list-unpacking into the estimate module
            # so that the network schema and the estimate schema are more
            # similar
            for blip in blip_list:
                pixels = blip.measurement()
                corners = self.field_map.get(blip.tag_id)
                self.est.add_state(time_slice, self.state)
                self.est.apriltag_for_smoothing_batch(
                    corners, pixels, time_slice, cam.camera_offset, cam.calib
                )
        self.est.update()
        print("NTEstimate.step() result ", self.est.result)
        results: tuple[int, gtsam.Pose2, np.ndarray] = cast(
            tuple[int, gtsam.Pose2, np.ndarray], self.est.get_result())
        timestamp = results[0]
        most_recent_estimate = results[1]
        covariance = results[2]
        # TODO: move this somehow
        twist = self.est.measurement
        odo_dt_us = self.est.odo_dt
        # future_estimate = self.est.extrapolate(most_recent_estimate)
        pose_estimate = PoseEstimate25(most_recent_estimate.x(),
                                      most_recent_estimate.y(),
                                      most_recent_estimate.theta(),
                                      math.sqrt(covariance[0,0]),
                                      math.sqrt(covariance[1,1]),
                                      math.sqrt(covariance[2,2]),
                                      twist.dx,
                                      twist.dy,
                                      twist.dtheta,
                                      odo_dt_us
                                      )
        self.pose_sender.send(pose_estimate, ntcore._now() - timestamp)



