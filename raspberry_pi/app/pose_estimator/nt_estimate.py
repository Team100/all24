"""Read measurements from Network Tables, run the
smoother, and publish the results on Network Tables."""

# pylint: disable=C0301,E0611,E1101,R0902,R0903,R0914,W0212

import math
from typing import cast

# TODO: remove gtsam
import gtsam
import ntcore
import numpy as np
from gtsam import noiseModel  # type:ignore

from app.config.camera_config import CameraConfig
from app.field.field_map import FieldMap
from app.network.structs import PoseEstimate25
from app.network.network import Network
from app.pose_estimator import util
from app.pose_estimator.estimate import Estimate

# TODO: consolidate with estimate.py.
PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([160, 80, 60]))
PRIOR_MEAN = gtsam.Pose2(8, 4, 0)

ODO_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))


class NTEstimate:
    def __init__(self, field_map: FieldMap, cam: CameraConfig, net: Network) -> None:
        self.field_map = field_map
        self.cam = cam
        self.net = net
        self.prior_receiver = net.get_prior_receiver("prior")
        self.blip_receiver = net.get_blip25_receiver("blip25")
        self.odo_receiver = net.get_odometry_receiver("odometry")
        self.gyro_receiver = net.get_gyro_receiver("gyro")
        self.pose_sender = net.get_pose_sender("pose")

        self.est = Estimate(0.1)
        # current estimate, used for initial value for next time
        # TODO: remove gtsam types
        self.state = gtsam.Pose2()
        self.est.init()
        # this timestamp is probably not close to the ones we will
        # receive from the network.
        self.est.add_state(0, PRIOR_MEAN)
        self.est.prior(0, PRIOR_MEAN, PRIOR_NOISE)

    def step(self) -> None:
        """Collect any pending measurements from
        the network and add them to the sim."""

        self._receive_prior()
        self._receive_blips()
        self._receive_odometry()
        self._receive_gyro()

        self.est.update()

        # print("NTEstimate.step() result ", self.est.result)
        results: tuple[int, gtsam.Pose2, np.ndarray] = cast(
            tuple[int, gtsam.Pose2, np.ndarray], self.est.get_result()
        )

        timestamp = results[0]
        most_recent_estimate = results[1]
        covariance = results[2]

        self.state = most_recent_estimate

        # TODO: move this somehow
        twist = self.est.measurement
        odo_dt_us = self.est.odo_dt
        # future_estimate = self.est.extrapolate(most_recent_estimate)
        pose_estimate = PoseEstimate25(
            most_recent_estimate.x(),
            most_recent_estimate.y(),
            most_recent_estimate.theta(),
            math.sqrt(covariance[0, 0]),
            math.sqrt(covariance[1, 1]),
            math.sqrt(covariance[2, 2]),
            twist.dx,
            twist.dy,
            twist.dtheta,
            odo_dt_us,
        )
        self.pose_sender.send(pose_estimate, ntcore._now() - timestamp)

    def _receive_prior(self) -> None:
        """If we ever receive a prior message, we restart the whole model."""
        prior = self.prior_receiver.get()
        if prior is None:
            return

        # restart the whole thing
        p = util.pose2d_to_pose2(prior)
        # TODO: supply sigma on the wire?
        s = noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.05]))

        self.est = Estimate(0.1)
        self.state = p
        self.est.init()
        now = self.net.now()
        self.est.add_state(now, p)
        self.est.prior(now, p, s)

    def _receive_blips(self) -> None:
        """Receive pending blips from the network"""
        # TODO: read the camera identity from the blip
        # cam = CameraConfig(Identity.UNKNOWN)
        sights = self.blip_receiver.get()
        # print("NTEstimate.step() sights ", sights)
        for sight in sights:

            time_slice = util.discrete(sight[0])
            blip_list = sight[1]

            # TODO: push this list-unpacking into the estimate module
            # so that the network schema and the estimate schema are more
            # similar
            for blip in blip_list:
                # print("TIME", time_slice, "BLIP", blip)
                pixels = blip.measurement()
                corners = self.field_map.get(blip.tag_id)
                self.est.add_state(time_slice, self.state)
                self.est.apriltag_for_smoothing_batch(
                    corners, pixels, time_slice, self.cam.camera_offset, self.cam.calib
                )

    def _receive_odometry(self) -> None:
        odo = self.odo_receiver.get()
        for pos in odo:
            time_slice = util.discrete(pos[0])
            # print("TIME SLICE ", time_slice)
            positions = pos[1]
            # print("TIME", time_slice, "ODO", positions)
            self.est.add_state(time_slice, self.state)
            self.est.odometry(time_slice, positions, ODO_NOISE)

    def _receive_gyro(self) -> None:
        gyro = self.gyro_receiver.get()
        for g in gyro:
            time_slice = util.discrete(g[0])
            yaw = g[1]
            # print("TIME", time_slice, "YAW", yaw)
            # if this is the only factor attached to this variable
            # then it will be underconstrained (i.e. no constraint on x or y)
            self.est.add_state(time_slice, self.state)
            self.est.gyro(time_slice, yaw.radians())
