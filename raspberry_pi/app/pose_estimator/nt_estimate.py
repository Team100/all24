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
from wpimath.geometry import Pose2d

from app.config.camera_config import CameraConfig
from app.config.identity import Identity
from app.network.network_protocol import Network, PoseEstimate25
from app.pose_estimator.estimate import Estimate
from app.pose_estimator.field_map import FieldMap

PRIOR_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
ODO_NOISE = noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))

# discrete time step is 20 ms
TIME_STEP_US = 20000


class NTEstimate:
    def __init__(self, field_map: FieldMap, net: Network) -> None:
        self.field_map = field_map
        self.net = net
        self.blip_receiver = net.get_blip25_receiver("foo")
        self.odo_receiver = net.get_odometry_receiver("bar")
        self.gyro_receiver = net.get_gyro_receiver("baz")
        self.pose_sender = net.get_pose_sender("pose")
        self.est = Estimate()
        # current estimate, used for initial value for next time
        # TODO: remove gtsam types
        self.state = gtsam.Pose2()
        self.est.init()
        # this timestamp is probably not close to the ones we will
        # receive from the network.
        prior_mean = gtsam.Pose2(0, 0, 0)
        self.est.add_state(0, prior_mean)
        self.est.prior(0, prior_mean, PRIOR_NOISE)

    def step(self) -> None:
        """Collect any pending measurements from
        the network and add them to the sim."""

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

    def _receive_blips(self) -> None:
        """Receive pending blips from the network"""
        # TODO: read the camera identity from the blip
        cam = CameraConfig(Identity.UNKNOWN)
        sights = self.blip_receiver.get()
        # print("NTEstimate.step() sights ", sights)
        for sight in sights:

            time_slice = NTEstimate.discrete(sight[0])
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

    def _receive_odometry(self) -> None:
        odo = self.odo_receiver.get()
        for pos in odo:
            time_slice = NTEstimate.discrete(pos[0])
            # print("TIME SLICE ", time_slice)
            positions = pos[1]
            self.est.add_state(time_slice, self.state)
            self.est.odometry(time_slice, positions, ODO_NOISE)

    def _receive_gyro(self) -> None:
        gyro = self.gyro_receiver.get()
        for g in gyro:
            time_slice = NTEstimate.discrete(g[0])
            yaw = g[1]
            print("TIME", time_slice, "YAW", yaw)
            # if this is the only factor attached to this variable
            # then it will be underconstrained (i.e. no constraint on x or y)
            self.est.add_state(time_slice, self.state)
            self.est.gyro(time_slice, yaw.radians())

    @staticmethod
    def discrete(timestamp_us: int) -> int:
        """Discretize time at 50 Hz"""
        return math.ceil(timestamp_us / TIME_STEP_US) * TIME_STEP_US
