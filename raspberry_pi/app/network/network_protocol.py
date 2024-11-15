""" Interface for Network Tables-like things. """

# pylint: disable=R0902,R0903,W0212,W2301

import dataclasses
from typing import Protocol

import numpy as np
from wpimath.geometry import (Pose2d, Pose3d, Rotation2d, Rotation3d,
                              Transform3d)
from wpiutil import wpistruct

from app.kinodynamics.swerve_module_position import SwerveModulePositions

F = ".3f"


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class Blip24:
    """AprilTag target pose used in 2024"""

    id: int
    pose: Transform3d


@wpistruct.make_wpistruct  # type:ignore
@dataclasses.dataclass
class Blip25:
    """AprilTag target for 2025, includes pixel coordinates.
    the struct concept only supports fixed-length records
    so i can't use an array here. it's ok because a tag only
    ever has exactly 4 corners; if one is occluded then the
    entire tag is unseen.
    """

    tag_id: int
    llx: float  # lower left
    lly: float
    lrx: float  # lower right
    lry: float
    urx: float  # upper right
    ury: float
    ulx: float  # upper left
    uly: float

    def measurement(self) -> np.ndarray:
        """Concatenated corners, for GTSAM."""
        return np.array(
            [
                self.llx,
                self.lly,
                self.lrx,
                self.lry,
                self.urx,
                self.ury,
                self.ulx,
                self.uly,
            ]
        )


@wpistruct.make_wpistruct
@dataclasses.dataclass
class PoseEstimate25:
    """Result of the pose estimator."""

    # most-recent state (corresponding to the NT timestamp)
    # TODO: make this a pose2d
    x: float
    y: float
    theta: float
    # std dev of most-recent state (sqrt of diagonal of marginal covariance)
    # TODO: make this a twist2d
    x_sigma: float
    y_sigma: float
    theta_sigma: float
    # twist of most-recent odometry
    # TODO: make this a twist2d
    dx: float
    dy: float
    dtheta: float
    # time between next-most-recent and most-recent
    dt: float

    def __str__(self) -> str:
        return (
            f"(x {self.x:{F}} y {self.y:{F}} Θ {self.theta:{F}} "
            f"sx {self.x_sigma:{F}} sy {self.y_sigma:{F}} sΘ {self.theta_sigma:{F}} "
            f"dx {self.dx:{F}} dy {self.dy:{F}} dΘ {self.dtheta:{F}} "
            f"dt {self.dt:{F}})"
        )


@wpistruct.make_wpistruct
@dataclasses.dataclass
class Cal3DS2:
    """Camera parameters mirroring gtsam.Cal3DS2.
    see Cal3DS2_Base.h and Cal3.h"""

    # focal length
    fx: float
    fy: float
    # skew
    s: float
    #  principal (i.e. center) point
    u0: float
    v0: float
    # radial 2nd-order and 4th-order
    k1: float
    k2: float
    #  tangential distortion
    p1: float
    p2: float


@wpistruct.make_wpistruct
@dataclasses.dataclass
class MyTwist3d:
    """This works around the missing Twist3d on the RPi's at the moment"""

    dx: float
    dy: float
    dz: float
    rx: float
    ry: float
    rz: float


@wpistruct.make_wpistruct
@dataclasses.dataclass
class CameraCalibration:
    camera_offset: Pose3d
    offset_sigma: MyTwist3d
    calib: Cal3DS2
    calib_sigma: Cal3DS2

    def __str__(self) -> str:
        return (
            f"(x {self.camera_offset.x:{F}} y {self.camera_offset.y:{F}} z {self.camera_offset.z:{F}} "
            f"rx {self.camera_offset.rotation().x:{F}} ry {self.camera_offset.rotation().y:{F}} rz {self.camera_offset.rotation().z:{F}} "
            f"dx {self.offset_sigma.dx:{F}} dy {self.offset_sigma.dy:{F}} dz {self.offset_sigma.dz:{F}} "
            f"drx {self.offset_sigma.rx:{F}} dry {self.offset_sigma.ry:{F}} drz {self.offset_sigma.rz:{F}} "
            f"fx {self.calib.fx:{F}} fy {self.calib.fy:{F}} cx {self.calib.u0:{F}} cy {self.calib.v0:{F}} k1 {self.calib.k1:{F}} k2 {self.calib.k2:{F}} "
            f"dfx {self.calib_sigma.fx:{F}} dfy {self.calib_sigma.fy:{F}} dcx {self.calib_sigma.u0:{F}} dcy {self.calib_sigma.v0:{F}} dk1 {self.calib_sigma.k1:{F}} dk2 {self.calib_sigma.k2:{F}})"
        )


class DoubleSender(Protocol):
    def send(self, val: float, delay_us: int) -> None: ...


class BlipSender(Protocol):
    def send(self, val: list[Blip24], delay_us: int) -> None: ...


class NoteSender(Protocol):
    def send(self, val: list[Rotation3d], delay_us: int) -> None: ...


class Blip25Sender(Protocol):
    def send(self, val: list[Blip25], delay_us: int) -> None:
        """This is used by the simulator, and by the cameras."""
        ...


class Blip25Receiver(Protocol):
    def get(self) -> list[tuple[int, list[Blip25]]]:
        """Receive the list of tuples (timetamp, list[blip]) seen in a single frame"""
        ...


class PoseSender(Protocol):
    def send(self, val: PoseEstimate25, delay_us: int) -> None: ...


class OdometrySender(Protocol):
    def send(self, val: SwerveModulePositions, delay_us: int) -> None: ...


class OdometryReceiver(Protocol):
    def get(self) -> list[tuple[int, SwerveModulePositions]]:
        """Receive a list of tuples (timestamp, positions)"""
        ...


class GyroSender(Protocol):
    def send(self, val: Rotation2d, delay_us: int) -> None: ...


class GyroReceiver(Protocol):
    def get(self) -> list[tuple[int, Rotation2d]]:
        """Receive a list of tuples (timestamp, yaw)"""
        ...


class CalibSender(Protocol):
    # TODO: we don't care about timestamp here so remove it
    def send(self, val: CameraCalibration, delay_us: int) -> None: ...


class PriorReceiver(Protocol):
    """Force a pose."""
    def get(self) -> Pose2d | None: ...


class Network(Protocol):
    def now(self) -> int: ...
    def get_double_sender(self, name: str) -> DoubleSender: ...
    def get_blip_sender(self, name: str) -> BlipSender: ...
    def get_note_sender(self, name: str) -> NoteSender: ...
    def get_blip25_sender(self, name: str) -> Blip25Sender: ...
    def get_blip25_receiver(self, name: str) -> Blip25Receiver: ...
    def get_pose_sender(self, name: str) -> PoseSender: ...
    def get_odometry_sender(self, name: str) -> OdometrySender: ...
    def get_odometry_receiver(self, name: str) -> OdometryReceiver: ...
    def get_gyro_sender(self, name: str) -> GyroSender: ...
    def get_gyro_receiver(self, name: str) -> GyroReceiver: ...
    def get_calib_sender(self, name: str) -> CalibSender: ...
    def get_prior_receiver(self, name: str) -> PriorReceiver: ...
    def flush(self) -> None: ...
