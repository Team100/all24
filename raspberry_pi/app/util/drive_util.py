# pylint: disable=C0200,R0903

from wpimath.geometry import Rotation2d

from app.kinodynamics.swerve_module_delta import SwerveModuleDelta, SwerveModuleDeltas
from app.kinodynamics.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
    SwerveModulePositions,
)


class DriveUtil:
    @staticmethod
    def module_position_delta(
        start: SwerveModulePositions, end: SwerveModulePositions
    ) -> SwerveModuleDeltas:
        """Uses the end angle to cover the whole interval.
        See DriveUtil.java:117"""
        return SwerveModuleDeltas(
            DriveUtil._delta(start.front_left, end.front_left),
            DriveUtil._delta(start.front_right, end.front_right),
            DriveUtil._delta(start.rear_left, end.rear_left),
            DriveUtil._delta(start.rear_right, end.rear_right),
        )

    @staticmethod
    def module_position_from_delta(
        start: SwerveModulePositions, delta: SwerveModuleDeltas
    ) -> SwerveModulePositions:
        """See DriveUtil.java:127"""
        return SwerveModulePositions(
            DriveUtil._plus(start.front_left, delta.front_left),
            DriveUtil._plus(start.front_right, delta.front_right),
            DriveUtil._plus(start.rear_left, delta.rear_left),
            DriveUtil._plus(start.rear_right, delta.rear_right),
        )

    @staticmethod
    def _delta(
        start: SwerveModulePosition100, end: SwerveModulePosition100
    ) -> SwerveModuleDelta:
        delta_m: float = end.distance_m - start.distance_m
        if end.angle.present:
            return SwerveModuleDelta(delta_m, end.angle)
        # these positions might be null, if the encoder has failed (which can seem to
        # happen if the robot is *severely* overrunning).
        return SwerveModuleDelta(0, OptionalRotation2d(False, Rotation2d(0)))

    @staticmethod
    def _plus(
        start: SwerveModulePosition100, delta: SwerveModuleDelta
    ) -> SwerveModulePosition100:
        new_distance_m = start.distance_m + delta.distance_m
        if delta.angle.present:
            return SwerveModulePosition100(new_distance_m, delta.angle)
        # if there's no angle, we're not going anywhere.
        return start
