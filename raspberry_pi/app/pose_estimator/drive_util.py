# pylint: disable=C0200,R0903

from wpimath.geometry import Rotation2d

from app.pose_estimator.swerve_module_delta import SwerveModuleDelta
from app.pose_estimator.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
)


class DriveUtil:
    @staticmethod
    def module_position_delta(
        start: list[SwerveModulePosition100], end: list[SwerveModulePosition100]
    ) -> list[SwerveModuleDelta]:
        """Uses the end angle to cover the whole interval."""
        deltas: list[SwerveModuleDelta] = []
        for i in range(len(start)):
            deltas.append(DriveUtil._delta(start[i], end[i]))
        return deltas
    
    @staticmethod
    def module_position_from_delta(
        start: list[SwerveModulePosition100], delta: list[SwerveModuleDelta]
    ) -> list [SwerveModulePosition100]:
        new_positions: list[SwerveModulePosition100] = []
        for i in range(len(start)):
            new_positions.append(DriveUtil._plus(start[i], delta[i]))
        return new_positions

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

    def _plus(
            start:SwerveModulePosition100, delta:SwerveModuleDelta) -> SwerveModulePosition100:
        new_distance_m = start.distance_m + delta.distance_m
        if delta.angle.present:
            return  SwerveModulePosition100(new_distance_m, delta.angle)
        # if there's no angle, we're not going anywhere.
        return start
    