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
        new_positions: list[SwerveModuleDelta] = []
        for i in range(len(start)):
            new_positions.append(DriveUtil.delta(start[i], end[i]))

        return new_positions

    @staticmethod
    def delta(
        start: SwerveModulePosition100, end: SwerveModulePosition100
    ) -> SwerveModuleDelta:
        delta_m: float = end.distance_m - start.distance_m
        if end.angle.present:
            return SwerveModuleDelta(delta_m, end.angle)
        # these positions might be null, if the encoder has failed (which can seem to
        # happen if the robot is *severely* overrunning).
        return SwerveModuleDelta(0, OptionalRotation2d(False, Rotation2d(0)))
