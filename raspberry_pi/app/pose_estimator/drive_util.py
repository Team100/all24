# pylint: disable=C0200,R0903

from wpimath.geometry import Rotation2d, Translation2d

from app.pose_estimator.swerve_module_position import (
    OptionalRotation2d,
    SwerveModulePosition100,
)


class DriveUtil:
    @staticmethod
    def module_position_delta(
        start: list[SwerveModulePosition100], end: list[SwerveModulePosition100]
    ) -> list[SwerveModulePosition100]:

        new_positions: list[SwerveModulePosition100] = []
        for i in range(len(start)):
            start_module: SwerveModulePosition100 = start[i]
            end_module: SwerveModulePosition100 = end[i]
            # these positions might be null, if the encoder has failed (which can seem to
            # happen if the robot is *severely* overrunning).
            delta_m: float = end_module.distance_m - start_module.distance_m
            if start_module.angle.present and end_module.angle.present:
                new_positions.append(
                    SwerveModulePosition100(
                        delta_m,
                        # this change breaks the odometry test on line 66, the 90 degree turn case.
                        # endModule.angle);
                        OptionalRotation2d(
                            True,
                            start_module.angle.value
                            + (end_module.angle.value - start_module.angle.value) * 0.5,
                        ),
                    )
                )
            else:
                new_positions[i] = SwerveModulePosition100(
                    delta_m, OptionalRotation2d(False, Rotation2d(0))
                )

        return new_positions
