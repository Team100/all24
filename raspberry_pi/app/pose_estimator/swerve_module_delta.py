import math

from wpimath.geometry import Rotation2d

from app.pose_estimator.swerve_module_position import OptionalRotation2d

# pylint: disable=C0200,R0903


class SwerveModuleDelta:
    def __init__(self, distance_m: float, angle: OptionalRotation2d) -> None:
        self.distance_m = distance_m
        self.angle = angle

    @staticmethod
    def of(dx: float, dy: float) -> "SwerveModuleDelta":
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            # avoid the garbage rotation.
            return SwerveModuleDelta(0.0, OptionalRotation2d(False, Rotation2d(0)))
        return SwerveModuleDelta(
            math.hypot(dx, dy), OptionalRotation2d(True, Rotation2d(dx, dy))
        )
