"""SwerveDriveKinematics100"""

# pylint: disable=C0301,E0611,R0903
import numpy as np
import numpy.typing as npt
from wpimath.geometry import Translation2d, Twist2d

from app.util.drive_util import DriveUtil
from app.kinodynamics.swerve_module_delta import (SwerveModuleDelta,
                                                    SwerveModuleDeltas)
from app.kinodynamics.swerve_module_position import SwerveModulePositions


class SwerveDriveKinematics100:
    """
    * array order:
    *
    * frontLeft
    * frontRight
    * rearLeft
    * rearRight
    """

    def __init__(self, module_translations_m: list[Translation2d]) -> None:
        self.num_modules = len(module_translations_m)
        self.module_locations = module_translations_m
        self.inverse_kinematics: npt.NDArray[np.float64] = self._inverse_matrix(
            self.module_locations
        )
        self.forward_kinematics = np.linalg.pinv(self.inverse_kinematics)

    def to_twist_2d(self, deltas: SwerveModuleDeltas) -> Twist2d:
        """FORWARD: module deltas -> twist."""
        # [d cos; d sin; ...] (2n x 1)
        delta_vector: npt.NDArray[np.float64] = self._deltas_2_vector(deltas)
        # [dx ;dy; dtheta]
        twist_vector: npt.NDArray[np.float64] = np.matmul(
            self.forward_kinematics, delta_vector
        )
        return self._vector_2_twist(twist_vector)

    def to_swerve_module_delta(self, twist: Twist2d) -> SwerveModuleDeltas:
        """
        INVERSE: twist -> module position deltas

        This assumes the wheel paths are geodesics; steering does not change.
        """

        # [dx; dy; dtheta] (3 x 1)
        twist_vector = self.twist_2_vector(twist)
        # [d cos; d sin; ...] (2n x 1)
        delta_vector = np.matmul(self.inverse_kinematics, twist_vector)
        return self.deltas_from_vector(delta_vector)

    def to_swerve_module_positions(
        self, initial: SwerveModulePositions, twist: Twist2d
    ) -> SwerveModulePositions:
        """Find the module deltas and apply them to the given initial positions.
        See SwerveDriveKinematics100.java:159"""
        deltas: SwerveModuleDeltas = self.to_swerve_module_delta(twist)
        return DriveUtil.module_position_from_delta(initial, deltas)

    @staticmethod
    def twist_2_vector(twist: Twist2d) -> npt.NDArray[np.float64]:
        return np.array([[twist.dx], [twist.dy], [twist.dtheta]])

    def deltas_from_vector(
        self, module_delta_vector: npt.NDArray[np.float64]
    ) -> SwerveModuleDeltas:
        """
        The resulting distance is always positive.

        @param moduleDeltaVector [d cos; d sin; ...] (2n x 1),
        equivalently [dx0; dy0; dx1; ...]
        See SwerveDriveKinematics100.java:402
        """
        return SwerveModuleDeltas(
            SwerveModuleDelta.of(module_delta_vector[0, 0], module_delta_vector[1, 0]),
            SwerveModuleDelta.of(module_delta_vector[2, 0], module_delta_vector[3, 0]),
            SwerveModuleDelta.of(module_delta_vector[4, 0], module_delta_vector[5, 0]),
            SwerveModuleDelta.of(module_delta_vector[6, 0], module_delta_vector[7, 0]),
        )

    def _deltas_2_vector(
        self, module_deltas: SwerveModuleDeltas
    ) -> npt.NDArray[np.float64]:
        """deltas -> [d cos; d sin; ... ] (2n x 1)
        See SwerveDriveKinematics100.java:292
        """
        module_delta_matrix = np.zeros((self.num_modules * 2, 1))
        module_deltas_all = module_deltas.all()
        for i in range(self.num_modules):
            module: SwerveModuleDelta = module_deltas_all[i]
            if abs(module.distance_m) < 1e-6 or not module.angle.present:
                module_delta_matrix[i * 2, 0] = 0
                module_delta_matrix[i * 2 + 1, 0] = 0
            else:
                module_delta_matrix[i * 2, 0] = (
                    module.distance_m * module.angle.value.cos()
                )
                module_delta_matrix[i * 2 + 1, 0] = (
                    module.distance_m * module.angle.value.sin()
                )

        return module_delta_matrix

    @staticmethod
    def _vector_2_twist(v: npt.NDArray[np.float64]) -> Twist2d:
        """[dx; dy; dtheta] (3 x 1) -> Twist2d"""
        return Twist2d(v[0, 0], v[1, 0], v[2, 0])

    @staticmethod
    def _inverse_matrix(
        module_locations: list[Translation2d],
    ) -> npt.NDArray[np.float64]:
        num_modules = len(module_locations)
        inverse_kinematics = np.zeros((num_modules * 2, 3))
        for i in range(num_modules):
            inverse_kinematics[i * 2 + 0, 0] = 1
            inverse_kinematics[i * 2 + 0, 1] = 0
            inverse_kinematics[i * 2 + 0, 2] = -module_locations[i].Y()
            inverse_kinematics[i * 2 + 1, 0] = 0
            inverse_kinematics[i * 2 + 1, 1] = 1
            inverse_kinematics[i * 2 + 1, 2] = +module_locations[i].X()
        return inverse_kinematics
