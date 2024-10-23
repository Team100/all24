"""SwerveDriveKinematics100"""

# pylint: disable=C0301,E0611,R0903
import numpy as np
import numpy.typing as npt
from wpimath.geometry import Translation2d, Twist2d

from app.pose_estimator.swerve_module_position import SwerveModulePosition100


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
        self.inverse_kinematics = self._inverse_matrix(self.module_locations)
        self.forward_kinematics = np.linalg.pinv(self.inverse_kinematics)

    def to_twist_2d(self, deltas: list[SwerveModulePosition100]) -> Twist2d:
        """FORWARD: module deltas -> twist."""
        # [d cos; d sin; ...] (2n x 1)
        delta_vector: npt.NDArray = self.deltas_2_vector(deltas)
        # [dx ;dy; dtheta]
        twist_vector: npt.NDArray = np.matmul(self.forward_kinematics, delta_vector)
        return self.vector_2_twist(twist_vector)

    def deltas_2_vector(
        self, module_deltas: list[SwerveModulePosition100]
    ) -> npt.NDArray:
        """deltas -> [d cos; d sin; ... ] (2n x 1) */"""
        module_delta_matrix = np.zeros((self.num_modules * 2, 1))
        for i in range(self.num_modules):
            module: SwerveModulePosition100 = module_deltas[i]
            if abs(module.distanceMeters) < 1e-6 or module.angle.isEmpty():
                module_delta_matrix[i * 2, 0] = 0
                module_delta_matrix[i * 2 + 1, 0] = 0
            else:
                module_delta_matrix[i * 2, 0] = (
                    module.distanceMeters * module.angle.get().getCos()
                )
                module_delta_matrix[i * 2 + 1, 0] = (
                    module.distanceMeters * module.angle.get().getSin()
                )

        return module_delta_matrix

    @staticmethod
    def vector_2_twist(v: npt.NDArray) -> Twist2d:
        """[dx; dy; dtheta] (3 x 1) -> Twist2d"""
        return Twist2d(v.get(0, 0), v.get(1, 0), v.get(2, 0))

    @staticmethod
    def _inverse_matrix(module_locations: list[Translation2d]) -> npt.NDArray:
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
