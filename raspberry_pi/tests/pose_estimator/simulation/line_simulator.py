"""Very simple simulated telemetry, accelerating to infinity in +x"""
# pylint: disable=C0301,E0611,R0902,R0903

from wpimath.geometry import Pose2d


class LineSimulator:
    def __init__(self) -> None:
        self.wpi_pose = Pose2d(0, 0, 0)
        self.time_s: float = 0
        self.gt_x: float = 0

        # constants
        self.gt_y: float = 0
        self.gt_theta: float = 0
        self.gt_vy: float = 0
        self.gt_vtheta: float = 0
        self.gt_ax: float = 1
        self.gt_ay: float = 0
        self.gt_atheta: float = 0

    def step(self, dt_s: float) -> None:
        self.time_s += dt_s
        self.gt_x = 0.5 * self.gt_ax * self.time_s * self.time_s
