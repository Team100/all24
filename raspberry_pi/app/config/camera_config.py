"""Intrinstic and distortion config of a camera."""

# pylint: disable=E1101,R0903

import gtsam
import numpy as np

from app.config.identity import Identity


# TODO: more configs for testing
class CameraConfig:
    def __init__(self, identity: Identity) -> None:
        match identity:
            case Identity.UNKNOWN:
                self.camera_offset = gtsam.Pose3(
                    gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
                    np.array([0, 0, 0.5]),
                )
                self.calib = gtsam.Cal3DS2(200.0, 200.0, 0.0, 400.0, 300.0, -0.2, 0.1)
            case _:
                self.camera_offset = gtsam.Pose3(
                    gtsam.Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])),
                    np.array([0, 0, 1]),
                )
                self.calib = gtsam.Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1)
