# pylint: disable=unused-import,consider-using-from-import,invalid-name,no-name-in-module,no-member,missing-function-docstring,too-many-locals
"""
Transcription of SelfCalibrationExample.cpp

This version uses landmarks in a row, and linear motion of the camera.
"""
import math
import time
import numpy as np

from gtsam import CustomFactor, NonlinearFactor, KeyVector

from gtsam import Cal3DS2  # includes distortion
from gtsam.noiseModel import Diagonal, Isotropic

# SFM-specific factors
from gtsam import PinholeCameraCal3DS2

# Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
from gtsam import Point2
from gtsam import Point3, Pose3, Rot3

# Inference and optimization
from gtsam import NonlinearFactorGraph, DoglegOptimizer, Values
from gtsam.symbol_shorthand import K, L, X

from gtsam.noiseModel import Base as SharedNoiseModel


# this is a direct translation of examples/SFMData.h
# which is slightly different from python/gtsam/examples/SFMdata.py.
def createPoints() -> list[Point3]:
    """
    Create the set of ground-truth landmarks
    """
    return [
        Point3(10.0, 10.0, 0.0),
        Point3(-10.0, 10.0, 0.0),
        # Point3(10.0, -10.0, 0.0),
        # Point3(-10.0, -10.0, 0.0),
        Point3(0.0, 0.0, 0.0),
    ]


def createPoses(
    init: Pose3 = Pose3(Rot3.Identity(), Point3(-10, 0, -30)),
    delta: Pose3 = Pose3(Rot3.Identity(), Point3(2, 0, 0)),
    steps: int = 2,
) -> list[Pose3]:
    """
    Create the set of ground-truth poses
    Default values give a circular trajectory,
    radius 30 at pi/4 intervals, always facing the circle center
    """
    poses: list[Pose3] = []
    poses.append(init)
    for i in range(1, steps):
        poses.append(poses[i - 1].compose(delta))
    # print("POSES ", poses)
    return poses


def General_SFM_Factor2_Cal3DS2(
    measured: Point2,
    model: SharedNoiseModel,
    poseKey: int,
    point: Point3,
    calibKey: int,
) -> NonlinearFactor:
    """
    Custom factor that's similar to GeneralSFMFactor2<Cal3DS2> but with a fixed landmark.
    """

    def error_func(this: CustomFactor, v: Values, H: list[np.ndarray]) -> np.ndarray:
        pose3: Pose3 = v.atPose3(this.keys()[0])
        calib: Cal3DS2 = v.atCal3DS2(this.keys()[1])

        H1_pose = np.zeros((2, 6), order="F")
        # TODO: null?
        H2_landmark = np.zeros((2, 3), order="F")
        H3_calib = np.zeros((2, 9), order="F")

        camera = PinholeCameraCal3DS2(pose3, calib)
        result = camera.project(point, H1_pose, H2_landmark, H3_calib) - measured

        if H is not None:
            H[0] = H1_pose
            H[1] = H3_calib

        return result

    return CustomFactor(model, KeyVector([poseKey, calibKey]), error_func)


def main() -> None:
    # Create the set of ground-truth
    points: list[Point3] = createPoints()
    poses: list[Pose3] = createPoses()

    # Create the factor graph
    graph = NonlinearFactorGraph()

    # Add a prior on pose x1.
    # 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    poseNoise = Diagonal.Sigmas([0.1, 0.1, 0.1, 0.3, 0.3, 0.3])
    graph.addPriorPose3(X(0), poses[0], poseNoise)

    # Simulated measurements from each camera pose, adding them to the factor graph
    # Kcal = Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)
    Kcal = Cal3DS2(50.0, 50.0, 0.0, 50.0, 50.0, -0.2, 0.1, 0.0, 0.0)
    measurementNoise = Isotropic.Sigma(2, 1.0)
    for i, pose in enumerate(poses):
        for j, point in enumerate(points):
            camera = PinholeCameraCal3DS2(pose, Kcal)
            measurement: Point2 = camera.project(point)
            # print("measurement ",measurement)
            # The only real difference with the Visual SLAM example is that here we
            # use a different factor type, that also calculates the Jacobian with
            # respect to calibration
            graph.add(
                General_SFM_Factor2_Cal3DS2(
                    measurement,
                    measurementNoise,
                    X(i),
                    point,
                    K(0),
                )
            )

    # Add a prior on the calibration.
    calNoise = Diagonal.Sigmas([500, 500, 0.1, 100, 100, 10, 10, 0.1, 0.1])
    graph.addPriorCal3DS2(K(0), Kcal, calNoise)

    # Create the initial estimate to the solution
    # now including an estimate on the camera calibration parameters
    initialEstimate = Values()
    initialEstimate.insert(
        K(0), Cal3DS2(60.0, 60.0, 0.0, 45.0, 45.0, 0.0, 0.0, 0.0, 0.0)
    )
    for i, pose in enumerate(poses):
        initialEstimate.insert(
            X(i),
            pose.compose(
                Pose3(Rot3.Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))
            ),
        )

    # Optimize the graph and print results
    t0 = time.time_ns()
    result: Values = DoglegOptimizer(graph, initialEstimate).optimize()
    t1 = time.time_ns()
    result.print("Final results:\n")
    print("duration ms ", (t1-t0)/1e6)

if __name__ == "__main__":
    main()
