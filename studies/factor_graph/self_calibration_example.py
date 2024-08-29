# pylint: disable=unused-import,consider-using-from-import,invalid-name,no-name-in-module,no-member,missing-function-docstring,too-many-locals
"""
Transcription of SelfCalibrationExample.cpp
"""

from gtsam import Cal3_S2
from gtsam.noiseModel import Diagonal, Isotropic

# SFM-specific factors
from gtsam import GeneralSFMFactor2Cal3_S2  # does calibration !
from gtsam import PinholeCameraCal3_S2

# Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
from gtsam import Point2
from gtsam import Point3, Pose3, Rot3

# Inference and optimization
from gtsam import NonlinearFactorGraph, DoglegOptimizer, DoglegParams, Values
from gtsam import LevenbergMarquardtOptimizer, LevenbergMarquardtParams
from gtsam.symbol_shorthand import K, L, X

# For loading the data
from sfm_data import createPoints, createPoses


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
    Kcal = Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)
    measurementNoise = Isotropic.Sigma(2, 1.0)
    for i, pose in enumerate(poses):
        for j, point in enumerate(points):
            camera = PinholeCameraCal3_S2(pose, Kcal)
            measurement: Point2 = camera.project(point)
            # The only real difference with the Visual SLAM example is that here we
            # use a different factor type, that also calculates the Jacobian with
            # respect to calibration
            graph.add(
                GeneralSFMFactor2Cal3_S2(
                    measurement,
                    measurementNoise,
                    X(i),
                    L(j),
                    K(0),
                )
            )

    # Add a prior on the position of the first landmark.
    pointNoise = Isotropic.Sigma(3, 0.1)
    graph.addPriorPoint3(L(0), points[0], pointNoise)  # add directly to graph

    # Add a prior on the calibration.
    calNoise = Diagonal.Sigmas([500, 500, 0.1, 100, 100])
    graph.addPriorCal3_S2(K(0), Kcal, calNoise)

    # Create the initial estimate to the solution
    # now including an estimate on the camera calibration parameters
    initialEstimate = Values()
    # initialEstimate.insert(K(0), Cal3_S2(60.0, 60.0, 0.0, 45.0, 45.0))
    # use a really bad estimate
    initialEstimate.insert(K(0), Cal3_S2(10.0, 10.0, 0.0, 10.0, 10.0))
    for i, pose in enumerate(poses):
        initialEstimate.insert(
            X(i),
            pose.compose(
                Pose3(Rot3.Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))
            ),
        )
    for j, point in enumerate(points):
        initialEstimate.insert(L(j), point + Point3(-0.25, 0.20, 0.15))

    # Optimize the graph and print results

    params = DoglegParams()
    result: Values = DoglegOptimizer(graph, initialEstimate, params).optimize()

    # params = LevenbergMarquardtParams()
    # result: Values = LevenbergMarquardtOptimizer(
    #     graph, initialEstimate, params
    # ).optimize()

    result.print("Final results:\n")


if __name__ == "__main__":
    main()
