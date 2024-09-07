# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0621
# mypy: disable-error-code="import-untyped"

# the general idea here is to use a value per robot pose
# and values for each camera offset from that pose, and
# another value for the camera calibration
# so the measurement factor is quaternary (though one of
# the edges leads to the constant landmarks)

# this version has two cameras.

from typing import Callable

import numpy as np

from gtsam import Cal3DS2  # includes distortion
from gtsam import PinholeCameraCal3DS2
from gtsam import Point2
from gtsam import Point3, Pose3, Rot3

from gtsam import CustomFactor, NonlinearFactor, KeyVector
from gtsam import NonlinearFactorGraph, DoglegOptimizer, Values

from gtsam.noiseModel import Diagonal, Isotropic
from gtsam.noiseModel import Base as SharedNoiseModel

from gtsam.symbol_shorthand import C  # camera offset
from gtsam.symbol_shorthand import K  # camera calibration
from gtsam.symbol_shorthand import X  # robot pose

from numerical_derivative import numericalDerivative31
from numerical_derivative import numericalDerivative32
from numerical_derivative import numericalDerivative33

CAMERA_NOISE = Isotropic.Sigma(2, 1.0)
LANDMARK_GROUND_TRUTH: list[Point3] = [
    Point3(-10.0, 10.0, 0.0),
    Point3(-5.0, 0.0, 0.0),
    Point3(0.0, 10.0, 10.0),
    Point3(5.0, 0.0, 0.0),
    Point3(10.0, 10.0, 0.0),
]
ROBOT_GROUND_TRUTH = [
    Pose3(Rot3.Identity(), Point3(-10, 0, -30)),
    Pose3(Rot3.Identity(), Point3(-8, 0, -30)),
    # more poses doesn't seem to make it more accurate ... ?
    Pose3(Rot3.Identity(), Point3(-6, 0, -30)),
    Pose3(Rot3.Identity(), Point3(-4, 0, -30)),
    Pose3(Rot3.Identity(), Point3(-2, 0, -30)),
    Pose3(Rot3.Identity(), Point3(0, 0, -30)),
]
OFFSET_GROUND_TRUTH = [
    Pose3(Rot3.Yaw(0.1), Point3(-0.5, 0, 0)),
    Pose3(Rot3.Yaw(-0.1), Point3(0.5, 0, 0)),
]


def h_fn(landmark: Point3) -> Callable[[Pose3, Pose3, Cal3DS2], Point2]:
    def h(pose3: Pose3, offset: Pose3, calib: Cal3DS2) -> Point2:
        camera = PinholeCameraCal3DS2(pose3.compose(offset), calib)
        return camera.project(landmark)

    return h


# measurements (camera signal) from ground-truth
def make_measurements(pose: Pose3, offset: Pose3, landmark: Point3) -> Point2:
    # ground truth calibration
    Kcal = Cal3DS2(50.0, 50.0, 0.0, 50.0, 50.0, -0.2, 0.1, 0.0, 0.0)
    return h_fn(landmark)(pose, offset, Kcal)


def VisionFactor(
    measured: Point2,
    model: SharedNoiseModel,
    poseKey: int,
    offsetKey: int,
    landmark: Point3,
    calibKey: int,
) -> NonlinearFactor:
    def error_func(this: CustomFactor, v: Values, H: list[np.ndarray]) -> np.ndarray:
        pose3: Pose3 = v.atPose3(this.keys()[0])
        offset: Pose3 = v.atPose3(this.keys()[1])
        calib: Cal3DS2 = v.atCal3DS2(this.keys()[2])
        h = h_fn(landmark)
        result = h(pose3, offset, calib) - measured
        if H is not None:
            H[0] = numericalDerivative31(h, pose3, offset, calib)
            H[1] = numericalDerivative32(h, pose3, offset, calib)
            H[2] = numericalDerivative33(h, pose3, offset, calib)
        return result

    return CustomFactor(model, KeyVector([poseKey, offsetKey, calibKey]), error_func)


def main() -> None:
    graph = NonlinearFactorGraph()

    # initial robot prior
    graph.addPriorPose3(
        X(0),
        Pose3(Rot3.Identity(), Point3(-10, 0, -30)),
        Diagonal.Sigmas([0.1, 0.1, 0.1, 0.3, 0.3, 0.3]),
    )

    # camera calibration prior
    graph.addPriorCal3DS2(
        K(0),
        Cal3DS2(50.0, 50.0, 0.0, 50.0, 50.0, -0.2, 0.1, 0.0, 0.0),
        Diagonal.Sigmas([500, 500, 0.1, 100, 100, 10, 10, 0.1, 0.1]),
    )

    # camera offset priors
    graph.addPriorPose3(
        C(0),
        Pose3(Rot3.Identity(), Point3(-0.5, 0, 0)),
        Diagonal.Sigmas([0.1, 0.1, 0.1, 0.3, 0.3, 0.3]),
    )
    graph.addPriorPose3(
        C(1),
        Pose3(Rot3.Identity(), Point3(0.5, 0, 0)),
        Diagonal.Sigmas([0.1, 0.1, 0.1, 0.3, 0.3, 0.3]),
    )

    # sightings from each robot pose
    for i, gt_robot_pose in enumerate(ROBOT_GROUND_TRUTH):
        for gt_landmark in LANDMARK_GROUND_TRUTH:
            for j, gt_offset in enumerate(OFFSET_GROUND_TRUTH):
                camera_measurement = make_measurements(
                    gt_robot_pose, gt_offset, gt_landmark
                )
                graph.add(
                    VisionFactor(
                        camera_measurement,
                        CAMERA_NOISE,
                        X(i),
                        C(j),
                        gt_landmark,
                        K(0),
                    )
                )

    initialEstimate = Values()
    # initial calibration
    initialEstimate.insert(
        K(0), Cal3DS2(60.0, 60.0, 0.0, 45.0, 45.0, 0.0, 0.0, 0.0, 0.0)
    )

    # initial robot poses
    for i, pose in enumerate(ROBOT_GROUND_TRUTH):
        initialEstimate.insert(
            X(i),
            pose.compose(
                Pose3(Rot3.Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))
            ),
        )

    # initial camera offsets
    for i, pose in enumerate(OFFSET_GROUND_TRUTH):
        initialEstimate.insert(
            C(i),
            pose.compose(
                Pose3(Rot3.Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))
            ),
        )

    result: Values = DoglegOptimizer(graph, initialEstimate).optimize()
    result.print("Final results:\n")
    # print("DOT\n", graph.dot(result))


if __name__ == "__main__":
    main()
