# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0621
# mypy: disable-error-code="import-untyped"

# the general idea here is to use a value per robot pose
# and values for each camera offset from that pose, and
# another value for the camera calibration
# so the measurement factor is quaternary (though one of
# the edges leads to the constant landmarks)

# this version adds a simple gyro factor, which mostly removes the
# rotational DOF in the solver, producing 100X better rotation
# accuracy

import time
from typing import Callable

import numpy as np

from gtsam import Cal3DS2  # includes distortion
from gtsam import PinholeCameraCal3DS2
from gtsam import Point2, Point3, Pose3, Rot3, Pose2, Rot2

from gtsam import CustomFactor, NonlinearFactor, KeyVector
from gtsam import NonlinearFactorGraph, DoglegOptimizer, Values

from gtsam.noiseModel import Diagonal, Isotropic
from gtsam.noiseModel import Base as SharedNoiseModel

from gtsam.symbol_shorthand import C  # camera offset
from gtsam.symbol_shorthand import K  # camera calibration
from gtsam.symbol_shorthand import X  # robot pose

from numerical_derivative import numericalDerivative11
from numerical_derivative import numericalDerivative31
from numerical_derivative import numericalDerivative32
from numerical_derivative import numericalDerivative33

# this is not a good model for the gyro behavior
# TODO: add a real drifting gyro simulation
GYRO_NOISE = Diagonal.Sigmas([0.04])

# camera "zero" is facing +z; this turns it to face +x
CAM_COORD = Pose3(Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])), Point3(0, 0, 0))
CAMERA_NOISE = Isotropic.Sigma(2, 1.0)
# a grid of landmarks on the y axis, x-forward, z-up,
LANDMARK_GROUND_TRUTH: list[Point3] = [
    Point3(0.0, -1.0, 0.0),
    Point3(0.0, 0.0, 0.0),
    Point3(0.0, 1.0, 0.0),
    Point3(0.0, -1.0, 1.0),
    Point3(0.0, 0.0, 1.0),
    Point3(0.0, 1.0, 1.0),
]
# Robot poses are all on the ground, so use Pose2
# this is in x-forward z-up coordinates now,
# so the scan is in Y
ROBOT_GROUND_TRUTH = [
    Pose2(Rot2.fromDegrees(0), Point2(-10, -2)),
    Pose2(Rot2.fromDegrees(0), Point2(-10, -1)),
    Pose2(Rot2.fromDegrees(0), Point2(-10, -0)),
    Pose2(Rot2.fromDegrees(0), Point2(-10, 1)),
    Pose2(Rot2.fromDegrees(0), Point2(-10, 2)),
]
# these offsets are x-forward z-up
# note: if offsets are parallel to the robot path then
# the system is underdetermined, so give these some yaw.
OFFSET_GROUND_TRUTH = [
    Pose3(Rot3.Yaw(0.5), Point3(0, 0.5, 0)),  # left
    Pose3(Rot3.Yaw(0), Point3(0, 0, 0)),  # center
    Pose3(Rot3.Yaw(-0.5), Point3(0, -0.5, 0)),  # right
]


def gyro_h_fn() -> Callable[[Pose3], float]:
    def h(robot_pose: Pose3) -> float:
        return robot_pose.rotation().yaw()

    return h


def vision_h_fn(landmark: Point3) -> Callable[[Pose3, Pose3, Cal3DS2], Point2]:
    def h(robot_pose: Pose3, camera_offset: Pose3, calib: Cal3DS2) -> Point2:
        """robot_pose and camera_offset are x-forward, z-up"""
        offset_pose = robot_pose.compose(camera_offset)
        # print("offset pose ", offset_pose)
        camera_pose = offset_pose.compose(CAM_COORD)
        # camera constructor expects z-forward y-down
        # print("camera pose ", camera_pose)
        # print("landmark ", landmark)
        camera = PinholeCameraCal3DS2(camera_pose, calib)
        return camera.project(landmark)

    return h


# gyro is unary for now
def GyroFactor(
    measured: float, model: SharedNoiseModel, poseKey: int
) -> NonlinearFactor:
    def error_func(this: CustomFactor, v: Values, H: list[np.ndarray]) -> np.ndarray:
        pose3: Pose3 = v.atPose3(this.keys()[0])
        h = gyro_h_fn()
        result = h(pose3) - measured
        if H is not None:
            H[0] = numericalDerivative11(h, pose3)
        return np.array([result])

    return CustomFactor(model, KeyVector([poseKey]), error_func)


# measurements (camera signal) from ground-truth
def vision_measurements(gt_pose: Pose2, offset: Pose3, landmark: Point3) -> Point2:
    # ground truth calibration
    Kcal = Cal3DS2(50.0, 50.0, 0.0, 50.0, 50.0, -0.2, 0.1, 0.0, 0.0)
    # makes a pose3 with x,y,yaw, x-forward, z-up
    # print("robot pose2 ", gt_pose)
    robot_pose = Pose3(gt_pose)
    # print("robot pose3 ", robot_pose)
    # print("landmark ", landmark)
    return vision_h_fn(landmark)(robot_pose, offset, Kcal)


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
        h = vision_h_fn(landmark)
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
    ground_truth_x0 = ROBOT_GROUND_TRUTH[0]
    # the error in the prior makes a big difference.
    prior_error = Pose3(Rot3.Yaw(0.01), Point3(0.01, 0.01, 0))
    # the prior noise seems not to matter much
    prior_noise = Diagonal.Sigmas([1, 1, 1, 3, 3, 3])
    graph.addPriorPose3(
        X(0),
        Pose3(ground_truth_x0).compose(prior_error),
        prior_noise,
    )

    # camera calibration prior
    graph.addPriorCal3DS2(
        K(0),
        # note the prior is not close
        Cal3DS2(60.0, 60.0, 0.0, 60.0, 60.0, 0, 0, 0.0, 0.0),
        # note we're saying that there are really no p1 or p2 terms.
        Diagonal.Sigmas([500, 500, 0.1, 100, 100, 10, 10, 0.001, 0.001]),
    )

    # camera offset priors
    for i, ground_truth_c in enumerate(OFFSET_GROUND_TRUTH):
        graph.addPriorPose3(
            C(i),
            ground_truth_c.compose(prior_error),
            prior_noise,
        )

    # sightings from each robot pose
    gt_robot_pose: Pose2
    for i, gt_robot_pose in enumerate(ROBOT_GROUND_TRUTH):
        gyro_measurement = gyro_h_fn()(Pose3(gt_robot_pose))
        graph.add(
            GyroFactor(
                gyro_measurement,
                GYRO_NOISE,
                X(i)
            )
        )
        for gt_landmark in LANDMARK_GROUND_TRUTH:
            for j, gt_offset in enumerate(OFFSET_GROUND_TRUTH):
                camera_measurement = vision_measurements(
                    gt_robot_pose, gt_offset, gt_landmark
                )
                # print("pixels ", camera_measurement)
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
    # initial calibration, not close
    initialEstimate.insert(
        K(0), Cal3DS2(60.0, 60.0, 0.0, 45.0, 45.0, 0.0, 0.0, 0.0, 0.0)
    )

    # initial pose error: values are not close to the correct values
    initial_error = Pose3(Rot3.Rodrigues(-0.1, 0.2, 0.25), Point3(0.5, -0.5, 0.50))

    # initial robot poses
    for i, gt_pose in enumerate(ROBOT_GROUND_TRUTH):
        pose = Pose3(gt_pose)
        initialEstimate.insert(X(i), pose.compose(initial_error))

    # initial camera offsets
    for i, pose in enumerate(OFFSET_GROUND_TRUTH):
        initialEstimate.insert(C(i), pose.compose(initial_error))

    t0 = time.time_ns()
    result: Values = DoglegOptimizer(graph, initialEstimate).optimize()
    t1 = time.time_ns()
    result.print("Final results:\n")
    print("duration ms ", (t1 - t0) / 1000000)

    # print("DOT\n", graph.dot(result))


if __name__ == "__main__":
    main()
