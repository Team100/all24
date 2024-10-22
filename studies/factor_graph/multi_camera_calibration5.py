# pylint: disable=C0103,C0114,C0115,C0116,E0611,R0904,R0913,W0621
# mypy: disable-error-code="import-untyped"

# the general idea here is to use a value per robot pose
# and values for each camera offset from that pose, and
# another value for the camera calibration
# so the measurement factor is quaternary (though one of
# the edges leads to the constant landmarks)

# this version adds odometry, in an effort to reduce variance.

import time
from typing import Callable

import numpy as np

from gtsam import Cal3DS2  # includes distortion
from gtsam import PinholeCameraCal3DS2
from gtsam import Point2, Point3, Pose3, Rot3, Pose2, Rot2

from gtsam import CustomFactor, NonlinearFactor, KeyVector
from gtsam import NonlinearFactorGraph, DoglegOptimizer, Values
from gtsam import Marginals
from gtsam import BetweenFactorPose2

from gtsam.noiseModel import Diagonal, Isotropic
from gtsam.noiseModel import Base as SharedNoiseModel

from gtsam.symbol_shorthand import C  # camera offset
from gtsam.symbol_shorthand import K  # camera calibration
from gtsam.symbol_shorthand import X  # robot pose

from numerical_derivative import numericalDerivative11
from numerical_derivative import numericalDerivative31
from numerical_derivative import numericalDerivative32
from numerical_derivative import numericalDerivative33

from custom_factor_type import CustomFactorType
from plot_utils2 import Plot


# this is not a good model for the gyro behavior
# TODO: add a real drifting gyro simulation
GYRO_NOISE = Diagonal.Sigmas([0.04])
# about 1cm and 1.5 degrees per odo fix
# TODO: add a factor for wheel diameter
ODO_NOISE = Diagonal.Sigmas(np.array([0.01, 0.01, 0.03]))


# camera "zero" is facing +z; this turns it to face +x
CAM_COORD = Pose3(Rot3(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])), Point3(0, 0, 0))
CAMERA_NOISE = Isotropic.Sigma(2, 1.0)
# a grid of landmarks on the y axis, x-forward, z-up,
LANDMARK_GROUND_TRUTH: list[Point3] = [
    Point3(0.0, -2.0, 1.0),
    # Point3(0.0, 0.0, 1.0),
    Point3(0.0, 2.0, 1.0),
    # z-variation helps quite a bit
    Point3(0.0, -2.0, 2.0),
    # Point3(0.0, 0.0, 2.0),
    Point3(0.0, 2.0, 2.0),
    # x-variation doesn't help very much
    # Point3(2.0, -2.0, 1.0),
    # Point3(2.0, 0.0, 1.0),
    # Point3(2.0, 2.0, 1.0),
]
# Robot poses are all on the ground, so use Pose2
# this is in x-forward z-up coordinates now,
# so the scan is in Y
# the "middle" points in y actually only help a little;
# it's the y range that seems most important.
ROBOT_GROUND_TRUTH = [
    # without rotations, sigma around 1.5m
    # Pose2(Rot2.Identity(), Point2(-10, -4)),
    # Pose2(Rot2.Identity(), Point2(-10, -2)),
    # Pose2(Rot2.Identity(), Point2(-10, -0)),
    # Pose2(Rot2.Identity(), Point2(-10, 2)),
    # Pose2(Rot2.Identity(), Point2(-10, 4)),
    # Pose2(Rot2.Identity(), Point2(-8, -4)),
    # Pose2(Rot2.Identity(), Point2(-8, -2)),
    # Pose2(Rot2.Identity(), Point2(-8, -0)),
    # Pose2(Rot2.Identity(), Point2(-8, 2)),
    # Pose2(Rot2.Identity(), Point2(-8, 4)),
    # with rotations, sigma around 30cm (!)
    Pose2(Rot2.fromDegrees(20), Point2(-9, -4)),
    Pose2(Rot2.fromDegrees(10), Point2(-9, -2)),
    Pose2(Rot2.fromDegrees(0), Point2(-9, -0)),
    Pose2(Rot2.fromDegrees(-10), Point2(-9, 2)),
    Pose2(Rot2.fromDegrees(-20), Point2(-9, 4)),
    Pose2(Rot2.fromDegrees(20), Point2(-7, -4)),
    Pose2(Rot2.fromDegrees(10), Point2(-7, -2)),
    Pose2(Rot2.fromDegrees(0), Point2(-7, -0)),
    Pose2(Rot2.fromDegrees(-10), Point2(-7, 2)),
    Pose2(Rot2.fromDegrees(-20), Point2(-7, 4)),
    # more points with the same rotations: 9-13 cm
    # Pose2(Rot2.fromDegrees(20), Point2(-8, -4)),
    # Pose2(Rot2.fromDegrees(10), Point2(-8, -2)),
    # Pose2(Rot2.fromDegrees(0), Point2(-8, -0)),
    # Pose2(Rot2.fromDegrees(-10), Point2(-8, 2)),
    # Pose2(Rot2.fromDegrees(-20), Point2(-8, 4)),
    # Pose2(Rot2.fromDegrees(20), Point2(-6, -4)),
    # Pose2(Rot2.fromDegrees(10), Point2(-6, -2)),
    # Pose2(Rot2.fromDegrees(0), Point2(-6, -0)),
    # Pose2(Rot2.fromDegrees(-10), Point2(-6, 2)),
    # Pose2(Rot2.fromDegrees(-20), Point2(-6, 4)),
    # try more rotations around the seame points, 15cm.
    # Pose2(Rot2.fromDegrees(-20), Point2(-9, -4)),
    # Pose2(Rot2.fromDegrees(-10), Point2(-9, -2)),
    # Pose2(Rot2.fromDegrees(0), Point2(-9, -0)),
    # Pose2(Rot2.fromDegrees(10), Point2(-9, 2)),
    # Pose2(Rot2.fromDegrees(20), Point2(-9, 4)),
    # Pose2(Rot2.fromDegrees(-20), Point2(-7, -4)),
    # Pose2(Rot2.fromDegrees(-10), Point2(-7, -2)),
    # Pose2(Rot2.fromDegrees(0), Point2(-7, -0)),
    # Pose2(Rot2.fromDegrees(10), Point2(-7, 2)),
    # Pose2(Rot2.fromDegrees(20), Point2(-7, 4)),
    # combining all of the above yields 6-11cm.
    # these short range points help a little, 5cm
    Pose2(Rot2.fromDegrees(30), Point2(-5, -4)),
    Pose2(Rot2.fromDegrees(15), Point2(-5, -2)),
    Pose2(Rot2.fromDegrees(0), Point2(-5, -0)),
    Pose2(Rot2.fromDegrees(-15), Point2(-5, 2)),
    Pose2(Rot2.fromDegrees(-30), Point2(-5, 4)),
    Pose2(Rot2.fromDegrees(30), Point2(-4, -4)),
    Pose2(Rot2.fromDegrees(15), Point2(-4, -2)),
    Pose2(Rot2.fromDegrees(0), Point2(-4, -0)),
    Pose2(Rot2.fromDegrees(-15), Point2(-4, 2)),
    Pose2(Rot2.fromDegrees(-30), Point2(-4, 4)),
    # maybe variation in y isn't useful at all?
    # this yields 10cm for 7 angles per spot, not terrible but not as good
    # spreading in y does not help, more angles doesn't help
    # Pose2(Rot2.fromDegrees(-30), Point2(-8, -4)),
    # Pose2(Rot2.fromDegrees(-10), Point2(-8, -4)),
    # Pose2(Rot2.fromDegrees(0), Point2(-8, -4)),
    # Pose2(Rot2.fromDegrees(10), Point2(-8, -4)),
    # Pose2(Rot2.fromDegrees(30), Point2(-8, -4)),
    # Pose2(Rot2.fromDegrees(-30), Point2(-5, -4)),
    # Pose2(Rot2.fromDegrees(-10), Point2(-5, -4)),
    # Pose2(Rot2.fromDegrees(0), Point2(-5, -4)),
    # Pose2(Rot2.fromDegrees(10), Point2(-5, -4)),
    # Pose2(Rot2.fromDegrees(30), Point2(-5, -4)),
    # Pose2(Rot2.fromDegrees(-30), Point2(-8, 4)),
    # Pose2(Rot2.fromDegrees(-10), Point2(-8, 4)),
    # Pose2(Rot2.fromDegrees(0), Point2(-8, 4)),
    # Pose2(Rot2.fromDegrees(10), Point2(-8, 4)),
    # Pose2(Rot2.fromDegrees(30), Point2(-8, 4)),
    # Pose2(Rot2.fromDegrees(-30), Point2(-5, 4)),
    # Pose2(Rot2.fromDegrees(-10), Point2(-5, 4)),
    # Pose2(Rot2.fromDegrees(0), Point2(-5, 4)),
    # Pose2(Rot2.fromDegrees(10), Point2(-5, 4)),
    # Pose2(Rot2.fromDegrees(30), Point2(-5, 4)),
]
# these offsets are x-forward z-up
# note: if offsets are parallel to the robot path then
# the system is underdetermined, so give these some yaw.
OFFSET_GROUND_TRUTH = [
    Pose3(Rot3.Yaw(0.5).compose(Rot3.Pitch(-0.25)), Point3(0, 0.5, 0)),  # left
    Pose3(Rot3.Yaw(0).compose(Rot3.Pitch(-0.25)), Point3(0, 0, 0)),  # center
    Pose3(Rot3.Yaw(-0.5).compose(Rot3.Pitch(-0.25)), Point3(0, -0.5, 0)),  # right
]


def gyro_h_fn() -> Callable[[Pose2], float]:
    def h(robot_pose: Pose2) -> float:
        return robot_pose.rotation().theta()

    return h


def vision_h_fn(landmark: Point3) -> Callable[[Pose2, Pose3, Cal3DS2], Point2]:
    def h(robot_pose: Pose2, camera_offset: Pose3, calib: Cal3DS2) -> Point2:
        """robot_pose and camera_offset are x-forward, z-up"""
        # ctor a pose3 with x,y,yaw, x-forward, z-up
        offset_pose = Pose3(robot_pose).compose(camera_offset)
        # print("offset pose ", offset_pose)
        camera_pose = offset_pose.compose(CAM_COORD)
        # camera constructor expects z-forward y-down
        # print("camera pose ", camera_pose)
        # print("landmark ", landmark)
        camera = PinholeCameraCal3DS2(camera_pose, calib)
        return camera.project(landmark)

    return h


def GyroFactor(
    measured: float, model: SharedNoiseModel, poseKey: int
) -> NonlinearFactor:
    """Unary for now: TODO: model a real gyro"""

    def error_func(this: CustomFactor, v: Values, H: list[np.ndarray]) -> np.ndarray:
        pose2: Pose2 = v.atPose2(this.keys()[0])
        h = gyro_h_fn()
        result = h(pose2) - measured
        if H is not None:
            H[0] = numericalDerivative11(h, pose2)
        return np.array([result])

    return CustomFactor(
        CustomFactorType.UNARY.value, model, KeyVector([poseKey]), error_func
    )


# measurements (camera signal) from ground-truth
def vision_measurements(gt_pose: Pose2, offset: Pose3, landmark: Point3) -> Point2:
    # ground truth calibration
    # increasing from 100x100 400x400 reduces the pose stdev by about 2x.
    Kcal = Cal3DS2(200.0, 200.0, 0.0, 200.0, 200.0, -0.2, 0.1, 0.0, 0.0)
    return vision_h_fn(landmark)(gt_pose, offset, Kcal)


def VisionFactor(
    measured: Point2,
    model: SharedNoiseModel,
    poseKey: int,
    offsetKey: int,
    landmark: Point3,
    calibKey: int,
) -> NonlinearFactor:
    def error_func(this: CustomFactor, v: Values, H: list[np.ndarray]) -> np.ndarray:
        pose2: Pose2 = v.atPose2(this.keys()[0])
        offset: Pose3 = v.atPose3(this.keys()[1])
        calib: Cal3DS2 = v.atCal3DS2(this.keys()[2])
        h = vision_h_fn(landmark)
        result = h(pose2, offset, calib) - measured
        if H is not None:
            H[0] = numericalDerivative31(h, pose2, offset, calib)
            H[1] = numericalDerivative32(h, pose2, offset, calib)
            H[2] = numericalDerivative33(h, pose2, offset, calib)
        return result

    return CustomFactor(
        CustomFactorType.OTHER.value,
        model,
        KeyVector([poseKey, offsetKey, calibKey]),
        error_func,
    )


def main() -> None:
    graph = NonlinearFactorGraph()

    # initial robot prior
    ground_truth_x0 = ROBOT_GROUND_TRUTH[0]
    # the error in the prior makes a big difference.

    # the prior noise is carried through the whole thing somehow. hhm..
    graph.addPriorPose2(
        X(0),
        ground_truth_x0.compose(Pose2(Rot2.fromDegrees(5), Point2(0.01, 0.01))),
        Diagonal.Sigmas([3, 3, 3]),
    )

    # camera calibration prior
    graph.addPriorCal3DS2(
        K(0),
        # note the prior is not close
        Cal3DS2(100.0, 100.0, 0.0, 100.0, 100.0, 0, 0, 0.0, 0.0),
        # note we're saying that there are really no p1 or p2 terms.
        Diagonal.Sigmas([500, 500, 0.1, 100, 100, 10, 10, 0.001, 0.001]),
    )

    # camera offset priors
    for i, ground_truth_c in enumerate(OFFSET_GROUND_TRUTH):
        graph.addPriorPose3(
            C(i),
            ground_truth_c.compose(Pose3(Rot3.Yaw(0.01), Point3(0.01, 0.01, 0))),
            Diagonal.Sigmas([1, 1, 1, 3, 3, 3]),
        )

    # sightings from each robot pose
    gt_robot_pose: Pose2
    for i, gt_robot_pose in enumerate(ROBOT_GROUND_TRUTH):
        # add gyro
        gyro_measurement = gyro_h_fn()(gt_robot_pose)
        # print("gyro ", gyro_measurement)
        graph.add(GyroFactor(gyro_measurement, GYRO_NOISE, X(i)))
        # add odometry
        if i > 0:
            twist = ROBOT_GROUND_TRUTH[i - 1].between(gt_robot_pose)
            graph.add(BetweenFactorPose2(X(i - 1), X(i), twist, ODO_NOISE))
        # add vision measurements
        for gt_landmark in LANDMARK_GROUND_TRUTH:
            for j, gt_offset in enumerate(OFFSET_GROUND_TRUTH):
                camera_measurement = vision_measurements(
                    gt_robot_pose, gt_offset, gt_landmark
                )
                if np.any(camera_measurement < 0):
                    continue
                if np.any(camera_measurement > 400):
                    continue
                print("pixels ", camera_measurement)
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
        initialEstimate.insert(
            X(i), gt_pose.compose(Pose2(Rot2.fromDegrees(5), Point2(0.01, 0.01)))
        )

    # initial camera offsets
    for i, pose in enumerate(OFFSET_GROUND_TRUTH):
        initialEstimate.insert(C(i), pose.compose(initial_error))

    t0 = time.time_ns()
    optimizer = DoglegOptimizer(graph, initialEstimate)
    result: Values = optimizer.optimize()
    t1 = time.time_ns()
    result.print("Final results:\n")
    print("duration ms ", (t1 - t0) / 1000000)

    # what's the uncertainty in the result?
    np.set_printoptions(precision=2, linewidth=150)
    marginals = Marginals(graph, result)
    print(
        "\nK0 K: ",
        result.atCal3DS2(K(0)).K(), # ndarray
        "\nK0 k: ",
        result.atCal3DS2(K(0)).k(), # ndarray
        "\nK0 mean: ",
        result.atCal3DS2(K(0)),
        "\nsigma: ",
        np.sqrt(np.diagonal(marginals.marginalCovariance(K(0)))),
        "\ncovariance: ",
        marginals.marginalCovariance(K(0)),
    )
    for i in range(len(ROBOT_GROUND_TRUTH)):
        print(f"X{i} mean: ", result.atPose2(X(i)))
        print(f"X{i} sigma: ", np.sqrt(np.diagonal(marginals.marginalCovariance(X(i)))))
    for i in range(len(OFFSET_GROUND_TRUTH)):
        print(f"C{i} matrix: ", result.atPose3(C(i)).matrix()) # ndarray
        print(f"C{i} mean: ", result.atPose3(C(i)))
        print(f"C{i} sigma: ", np.sqrt(np.diagonal(marginals.marginalCovariance(C(i)))))
        print(f"C{i} covariance: ", marginals.marginalCovariance(C(i)))

    # print("DOT\n", graph.dot(result))

    # fig, ax = Plot.subplots(1, 2, 12, 6)
    fig, ax = Plot.subplots(1, 1, 6, 6)
    # p0 = Plot(optimizer, "p0", fig, ax[0])
    p0 = Plot(optimizer, "p0", fig, ax)
    p0.plot_variables_and_factors(
        result, [X(i) for i in range(len(ROBOT_GROUND_TRUTH))], [], graph
    )
    p0.plot_some_points(LANDMARK_GROUND_TRUTH)
    p0.wait()


if __name__ == "__main__":
    main()
