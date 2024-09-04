# pylint: disable=C0103,C0114,C0115,C0116,E0611,E1101,R0904,R0913,W0621
# mypy: disable-error-code="import-untyped"

import math
import time
import numpy as np
import gtsam
import gtsam_unstable
from gtsam.symbol_shorthand import X
from landmark import Landmark
from plot_utils2 import Plot
from custom_factor_type import CustomFactorType

PAUSE_TIME = 1.0
ANGLE_SCALE = 0.1
LINEAR_SCALE = 0.2

NOISE1 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01]))
NOISE2 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.5, 0.5]))
NOISE3 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))

# the crash only happens in incremental mode
INCREMENTAL = True


def initialize(isam, landmarks: list[Landmark], robot_x: gtsam.Pose2) -> None:
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    timestamps = gtsam.FixedLagSmootherKeyTimestampMap()

    for l in landmarks:
        graph.add(gtsam.PriorFactorPoint2(l.symbol, l.x, NOISE2))
        values.insert(l.symbol, gtsam.Point2(*l.x))
        timestamps.insert((l.symbol, 0))

    graph.add(gtsam.PriorFactorPose2(X(0), robot_x, NOISE3))
    values.insert(X(0), robot_x)
    timestamps.insert((X(0), 0))
    isam.update(graph, values, timestamps)


def add_odometry_and_target_sights(
    isam,
    x_i: int,
    latest_robot_pose_estimate: gtsam.Pose2,
    robot_x: gtsam.Pose2,
    robot_delta: gtsam.Pose2,
    landmarks: list[Landmark],
) -> None:
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    timestamps = gtsam.FixedLagSmootherKeyTimestampMap()

    new_robot_variable(x_i, latest_robot_pose_estimate, robot_delta, values, timestamps)
    add_odometry_factor(x_i, robot_delta, graph)
    # something about the boundary and the vision interact to make this crash
    add_boundary_factor(x_i, graph)
    add_vision_factors(x_i, robot_x, landmarks, graph, timestamps)

    isam.update(graph, values, timestamps)


def add_vision_factors(
    x_i: int,
    robot_x: gtsam.Pose2,
    landmarks: list[Landmark],
    graph: gtsam.NonlinearFactorGraph,
    timestamps: gtsam.FixedLagSmootherKeyTimestampMap,
):
    # separate range and bearing to make it simpler for now, using the implementation from 2010 as
    # a guide.
    # https://github.com/borglab/gtsam/blob/76d478c0e0a2b486dd4e2aaebd8cd887df457f89/gtsam/slam/BearingFactor.h
    # TODO: replace range and bearing with actual camera factors.
    # def bearing_error_func(
    #     this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    # ) -> np.ndarray:
    #     gT1: gtsam.Pose2 = v.atPose2(this.keys()[0])
    #     gT2: gtsam.Point2 = v.atPoint2(this.keys()[1])
    #     hx:gtsam.Rot2 = gT1.bearing()

    #     if H is not None:
    #         H[0] = np.eye(3)
    #         H[1] = np.eye(3)

    for l in landmarks:
        l_angle = robot_x.bearing(l.x)
        l_range = robot_x.range(l.x)
        if abs(l_angle.theta()) < 0.5 and l_range < 4:
            graph.add(
                gtsam.BearingRangeFactor2D(X(x_i), l.symbol, l_angle, l_range, NOISE2)
            )
        # graph.add(
        #     gtsam.CustomFactor(
        #         CustomFactorType.BEARING,
        #         NOISE2,
        #         gtsam.KeyVector([X(x_i), l.symbol]),
        #         bearing_error_func,
        #     )
        # )
        timestamps.insert((l.symbol, x_i))


def add_boundary_factor(x_i: int, graph: gtsam.NonlinearFactorGraph):
    # limit motion to the "field" boundary.
    # (can't use BoundingConstraint because it needs a subclass to implement value())
    # this is a 1d constraint on a 3d state (pose2) so it has a 1d noise model and 1x3 jacobian
    # see https://gtsam.org/tutorials/intro.html
    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        boundary_x = 4.5
        # there's just one key, because we use it with a length-1 keyvector.
        gT1 = v.atPose2(this.keys()[0])

        if gT1.x() <= boundary_x:
            # constraint is not active.
            if H is not None:
                H[0] = np.array([[0, 0, 0]])
            return np.array([0])

        # constraint is active
        if H is not None:
            H[0] = np.array([[gT1.rotation().c(), -gT1.rotation().s(), 0]])
        return np.array([gT1.x() - boundary_x])

    graph.add(
        gtsam.CustomFactor(
            CustomFactorType.BOUNDARY.value,
            # this makes it fail a little while after it hits the boundary
            # gtsam.noiseModel.Constrained.All(1),
            # this makes it fail when it sees the vision target after the boundary
            NOISE1,
            gtsam.KeyVector([X(x_i)]),
            error_func,
        )
    )


def add_odometry_factor(
    x_i: int, robot_delta: gtsam.Pose2, graph: gtsam.NonlinearFactorGraph
):
    # to figure out custom factors.
    # https://github.com/borglab/gtsam/blob/develop/python/CustomFactors.md
    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        gT1 = v.atPose2(this.keys()[0])
        gT2 = v.atPose2(this.keys()[1])
        odo = gT1.between(gT2)
        error = robot_delta.localCoordinates(odo)
        if H is not None:
            H[0] = -odo.inverse().AdjointMap()
            H[1] = np.eye(3)
        return error

    graph.add(
        gtsam.CustomFactor(
            CustomFactorType.BETWEEN.value,
            NOISE3,
            gtsam.KeyVector([X(x_i - 1), X(x_i)]),
            error_func,
        )
    )


def new_robot_variable(
    x_i, latest_robot_pose_estimate, robot_delta, values, timestamps
):
    # add a new value representing the robot pose, for this timestamp
    # use the last-solved estimate, extended with odometry, as the initial guess for the new state
    # the incremental smoother is very sensitive to this initial value
    new_estimate = latest_robot_pose_estimate.compose(robot_delta)
    values.insert(X(x_i), new_estimate)
    # if you're using the batch smoother, the initial value almost doesn't matter:
    # values.insert(X(x_i), gtsam.Pose2())
    timestamps.insert((X(x_i), x_i))


def forward_and_left() -> gtsam.Pose2:
    return gtsam.Pose2.Expmap([LINEAR_SCALE, 0, ANGLE_SCALE])


def make_smoother() -> gtsam.FixedLagSmoother:
    if INCREMENTAL:
        # use incremental (faster, quite sensitive to initial values)
        optimization_params = gtsam.ISAM2GaussNewtonParams()
        optimization_params.setWildfireThreshold(0.001)  # the default is 0.001
        # the crash is caused by the use of dogleg solver,
        # see ISAM2.cpp:758; in the "in between" mode of the dogleg solver
        # it takes the dot product of two diffrent-length vectors, i think because
        # the incremental smoother creates this inconsistency somehow (all the time).
        # it is harmless except for this in-between case.
        # optimization_params = gtsam.ISAM2DoglegParams()
        print(optimization_params)
        isam_params = gtsam.ISAM2Params()
        # the crash is cause by the optimization params.
        isam_params.setOptimizationParams(optimization_params)
        # relinearizing less makes the path more consistent as new factors are added
        # relinearizing more makes it more jittery
        isam_params.relinearizeSkip = 1
        isam_params.evaluateNonlinearError = False
        isam_params.cacheLinearizedFactors = True
        print(isam_params)
        # the crash is caused by these params somehow
        return gtsam_unstable.IncrementalFixedLagSmoother(10, isam_params)
        # return gtsam_unstable.IncrementalFixedLagSmoother(10)

    # use batch (2.5x slower, very insensitive to initial values)
    lm_params = gtsam.LevenbergMarquardtParams.LegacyDefaults()
    print(lm_params)
    return gtsam.BatchFixedLagSmoother(10, lm_params)


def make_plots(isam) -> tuple[Plot, Plot]:
    fig, ax = Plot.subplots(1, 2, 12, 6)
    p0 = Plot(isam, "p0", fig, ax[0])
    p1 = Plot(isam, "p1", fig, ax[1])
    fig.tight_layout()
    return p0, p1


def main() -> None:
    print("DEBUG? ", gtsam.isDebugVersion())
    isam = make_smoother()

    #######################
    # initialize plots
    landmarks: list[Landmark] = [Landmark(0, 0.5, 0.5), Landmark(1, 0.5, 4.5)]

    p0, p1 = make_plots(isam)

    robot_x: gtsam.Pose2 = gtsam.Pose2(1, 2.5, -math.pi / 2)
    prev_robot_x = robot_x
    latest_robot_pose_estimate = robot_x
    initialize(isam, landmarks, robot_x)
    # gtsam uses compile-time types so the only way to sort out which variable
    # is which actual type is by keeping a little list.
    pose_variables: list[X] = [X(0)]
    for x_i in range(1, 500):
        # time.sleep(0.1)
        robot_delta: gtsam.Pose2 = forward_and_left()
        # robot_x = latest_robot_pose_estimate.compose(robot_delta)
        robot_x = prev_robot_x.compose(robot_delta)
        prev_robot_x = robot_x
        # only one isam.update() is allowed per time step
        t0 = time.time_ns()
        add_odometry_and_target_sights(
            isam, x_i, latest_robot_pose_estimate, robot_x, robot_delta, landmarks
        )
        result: gtsam.Values = isam.calculateEstimate()
        t1 = time.time_ns()
        # print(dir(result))
        factors: gtsam.NonlinearFactorGraph = isam.getFactors()
        pose_variables.append(X(x_i))
        latest_robot_pose_estimate = result.atPose2(X(x_i))
        pose_variables = [pv for pv in pose_variables if result.exists(pv)]
        if x_i % 5 == 0:
            print(f"i {x_i} duration (ns) {t1-t0}")
        p0.plot_variables_and_factors(result, pose_variables, landmarks, factors)
        p1.plot_variables_and_factors(result, pose_variables, landmarks, factors)


if __name__ == "__main__":
    main()
