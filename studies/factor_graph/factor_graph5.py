# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

import math
import time
import numpy as np
import gtsam
import gtsam_unstable  # type:ignore
from gtsam.symbol_shorthand import X
from landmark import Landmark
from plot_utils import Plot

PAUSE_TIME = 1.0
ANGLE_SCALE = 0.1
LINEAR_SCALE = 0.2

# noise is really high to make the animation more interesting
NOISE2 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))
NOISE3 = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))

BOUNDARY = True


# the BoundingConstraint is not available in python, I think because it
# relies on subclasses implementing the "value()" method.
# so use the Custom Factor idea.
# https://github.com/borglab/gtsam/blob/develop/python/CustomFactors.md
#
# this is cut-and-paste from the example in that doc.
#
def custom_between_factor(expectation: gtsam.Pose2):
    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        """
        Error function that mimics a BetweenFactor
        :param this: reference to the current CustomFactor being evaluated
        :param v: Values object
        :param H: list of references to the Jacobian arrays
        :return: the non-linear error
        """
        key0 = this.keys()[0]
        key1 = this.keys()[1]
        gT1, gT2 = v.atPose2(key0), v.atPose2(key1)
        error = expectation.localCoordinates(gT1.between(gT2))

        if H is not None:
            result = gT1.between(gT2)
            H[0] = -result.inverse().AdjointMap()
            H[1] = np.eye(3)
        return error

    return error_func


# limit motion to the "field" boundary.
# see simulated2DConstraints.h:ScalarCoordConstraint1
# note this constrains *tangent space* which isn't right
def custom_boundary_constraint():
    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        # there's just one key, because we use it with a length-1 keyvector.
        key0 = this.keys()[0]
        gT1 = v.atPose2(key0)

        if gT1.x() <= 4.5:
            # constraint is not active.
            if H is not None:
                H[0] = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
            return np.array([0, 0.0, 0.0])

        # constraint is active

        ################
        # this is from Pose2::range()
        # jacobian has only x component

        d = np.array([gT1.x() - 4.5, 0, 0])
        D_d_pose = np.array(
            [
                [gT1.rotation().c(), -gT1.rotation().s(), 0],
                [0, 0, 0],
                [0, 0, 0],
            ]
        )
        if H is not None:
            H[0] = D_d_pose
            # print(H[0])
        error = d.dot(D_d_pose)
        print(error)
        return error

        ###############
        # this works better than the previous attempt
        # but it is still wrong; this pose should not have
        # any authority in y.

        # pose_at_boundary = gtsam.Pose2(4.5, gT1.y(), gT1.theta())
        # # zero error at the boundary
        # expectation = gtsam.Pose2()
        # twist = gT1.between(pose_at_boundary)
        # # print(twist)
        # error = expectation.localCoordinates(twist)
        # # print(error)

        # if H is not None:
        #     H[0] = -twist.inverse().AdjointMap()
        #     print("H ", H[0])
        # print("ERROR ", error)
        # return error

        ###########33
        # this is fron awhile ago
        #
        # error = gT1.x() - 4.5
        # if H is not None:
        #     # this is wrong, it needs to be rotated
        #     H[0] = np.array([[1000, 0, 0], [0, 0, 0], [0, 0, 0]])
        # return np.array([1000*error, 0.0, 0.0])

    return error_func


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
    new_factors: list[int] = isam.getISAM2Result().getNewFactorsIndices()
    print("INIT ", new_factors)


def add_odometry_and_target_sights(
    isam,
    x_i: int,
    robot_x: gtsam.Pose2,
    robot_delta: gtsam.Pose2,
    landmarks: list[Landmark],
) -> None:
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()
    timestamps = gtsam.FixedLagSmootherKeyTimestampMap()
    graph.add(
        gtsam.CustomFactor(
            NOISE3,
            gtsam.KeyVector([X(x_i - 1), X(x_i)]),
            custom_between_factor(robot_delta),
        )
    )

    values.insert(X(x_i), robot_x)
    timestamps.insert((X(x_i), x_i))
    for l in landmarks:
        l_angle = robot_x.bearing(l.x)
        l_range = robot_x.range(l.x)
        if abs(l_angle.theta()) < 0.5 and l_range < 4:
            graph.add(
                gtsam.BearingRangeFactor2D(X(x_i), l.symbol, l_angle, l_range, NOISE2)
            )
        timestamps.insert((l.symbol, x_i))

    # also add boundary factor
    #
    # Constrained.MixedSignal says there is some (low) uncertainty about the
    # constrained cartesian dimensions, but doesn't say anything about rotation.
    #
    # Constrained.All means that the error produced by our custom factor is not scaled
    # by sigma at all.  see NoiseModel.cpp.  this seems wrong.
    #
    # the Unit noise model seems very soft, not useful.
    #
    # the NOISE3 model seems inappropriate for a boundary.
    if BOUNDARY:
        graph.add(
            gtsam.CustomFactor(
                # the boundary affects translation but not rotation
                gtsam.noiseModel.Constrained.MixedSigmas(1, np.array([0.2, 0.2, 0.0])),
                # gtsam.noiseModel.Constrained.All(3),
                # gtsam.noiseModel.Unit.Create(3),
                # NOISE3,
                gtsam.KeyVector([X(x_i)]),
                custom_boundary_constraint(),
            )
        )
    isam.update(graph, values, timestamps)
    # this is the list of factors i just added, in order.
    # one for
    new_factors: list[int] = isam.getISAM2Result().getNewFactorsIndices()
    # print("ODO/SIGHT ", new_factors)


def forward_and_left() -> gtsam.Pose2:
    return gtsam.Pose2.Expmap([LINEAR_SCALE, 0, ANGLE_SCALE])


def main() -> None:
    print("DEBUG? ", gtsam.isDebugVersion())

    landmarks: list[Landmark] = [Landmark(0, 0.5, 0.5), Landmark(1, 0.5, 4.5)]
    isam = gtsam_unstable.IncrementalFixedLagSmoother(10)
    fig, ax = Plot.subplots(1, 2, 12, 6)
    p0 = Plot(isam, "p0", fig, ax[0])
    p1 = Plot(isam, "p1", fig, ax[1])
    fig.tight_layout()
    robot_x: gtsam.Pose2 = gtsam.Pose2(1, 2.5, -math.pi / 2)
    prev_robot_x = robot_x
    initialize(isam, landmarks, robot_x)
    # gtsam uses compile-time types so the only way to sort out which variable
    # is which actual type is by keeping a little list.
    pose_variables: list[X] = [X(0)]
    for x_i in range(1, 200):
        robot_delta: gtsam.Pose2 = forward_and_left()
        robot_x = prev_robot_x.compose(robot_delta)
        prev_robot_x = robot_x
        t0 = time.time_ns()
        # only one isam.update() is allowed per time step
        add_odometry_and_target_sights(isam, x_i, robot_x, robot_delta, landmarks)
        result: gtsam.Values = isam.calculateEstimate()
        # print(dir(result))
        factors: gtsam.NonlinearFactorGraph = isam.getFactors()
        # for f_i in range(factors.size()):
        #     if  factors.exists(f_i):
        #         print(type(factors.at(f_i)))
        #         print(factors.at(f_i))
        # print(factors)
        pose_variables.append(X(x_i))
        pose_variables = [pv for pv in pose_variables if result.exists(pv)]
        t1 = time.time_ns()
        if x_i % 5 == 0:
            print(f"i {x_i} duration (ns) {t1-t0}")
        p0.plot_variables(result, pose_variables, landmarks)
        p1.plot_variables(result, pose_variables, landmarks)


if __name__ == "__main__":
    main()
