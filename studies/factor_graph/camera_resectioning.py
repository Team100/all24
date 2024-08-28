# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

# this is a 1:1 transcription of CameraResectioning.cpp.

import numpy as np

import gtsam
from gtsam import Cal3_S2, Point2, Point3, Pose3, Rot3, Values
from gtsam.symbol_shorthand import X


#
def resectioning_factor(
    model: gtsam.noiseModel,
    key: int,
    calib: Cal3_S2,
    p: Point2,
    P: Point3,
) -> gtsam.NonlinearFactor:

    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        pose = v.atPose3(this.keys()[0])
        camera = gtsam.PinholeCameraCal3_S2(pose, calib)
        if H is None:
            return camera.project(P) - p
        Dpose = np.zeros((2, 6), order='F')
        result = camera.project(P, Dpose, np.zeros((2, 3), order='F'), np.zeros((2, 5), order='F')) - p
        H[0] = Dpose
        return result

    return gtsam.CustomFactor(
        model,
        gtsam.KeyVector([key]),
        error_func,
    )


def main() -> None:
    """
    Camera: f = 1, Image: 100x100, center: 50, 50.0
    Pose (ground truth): (Xw, -Yw, -Zw, [0,0,2.0]')
                              ^    ^ ??? what are these negative signs
    Known landmarks:
       3D Points: (10,10,0) (-10,10,0) (-10,-10,0) (10,-10,0)
    Perfect measurements:
       2D Point:  (55,45)   (45,45)    (45,55)     (55,55)
    """

    # read camera intrinsic parameters
    calib = gtsam.Cal3_S2(1, 1, 0, 50, 50)

    # 1. create graph
    graph = gtsam.NonlinearFactorGraph()

    # 2. add factors to the graph
    # add measurement factors
    measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.5, 0.5]))

    graph.add(
        resectioning_factor(
            measurement_noise, X(1), calib, Point2(55, 45), Point3(10, 10, 0)
        )
    )
    graph.add(
        resectioning_factor(
            measurement_noise, X(1), calib, Point2(45, 45), Point3(-10, 10, 0)
        )
    )
    graph.add(
        resectioning_factor(
            measurement_noise, X(1), calib, Point2(45, 55), Point3(-10, -10, 0)
        )
    )
    graph.add(
        resectioning_factor(
            measurement_noise, X(1), calib, Point2(55, 55), Point3(10, -10, 0)
        )
    )

    # 3. Create an initial estimate for the camera pose
    initial: Values = Values()
    # this "estimate" is in fact the solution, so the solver doesn't do any work.
    # initial.insert(X(1), Pose3(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(0, 0, 2)))
    # this is a much worse initial estimate
    initial.insert(X(1), Pose3(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(0, 0, 5)))

    # 4. Optimize the graph using Levenberg-Marquardt
    result: Values = gtsam.LevenbergMarquardtOptimizer(graph, initial).optimize()
    result.print("Final result:\n")

    print(graph.dot(result))


if __name__ == "__main__":
    main()
