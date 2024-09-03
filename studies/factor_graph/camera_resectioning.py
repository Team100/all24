# pylint: disable=consider-using-from-import,invalid-name,no-name-in-module,no-member,missing-function-docstring
"""
This is a 1:1 transcription of CameraResectioning.cpp.
"""
import time
import numpy as np
from gtsam import Cal3_S2, CustomFactor, LevenbergMarquardtOptimizer, KeyVector
from gtsam import LevenbergMarquardtParams
from gtsam import NonlinearFactor, NonlinearFactorGraph
from gtsam import PinholeCameraCal3_S2, Point2, Point3, Pose3, Rot3, Values
from gtsam.noiseModel import Base as SharedNoiseModel, Diagonal
from gtsam.symbol_shorthand import X


def resectioning_factor(
    model: SharedNoiseModel,
    key: int,
    calib: Cal3_S2,
    p: Point2,
    P: Point3,
) -> NonlinearFactor:

    def error_func(this: CustomFactor, v: Values, H: list[np.ndarray]) -> np.ndarray:
        pose = v.atPose3(this.keys()[0])
        print("pose pose ", pose)
        camera = PinholeCameraCal3_S2(pose, calib)
        print("camera pose ", camera.pose())
        if H is None:
            return camera.project(P) - p
        Dpose = np.zeros((2, 6), order="F")
        Dpoint = np.zeros((2, 3), order="F")
        Dcal = np.zeros((2, 5), order="F")

        # q = pose.transformTo(P)
        # print("pose ", pose)
        # print("P ", P)
        # print("q ", q)

        result = camera.project(P, Dpose, Dpoint, Dcal) - p
        H[0] = Dpose
        # print("Dpose ", Dpose)
        return result

    return CustomFactor(model, KeyVector([key]), error_func)


def main() -> None:
    """
    Camera: f = 1, Image: 100x100, center: 50, 50.0
    Pose (ground truth): (Xw, -Yw, -Zw, [0,0,2.0]')
    Known landmarks:
       3D Points: (10,10,0) (-10,10,0) (-10,-10,0) (10,-10,0)
    Perfect measurements:
       2D Point:  (55,45)   (45,45)    (45,55)     (55,55)
    """

    # read camera intrinsic parameters
    calib = Cal3_S2(1, 1, 0, 50, 50)

    # 1. create graph
    graph = NonlinearFactorGraph()

    # 2. add factors to the graph
    measurement_noise = Diagonal.Sigmas(np.array([0.5, 0.5]))
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
    # note: the optimizer will traverse negative-z (chirality-violating) territory
    # on the way to the correct solution, so turn off the cmake option
    # GTSAM_THROW_CHEIRALITY_EXCEPTION to prevent an exception from being thrown.
    initial: Values = Values()
    initial.insert(X(1), Pose3(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(0, 0, 5)))

    # 4. Optimize the graph using Levenberg-Marquardt
    params = LevenbergMarquardtParams()
    # Dellaert suggested using a high initial lambda
    # to get around the chirality exception
    params.setlambdaInitial(1e6)

    t0 = time.time_ns()
    result: Values = LevenbergMarquardtOptimizer(graph, initial, params).optimize()
    t1 = time.time_ns()
    result.print("Final result:\n")
    print("duration ms ", (t1-t0)/1000000)

if __name__ == "__main__":
    main()
