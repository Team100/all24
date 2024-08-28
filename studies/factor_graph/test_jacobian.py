# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement


# i am completely confused about how the numpy/eigen boundary works
# for jacobians in custom factors.

import math
import numpy as np
import gtsam
from gtsam import Cal3_S2, Point2, Point3, Pose2, Pose3, Rot3, Values
from gtsam.symbol_shorthand import X


def my_factor(
    model: gtsam.noiseModel,
    key: int,
    m: Point2,
) -> gtsam.NonlinearFactor:

    # this is just distance from m, see https://gtsam.org/tutorials/intro.html

    def error_func(
        this: gtsam.CustomFactor, v: gtsam.Values, H: list[np.ndarray]
    ) -> np.ndarray:
        p = v.atPose2(this.keys()[0])
        if H is not None:
            H[0] = np.array(
                [
                    [p.rotation().c(), -p.rotation().s(), 0],
                    [p.rotation().s(), p.rotation().c(), 0],
                ],
                # this seems not to matter. (!)
                # order="F",
                # order="C",
            )
        result = np.array(p.translation() - m)
        return result

    return gtsam.CustomFactor(
        model,
        gtsam.KeyVector([key]),
        error_func,
    )


def main() -> None:
    graph = gtsam.NonlinearFactorGraph()
    measurement_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01]))
    graph.add(my_factor(measurement_noise, X(1), Point2(-1, 1)))
    initial: Values = Values()
    initial.insert(X(1), Pose2(1, -1, math.pi/4))
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial)
    result: Values = optimizer.optimize()
    result.print("Final result:\n")


if __name__ == "__main__":
    main()
