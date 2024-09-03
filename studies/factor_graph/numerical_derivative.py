# pylint: disable=C0103,C0114,C0116,E0611,R0913
# mypy: disable-error-code="import-untyped"
# see numericalDerivative.h

# i couldn't figure out how to make the wrapper work
# with the std::function argument so i'm just reimplementing
# the parts i need.

from typing import Callable, TypeVar
import numpy as np

from gtsam import (
    Cal3_S2,
    Cal3DS2,
    CalibratedCamera,
    PinholeCameraCal3_S2,
)
from gtsam import Point2, Point3, Pose2, Rot2, Pose3, Rot3, Unit3


def local(a, b):
    if isinstance(a, (np.ndarray, float, int)):
        return b - a
    return a.localCoordinates(b)


def retract(a, b):
    if isinstance(a, (np.ndarray, float, int)):
        return a + b
    return a.retract(b)


Y = TypeVar("Y")
X = TypeVar("X")


def numericalDerivative11(
    h: Callable[[X], Y], x: X, m: int, N: int, delta=1e-5
) -> np.ndarray:
    hx = h(x)
    dx = np.zeros(N)
    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = local(hx, h(retract(x, dx)))
        dx[j] = -delta
        dy2 = local(hx, h(retract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H

X1 = TypeVar("X1")
X2 = TypeVar("X2")

def numericalDerivative21(
        h: Callable[[X1, X2], Y], x1: X1, x2: X2, m: int, N: int, delta=1e-5
) -> np.array:
    return numericalDerivative11(lambda x: h(x, x2), x1, m, N, delta)

def numericalDerivative22(
        h: Callable[[X1, X2], Y], x1: X1, x2: X2, m: int, N: int, delta=1e-5
) -> np.array:
    return numericalDerivative11(lambda x: h(x1, x), x2, m, N, delta)



def numericalDerivative31Point2Pose3Pose3Cal3DS2(
    h: Callable[[Pose3, Pose3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Pose3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x, x2, x3), x1, 2, 6, delta)


def numericalDerivative32Point2Pose3Pose3Cal3DS2(
    h: Callable[[Pose3, Pose3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Pose3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x1, x, x3), x2, 2, 6, delta)


def numericalDerivative33Point2Pose3Pose3Cal3DS2(
    h: Callable[[Pose3, Pose3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Pose3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x1, x2, x), x3, 2, 9, delta)




def numericalDerivative31Point2Pose3Unit3Cal3_S2(
    h: Callable[[Pose3, Unit3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Unit3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x, x2, x3), x1, 2, 6, delta)


def numericalDerivative32Point2Pose3Unit3Cal3_S2(
    h: Callable[[Pose3, Unit3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Unit3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x1, x, x3), x2, 2, 6, delta)


def numericalDerivative33Point2Pose3Unit3Cal3_S2(
    h: Callable[[Pose3, Unit3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Unit3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x1, x2, x), x3, 2, 5, delta)


def numericalDerivative31Point2Pose3Point3Cal3_S2(
    h: Callable[[Pose3, Point3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Point3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x, x2, x3), x1, 2, 6, delta)


def numericalDerivative32Point2Pose3Point3Cal3_S2(
    h: Callable[[Pose3, Point3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Point3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x1, x, x3), x2, 2, 3, delta)


def numericalDerivative33Point2Pose3Point3Cal3_S2(
    h: Callable[[Pose3, Point3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Point3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x1, x2, x), x3, 2, 5, delta)






def numericalDerivative22Pose3Pose3Pose3(
    h: Callable[[Pose3, Pose3], Pose3], x1: Pose3, x2: np.array, delta=1e-5
) -> np.array:
    return numericalDerivative11(lambda x: h(x1, x), x2, 6, 6, delta)


def numericalDerivative31Pose3Pose3Pose3Double(
    h: Callable[[Pose3, Pose3, float], Pose3],
    x1: Pose3,
    x2: np.array,
    x3: float,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x, x2, x3), x1, 6, 6, delta)


def numericalDerivative32Pose3Pose3Pose3Double(
    h: Callable[[Pose3, Pose3, float], Pose3],
    x1: Pose3,
    x2: np.array,
    x3: float,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11(lambda x: h(x1, x, x3), x2, 6, 6, delta)







def numericalDerivative61Vector6DoubleDoubleDoubleDoubleDoubleDouble(
    h, x1, x2, x3, x4, x5, x6, delta=1e-5
):
    return numericalDerivative11(lambda a1: h(a1, x2, x3, x4, x5, x6), x1, 6, 1, delta)
