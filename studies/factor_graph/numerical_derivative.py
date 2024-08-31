# pylint: disable=C0103,C0114,C0116,R0913
# see numericalDerivative.h

# i couldn't figure out how to make the wrapper work
# with the std::function argument so i'm just reimplementing
# the parts i need.

from typing import Callable
import numpy as np

from gtsam import Point2


def VectorLocal(a, b):
    """See VectorSpace.h, TangentVector.Local is difference."""
    return b - a


def VectorRetract(a, b):
    """See VectorSpace.h, TangentVector.Retract is sum."""
    return a + b


def numericalGradientVector2(
    h: Callable[[np.array], float], x: np.array, delta=1e-5
) -> np.array:
    N = 2
    factor = 1.0 / (2.0 * delta)
    d = np.zeros(N)
    g = np.zeros(N)
    for j in range(N):
        d[j] = delta
        hxplus: float = h(VectorRetract(x, d))
        d[j] = -delta
        hxmin: float = h(VectorRetract(x, d))
        d[j] = 0
        g[j] = (hxplus - hxmin) * factor
    return g

def numericalDerivative21DoublePoint2Point2(
        h:Callable[[Point2, Point2], float], x1: Point2, x2: Point2, delta=1e-5 
) -> np.array:
    return numericalDerivative11DoublePoint2(
        lambda x: h(x, x2), x1, delta
    )
    
def numericalDerivative22DoublePoint2Point2(
        h:Callable[[Point2, Point2], float], x1: Point2, x2: Point2, delta=1e-5 
) -> np.array:
    return numericalDerivative11DoublePoint2(
        lambda x: h(x1, x), x2, delta
    )
    

def numericalDerivative11DoublePoint2(
    h: Callable[[Point2], float], x: Point2, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 2  # for now
    hx = h(x)
    m: int = 1

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(VectorRetract(x, dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(VectorRetract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H    


def numericalDerivative11VectorDouble(
    h: Callable[[float], np.array], x: float, delta=1e-5
) -> np.array:
    N = 1  # for now
    hx = h(x)
    m: int = np.shape(hx)[0]

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(VectorRetract(x, dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(VectorRetract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative61Vector6DoubleDoubleDoubleDoubleDoubleDouble(
    h, x1, x2, x3, x4, x5, x6, delta=1e-5
):
    return numericalDerivative11VectorDouble(
        lambda a1: h(a1, x2, x3, x4, x5, x6), x1, delta
    )
