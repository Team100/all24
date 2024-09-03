# pylint: disable=C0103,C0114,C0116,E0611,R0913
# see numericalDerivative.h

# i couldn't figure out how to make the wrapper work
# with the std::function argument so i'm just reimplementing
# the parts i need.

from typing import Callable
import numpy as np

from gtsam import Cal3_S2, Cal3DS2, CalibratedCamera, PinholeCameraCal3_S2  # type:ignore
from gtsam import Point2, Point3, Pose2, Rot2, Pose3, Rot3, Unit3  # type:ignore


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


def numericalDerivative31Point2Pose3Pose3Cal3DS2(
    h: Callable[[Pose3, Pose3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Pose3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Pose3(lambda x: h(x, x2, x3), x1, delta)

def numericalDerivative32Point2Pose3Pose3Cal3DS2(
    h: Callable[[Pose3, Pose3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Pose3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Pose3(lambda x: h(x1, x, x3), x2, delta)

def numericalDerivative33Point2Pose3Pose3Cal3DS2(
    h: Callable[[Pose3, Pose3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Pose3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Cal3DS2(lambda x: h(x1, x2, x), x3, delta)



def numericalDerivative21Point2Pose3Cal3DS2(
    h: Callable[[Pose3, Cal3DS2], Point2],
    x1: Pose3,
    x2: Cal3DS2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Pose3(
        lambda x: h(x, x2), x1, delta
    )

def numericalDerivative22Point2Pose3Cal3DS2(
    h: Callable[[Pose3, Cal3DS2], Point2],
    x1: Pose3,
    x2: Cal3DS2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Cal3DS2(
        lambda x: h(x1, x), x2, delta
    )



def numericalDerivative21Point2Cal3DS2Point2(
    h: Callable[[Cal3DS2, Point2], Point2],
    x1: Cal3DS2,
    x2: Point2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Cal3DS2(
        lambda x: h(x, x2), x1, delta
    )

def numericalDerivative22Point2Cal3DS2Point2(
    h: Callable[[Cal3DS2, Point2], Point2],
    x1: Cal3DS2,
    x2: Point2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Point2(
        lambda x: h(x1, x), x2, delta
    )

def numericalDerivative11Point2Cal3DS2(
    h: Callable[[Cal3DS2], Point2], x: Cal3DS2, delta=1e-5
) -> np.array:
    N = 9
    hx = h(x)
    m: int = 2

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H



def numericalDerivative21DoublePinholeCameraCal3_S2CalibratedCamera(
    h: Callable[[PinholeCameraCal3_S2, CalibratedCamera], float],
    x1: PinholeCameraCal3_S2,
    x2: CalibratedCamera,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11DoublePinholeCameraCal3_S2(
        lambda x: h(x, x2), x1, delta
    )


def numericalDerivative22DoublePinholeCameraCal3_S2CalibratedCamera(
    h: Callable[[PinholeCameraCal3_S2, CalibratedCamera], float],
    x1: PinholeCameraCal3_S2,
    x2: CalibratedCamera,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11DoubleCalibratedCamera(lambda x: h(x1, x), x2, delta)


def numericalDerivative11DoubleCalibratedCamera(
    h: Callable[[CalibratedCamera], float], x: CalibratedCamera, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 2

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative21DoublePinholeCameraCal3_S2Pose3(
    h: Callable[[PinholeCameraCal3_S2, Pose3], float],
    x1: PinholeCameraCal3_S2,
    x2: Pose3,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11DoublePinholeCameraCal3_S2(
        lambda x: h(x, x2), x1, delta
    )


def numericalDerivative22DoublePinholeCameraCal3_S2Pose3(
    h: Callable[[PinholeCameraCal3_S2, Pose3], float],
    x1: PinholeCameraCal3_S2,
    x2: Pose3,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11DoublePose3(lambda x: h(x1, x), x2, delta)


def numericalDerivative21DoublePinholeCameraCal3_S2Point3(
    h: Callable[[PinholeCameraCal3_S2, Point3], float],
    x1: PinholeCameraCal3_S2,
    x2: Point3,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11DoublePinholeCameraCal3_S2(
        lambda x: h(x, x2), x1, delta
    )


def numericalDerivative22DoublePinholeCameraCal3_S2Point3(
    h: Callable[[PinholeCameraCal3_S2, Point3], float],
    x1: PinholeCameraCal3_S2,
    x2: Point3,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11DoublePoint3(lambda x: h(x1, x), x2, delta)


def numericalDerivative11DoublePinholeCameraCal3_S2(
    h: Callable[[PinholeCameraCal3_S2], float], x: PinholeCameraCal3_S2, delta=1e-5
) -> np.array:
    N = 11
    hx = h(x)
    m: int = 1

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative21Point2PinholeCameraCal3_S2Point3(
    h: Callable[[PinholeCameraCal3_S2, Point3], Point2],
    x1: PinholeCameraCal3_S2,
    x2: Point3,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2PinholeCameraCal3_S2(
        lambda x: h(x, x2), x1, delta
    )


def numericalDerivative22Point2PinholeCameraCal3_S2Point3(
    h: Callable[[PinholeCameraCal3_S2, Point3], Point2],
    x1: PinholeCameraCal3_S2,
    x2: Point3,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Point3(lambda x: h(x1, x), x2, delta)


def numericalDerivative11Point2PinholeCameraCal3_S2(
    h: Callable[[PinholeCameraCal3_S2], Point2], x: PinholeCameraCal3_S2, delta=1e-5
) -> np.array:
    N = 11
    hx = h(x)
    m: int = 2

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative31Point2Pose3Unit3Cal3_S2(
    h: Callable[[Pose3, Unit3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Unit3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Pose3(lambda x: h(x, x2, x3), x1, delta)


def numericalDerivative32Point2Pose3Unit3Cal3_S2(
    h: Callable[[Pose3, Unit3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Unit3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Unit3(lambda x: h(x1, x, x3), x2, delta)


def numericalDerivative33Point2Pose3Unit3Cal3_S2(
    h: Callable[[Pose3, Unit3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Unit3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Cal3_S2(lambda x: h(x1, x2, x), x3, delta)


def numericalDerivative11Point2Unit3(
    h: Callable[[Unit3], Point2], x: Unit3, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 2

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative31Point2Pose3Point3Cal3_S2(
    h: Callable[[Pose3, Point3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Point3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Pose3(lambda x: h(x, x2, x3), x1, delta)


def numericalDerivative32Point2Pose3Point3Cal3_S2(
    h: Callable[[Pose3, Point3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Point3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Point3(lambda x: h(x1, x, x3), x2, delta)


def numericalDerivative33Point2Pose3Point3Cal3_S2(
    h: Callable[[Pose3, Point3, Cal3_S2], Point2],
    x1: Pose3,
    x2: Point3,
    x3: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Point2Cal3_S2(lambda x: h(x1, x2, x), x3, delta)


def numericalDerivative11Point2Pose3(
    h: Callable[[Pose3], Point2], x: Pose3, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 2

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Point2Point3(
    h: Callable[[Point3], Point2], x: Point3, delta=1e-5
) -> np.array:
    N = 3
    hx = h(x)
    m: int = 2

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


def numericalDerivative11Point2Cal3_S2(
    h: Callable[[Cal3_S2], Point2], x: Cal3_S2, delta=1e-5
) -> np.array:
    N = 5
    hx = h(x)
    m: int = 2

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Pose3PinholeCameraCal3_S2(
    h: Callable[[PinholeCameraCal3_S2], Pose3], x: PinholeCameraCal3_S2, delta=1e-5
) -> np.array:
    N = 11
    hx = h(x)
    m: int = 6

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative21PinholeCameraCal3_S2Pose3Cal3_S2(
    h: Callable[[Pose3, Cal3_S2], PinholeCameraCal3_S2],
    x1: Pose3,
    x2: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11PinholeCameraCal3_S2Pose3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22PinholeCameraCal3_S2Pose3Cal3_S2(
    h: Callable[[Pose3, Cal3_S2], PinholeCameraCal3_S2],
    x1: Pose3,
    x2: Cal3_S2,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11PinholeCameraCal3_S2Cal3_S2(
        lambda x: h(x1, x), x2, delta
    )


def numericalDerivative11PinholeCameraCal3_S2Pose3(
    h: Callable[[Pose3], PinholeCameraCal3_S2], x: Pose3, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 11

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11PinholeCameraCal3_S2Cal3_S2(
    h: Callable[[Cal3_S2], PinholeCameraCal3_S2], x: Cal3_S2, delta=1e-5
) -> np.array:
    N = 5
    hx = h(x)
    m: int = 11

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative21Unit3Pose3Point3(
    h: Callable[[Pose3, Point3], Unit3], x1: Pose3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11Unit3Pose3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Unit3Pose3Point3(
    h: Callable[[Pose3, Point3], Unit3], x1: Pose3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11Unit3Point3(lambda x: h(x1, x), x2, delta)


def numericalDerivative11Unit3Pose3(
    h: Callable[[Pose3], Unit3], x: Pose3, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 2

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Unit3Point3(
    h: Callable[[Point3], Unit3], x: Point3, delta=1e-5
) -> np.array:
    N = 3
    hx = h(x)
    m: int = 2

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative21DoublePose3Pose3(
    h: Callable[[Pose3, Pose3], float], x1: Pose3, x2: Pose3, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePose3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22DoublePose3Pose3(
    h: Callable[[Pose3, Pose3], float], x1: Pose3, x2: Pose3, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePose3(lambda x: h(x1, x), x2, delta)


def numericalDerivative21DoublePose3Point3(
    h: Callable[[Pose3, Point3], float], x1: Pose3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePose3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22DoublePose3Point3(
    h: Callable[[Pose3, Point3], float], x1: Pose3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePoint3(lambda x: h(x1, x), x2, delta)


def numericalDerivative21Point3Pose3Point3(
    h: Callable[[Pose3, Point3], Point3], x1: Pose3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11Point3Pose3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Point3Pose3Point3(
    h: Callable[[Pose3, Point3], Point3], x1: Pose3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11Point3Point3(lambda x: h(x1, x), x2, delta)


def numericalDerivative21Pose3Pose3Pose3(
    h: Callable[[Pose3, Pose3], Pose3], x1: Pose3, x2: np.array, delta=1e-5
) -> np.array:
    return numericalDerivative11Pose3Pose3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Pose3Pose3Pose3(
    h: Callable[[Pose3, Pose3], Pose3], x1: Pose3, x2: np.array, delta=1e-5
) -> np.array:
    return numericalDerivative11Pose3Pose3(lambda x: h(x1, x), x2, delta)


def numericalDerivative31Pose3Pose3Pose3Double(
    h: Callable[[Pose3, Pose3, float], Pose3],
    x1: Pose3,
    x2: np.array,
    x3: float,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Pose3Pose3(lambda x: h(x, x2, x3), x1, delta)


def numericalDerivative32Pose3Pose3Pose3Double(
    h: Callable[[Pose3, Pose3, float], Pose3],
    x1: Pose3,
    x2: np.array,
    x3: float,
    delta=1e-5,
) -> np.array:
    return numericalDerivative11Pose3Pose3(lambda x: h(x1, x, x3), x2, delta)


def numericalDerivative21Pose3Rot3Point3(
    h: Callable[[Rot3, Point3], Pose3], x1: Rot3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11Pose3Rot3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Pose3Rot3Point3(
    h: Callable[[Rot3, Point3], Pose3], x1: Rot3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11Pose3Point3(lambda x: h(x1, x), x2, delta)


def numericalDerivative11Pose3Pose3(
    h: Callable[[Pose3], Pose3], x: Pose3, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 6

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Pose3Rot3(
    h: Callable[[Rot3], Pose3], x: Rot3, delta=1e-5
) -> np.array:
    N = 3
    hx = h(x)
    m: int = 6

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Pose3Point3(
    h: Callable[[Point3], Pose3], x: Point3, delta=1e-5
) -> np.array:
    N = 3
    hx = h(x)
    m: int = 6

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


# TODO fix this, x = one-cell "array"
def numericalDerivative11Pose3Double(
    h: Callable[[np.ndarray], Pose3], x: np.ndarray, delta=1e-5
) -> np.array:
    N = 1
    hx: Pose3 = h(x)
    m: int = 6

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative21Vector6Pose3Vector6(
    h: Callable[[Pose3, np.array], np.array], x1: Pose3, x2: np.array, delta=1e-5
) -> np.array:
    return numericalDerivative11Vector6Pose3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Vector6Pose3Vector6(
    h: Callable[[Pose3, np.array], np.array], x1: Pose3, x2: np.array, delta=1e-5
) -> np.array:
    return numericalDerivative11Vector6Vector6(lambda x: h(x1, x), x2, delta)


def numericalDerivative21Vector6Vector6Vector6(
    h: Callable[[np.array, np.array], np.array], x1: np.array, x2: np.array, delta=1e-5
) -> np.array:
    return numericalDerivative11Vector6Vector6(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Vector6Vector6Vector6(
    h: Callable[[np.array, np.array], np.array], x1: np.array, x2: np.array, delta=1e-5
) -> np.array:
    return numericalDerivative11Vector6Vector6(lambda x: h(x1, x), x2, delta)


def numericalDerivative11Vector6Pose3(
    h: Callable[[Pose3], np.array], x: Pose3, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 6

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Pose3Vector6(
    h: Callable[[np.array], Pose3], x: np.array, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 6

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Vector6Vector6(
    h: Callable[[np.array], np.array], x: np.array, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 6

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


def numericalDerivative21DoublePoint3Point3(
    h: Callable[[Point3, Point3], float], x1: Point3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePoint3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22DoublePoint3Point3(
    h: Callable[[Point3, Point3], float], x1: Point3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePoint3(lambda x: h(x1, x), x2, delta)


def numericalDerivative21Point3Point3Point3(
    h: Callable[[Point3, Point3], Point3], x1: Point3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11Point3Point3(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Point3Point3Point3(
    h: Callable[[Point3, Point3], Point3], x1: Point3, x2: Point3, delta=1e-5
) -> np.array:
    return numericalDerivative11Point3Point3(lambda x: h(x1, x), x2, delta)


def numericalDerivative21DoublePoint2Point2(
    h: Callable[[Point2, Point2], float], x1: Point2, x2: Point2, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePoint2(lambda x: h(x, x2), x1, delta)


def numericalDerivative22DoublePoint2Point2(
    h: Callable[[Point2, Point2], float], x1: Point2, x2: Point2, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePoint2(lambda x: h(x1, x), x2, delta)


def numericalDerivative21DoublePose2Pose2(
    h: Callable[[Pose2, Pose2], float], x1: Pose2, x2: Pose2, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePose2(lambda x: h(x, x2), x1, delta)


def numericalDerivative22DoublePose2Pose2(
    h: Callable[[Pose2, Pose2], float], x1: Pose2, x2: Pose2, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePose2(lambda x: h(x1, x), x2, delta)


def numericalDerivative21DoublePose2Point2(
    h: Callable[[Pose2, Point2], float], x1: Pose2, x2: Point2, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePose2(lambda x: h(x, x2), x1, delta)


def numericalDerivative22DoublePose2Point2(
    h: Callable[[Pose2, Point2], float], x1: Pose2, x2: Point2, delta=1e-5
) -> np.array:
    return numericalDerivative11DoublePoint2(lambda x: h(x1, x), x2, delta)


def numericalDerivative21Point2Pose2Point2(
    h: Callable[[Pose2, Point2], Point2], x1: Pose2, x2: Point2, delta=1e-5
) -> np.array:
    return numericalDerivative11Point2Pose2(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Point2Pose2Point2(
    h: Callable[[Pose2, Point2], Point2], x1: Pose2, x2: Point2, delta=1e-5
) -> np.array:
    return numericalDerivative11Point2Point2(lambda x: h(x1, x), x2, delta)


def numericalDerivative21Pose2Pose2Pose2(
    h: Callable[[Pose2, Pose2], Pose2], x1: Pose2, x2: Pose2, delta=1e-5
) -> np.array:
    return numericalDerivative11Pose2Pose2(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Pose2Pose2Pose2(
    h: Callable[[Pose2, Pose2], Pose2], x1: Pose2, x2: Pose2, delta=1e-5
) -> np.array:
    return numericalDerivative11Pose2Pose2(lambda x: h(x1, x), x2, delta)


def numericalDerivative21Rot2Pose2Point2(
    h: Callable[[Pose2, Point2], Rot2], x1: Pose2, x2: Point2, delta=1e-5
) -> np.array:
    return numericalDerivative11Rot2Pose2(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Rot2Pose2Point2(
    h: Callable[[Pose2, Point2], Rot2], x1: Pose2, x2: Point2, delta=1e-5
) -> np.array:
    return numericalDerivative11Rot2Point2(lambda x: h(x1, x), x2, delta)


def numericalDerivative21Rot2Pose2Pose2(
    h: Callable[[Pose2, Pose2], Rot2], x1: Pose2, x2: Pose2, delta=1e-5
) -> np.array:
    return numericalDerivative11Rot2Pose2(lambda x: h(x, x2), x1, delta)


def numericalDerivative22Rot2Pose2Pose2(
    h: Callable[[Pose2, Pose2], Rot2], x1: Pose2, x2: Pose2, delta=1e-5
) -> np.array:
    return numericalDerivative11Rot2Pose2(lambda x: h(x1, x), x2, delta)


def numericalDerivative11Rot2Pose2(
    h: Callable[[Pose2], Rot2], x: Pose2, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 3  # for now
    hx = h(x)
    m: int = 1

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Rot2Point2(
    h: Callable[[Point2], Rot2], x: Point2, delta=1e-5
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
        dy1 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Pose2Vector3(
    h: Callable[[np.array], Pose2], x: np.array, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 3  # for now
    hx: Pose2 = h(x)
    m: int = 3

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(VectorRetract(x, dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Vector3Pose2(
    h: Callable[[Pose2], np.array], x: Pose2, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 3  # for now
    hx = h(x)
    m: int = 3

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11DoublePose3(
    h: Callable[[Pose3], float], x: Pose3, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 1

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11DoublePoint3(
    h: Callable[[Point3], float], x: Point3, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 3  # for now
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


def numericalDerivative11Point2Pose2(
    h: Callable[[Pose2], Point2], x: Pose2, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 3  # for now
    hx = h(x)
    m: int = 2

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Point3Pose3(
    h: Callable[[Pose3], Point3], x: Pose3, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 3

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Rot3Pose3(
    h: Callable[[Pose3], Rot3], x: Pose3, delta=1e-5
) -> np.array:
    N = 6
    hx = h(x)
    m: int = 3

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Point2Point2(
    h: Callable[[Point2], Point2], x: Point2, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 2  # for now
    hx = h(x)
    m: int = 2

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


def numericalDerivative11Pose2Pose2(
    h: Callable[[Pose2], Pose2], x: Pose2, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 3  # for now
    hx = h(x)
    m: int = 3

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = -delta
        dy2 = hx.localCoordinates(h(x.retract(dx)))
        dx[j] = 0
        H[:, j] = (dy1 - dy2) * factor
    return H


def numericalDerivative11Point3Point3(
    h: Callable[[Point3], Point3], x: Point3, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 3  # for now
    hx = h(x)
    m: int = 3

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


def numericalDerivative11DoublePose2(
    h: Callable[[Pose2], float], x: Pose2, delta=1e-5
) -> np.array:
    """Always produces a 2d array."""
    N = 3  # for now
    hx = h(x)
    m: int = 1

    dx = np.zeros(N)

    H = np.zeros((m, N))
    factor: float = 1.0 / (2.0 * delta)
    for j in range(N):
        dx[j] = delta
        dy1 = VectorLocal(hx, h(x.retract(dx)))
        dx[j] = -delta
        dy2 = VectorLocal(hx, h(x.retract(dx)))
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
