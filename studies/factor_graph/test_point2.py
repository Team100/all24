# really to test numeric differentation
# see testPoint.cpp

import math

import unittest
import numpy as np
from gtsam import Matrix, Point2


double norm2(const Point2& p, OptionalJacobian<1,2> H) {
  double r = std::sqrt(p.x() * p.x() + p.y() * p.y());
  if (H) {
    if (std::abs(r) > 1e-10)
      *H << p.x() / r, p.y() / r;
    else
      *H << 1, 1;  // really infinity, why 1 ?
  }
  return r;
}


def distance2(p: Point2,  q: Point2,  H1: np.array, H2: np.array) -> float:
    """
    H1: OptionalJacobian<1, 2>
    H2: OptionalJacobian<1, 2>
    """
    d:Point2 = q - p
    if (H1 || H2):
        H: np.zeros((1,2))
        r: float = norm2(d, H)
        if (H1) *H1 = -H;
        if (H2) *H2 =  H;
        return r;
    else:
        return d.norm();
    


class TestPoint2(unittest.TestCase):

    def test_norm(self):
        p0 = Point2(math.cos(5.0), math.sin(5.0));
        DOUBLES_EQUAL(1, p0.norm(), 1e-6);
        p1 = Point2(4, 5)
        p2 = Point2(1, 1);
        DOUBLES_EQUAL( 5, distance2(p1, p2), 1e-6);
        DOUBLES_EQUAL( 5, (p2-p1).norm(), 1e-6);

        Matrix expectedH, actualH;
        double actual;

        // exception, for (0,0) derivative is [Inf,Inf] but we return [1,1]
        actual = norm2(x1, actualH);
        EXPECT_DOUBLES_EQUAL(0, actual, 1e-9);
        expectedH = (Matrix(1, 2) << 1.0, 1.0).finished();
        EXPECT(assert_equal(expectedH,actualH));

        actual = norm2(x2, actualH);
        EXPECT_DOUBLES_EQUAL(sqrt(2.0), actual, 1e-9);
        expectedH = numericalDerivative11(norm_proxy, x2);
        EXPECT(assert_equal(expectedH,actualH));

        // analytical
        expectedH = (Matrix(1, 2) << x2.x()/actual, x2.y()/actual).finished();
        EXPECT(assert_equal(expectedH,actualH));
        

if __name__ == "__main__":
    unittest.main()
