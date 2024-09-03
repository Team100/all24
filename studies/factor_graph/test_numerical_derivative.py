# pylint: disable=no-member,C0103,C0114,C0115,C0116,R0913
# mypy: disable-error-code="import-untyped"
# see testNumericalDerivative.cpp

import math
import unittest
import numpy as np
from numpy.testing import assert_allclose
from numerical_derivative import numericalDerivative61


def f6(x1: float, x2: float, x3: float, x4: float, x5: float, x6: float) -> np.array:
    print(type(x1))
    return np.array(
        [
            math.sin(x1),
            math.cos(x2),
            x3 * x3,
            x4 * x4 * x4,
            math.sqrt(x5),
            math.sin(x6) - math.cos(x6),
        ]
    )


class TestNumericalDerivative(unittest.TestCase):
    def test_numeriDerivative61(self) -> None:
        expected61 = np.array([[math.cos(1)], [0], [0], [0], [0], [0]])
        actual61 = numericalDerivative61(f6, 1, 2, 3, 4, 5, 6)
        assert_allclose(expected61, actual61, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
