# pylint: disable=invalid-name,too-many-statements,no-name-in-module,no-member,missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement

import numpy as np

from gtsam.symbol_shorthand import L  # type:ignore


class Landmark:

    def __init__(self, i, x, y):
        self.symbol = L(i)
        self.x = np.array([x, y])
