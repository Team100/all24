import unittest
import numpy as np
from numpy.typing import NDArray

from app.dashboard.real_display import RealDisplay

Mat = NDArray[np.uint8]


class DisplayTest(unittest.TestCase):
    def test_display(self) -> None:
        display: RealDisplay = RealDisplay(100, 100, "foo")
        img: Mat = np.zeros([100, 100], dtype=np.uint8)
        display.text(img, "hi", (0, 0))
        # TODO: add an assertion
