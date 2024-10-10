import unittest
import numpy as np
from numpy.typing import NDArray

from app.dashboard.real_display import RealDisplay

Mat = NDArray[np.uint8]


class DisplayTest(unittest.TestCase):
    def test_display(self) -> None:
        display: RealDisplay = RealDisplay(100, 100, 0)
        img: Mat = np.zeros([100, 100], dtype=np.uint8)
        display.draw_text(img, "hi", (0, 0))
        # TODO: add an assertion
