import time
import unittest

from app.util.timer import Timer


class TimerTest(unittest.TestCase):
    def test_timer(self) -> None:
        t0: int = Timer.time_ns()
        time.sleep(0.01)
        t1: int = Timer.time_ns()
        self.assertGreater(t1, t0)
