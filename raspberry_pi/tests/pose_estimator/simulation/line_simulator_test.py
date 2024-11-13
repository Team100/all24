import unittest

from tests.pose_estimator.simulation.line_simulator import LineSimulator


class LineSimulatorTest(unittest.TestCase):
    def test_simple(self) -> None:
        sim = LineSimulator()
        self.assertAlmostEqual(0, sim.gt_x)
        sim.step(0.02)
        self.assertAlmostEqual(0.0002, sim.gt_x)
        sim.step(0.02)
        self.assertAlmostEqual(0.0008, sim.gt_x)
        sim.step(0.02)
        self.assertAlmostEqual(0.0018, sim.gt_x)
