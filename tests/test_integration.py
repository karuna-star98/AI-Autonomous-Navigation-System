"""End-to-end tests: the whole pipeline must reach the goal without collisions."""
import unittest

import _path  # noqa: F401
from autonav.simulation import Simulation


class EndToEndTests(unittest.TestCase):
    def _check(self, scenario, seed):
        m = Simulation(scenario, seed).run()
        self.assertTrue(m.success, f"{scenario}/{seed} failed: {m.failure_reason}")
        self.assertEqual(m.collisions, 0, f"{scenario}/{seed} had collisions")
        self.assertGreater(m.min_clearance_m, 0.0)
        return m

    def test_warehouse(self):
        self._check("warehouse", 0)

    def test_street(self):
        self._check("street", 0)

    def test_random_world(self):
        self._check("random", 3)

    def test_deterministic_given_seed(self):
        a = Simulation("random", 2).run()
        b = Simulation("random", 2).run()
        self.assertEqual(a.sim_time_s, b.sim_time_s)
        self.assertEqual(a.path_length_m, b.path_length_m)


if __name__ == "__main__":
    unittest.main()
