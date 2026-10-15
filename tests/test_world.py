import unittest

import numpy as np

import _path  # noqa: F401
from autonav.config import Config
from autonav.world import DynamicObstacle, StaticRect, World


class WorldTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.w = World(self.cfg, "t", [StaticRect(10, 8, 2, 4)], [], (2, 10), (28, 10))

    def test_raycast_hits_box_at_right_distance(self):
        d = self.w.raycast(5.0, 10.0, np.array([0.0]), 10.0)[0]
        self.assertAlmostEqual(d, 5.0, delta=0.1)

    def test_raycast_max_range_when_clear(self):
        d = self.w.raycast(5.0, 10.0, np.array([np.pi]), 2.0)[0]   # towards open floor 4.6 m from wall
        self.assertAlmostEqual(d, 2.0, delta=1e-6)

    def test_collision_detection(self):
        self.assertTrue(self.w.collides(10.5, 10.0, 0.35))
        self.assertFalse(self.w.collides(5.0, 10.0, 0.35))
        self.assertTrue(self.w.collides(0.3, 10.0, 0.35))          # inside the boundary wall

    def test_dynamic_obstacle_ping_pongs(self):
        d = DynamicObstacle([(0, 0), (4, 0)], 2.0, 0.3)
        d.advance(4.0); self.assertAlmostEqual(d.pos[0], 4.0)
        d.advance(1.0); self.assertAlmostEqual(d.pos[0], 3.0)      # turned around
        d.advance(3.0); self.assertAlmostEqual(d.pos[0], 0.0)
        d.advance(1.0); self.assertAlmostEqual(d.pos[0], 1.0)

    def test_ground_truth_occupancy_grid(self):
        g = self.w.occupancy_grid()
        self.assertEqual(g.shape, (self.cfg.grid_h, self.cfg.grid_w))
        self.assertTrue(g.any())


if __name__ == "__main__":
    unittest.main()
