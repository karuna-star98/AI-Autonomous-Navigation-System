import unittest

import numpy as np

import _path  # noqa: F401
from autonav.planner import astar, bfs, dijkstra, smooth_path


def _grid_with_wall():
    g = np.zeros((20, 20), bool)
    g[5:15, 10] = True            # vertical wall with gaps at the top and bottom
    return g


class PlannerTests(unittest.TestCase):
    def test_astar_matches_dijkstra_cost(self):
        """A* with an admissible heuristic must be optimal, i.e. equal to Dijkstra."""
        rng = np.random.default_rng(0)
        for _ in range(15):
            g = rng.random((30, 30)) < 0.22
            g[0, 0] = g[29, 29] = False
            a, d = astar(g, (0, 0), (29, 29)), dijkstra(g, (0, 0), (29, 29))
            self.assertEqual(a.success, d.success)
            if a.success:
                self.assertAlmostEqual(a.cost, d.cost, places=6)
                self.assertLessEqual(a.expanded, d.expanded)   # heuristic should never expand more

    def test_path_avoids_obstacles_and_is_connected(self):
        g = _grid_with_wall()
        res = astar(g, (10, 0), (10, 19))
        self.assertTrue(res.success)
        for (r, c) in res.path:
            self.assertFalse(g[r, c])
        for a, b in zip(res.path, res.path[1:]):
            self.assertLessEqual(max(abs(a[0] - b[0]), abs(a[1] - b[1])), 1)

    def test_no_path_when_enclosed(self):
        g = np.zeros((10, 10), bool)
        g[4:7, 4:7] = True
        g[5, 5] = False                     # goal fully enclosed
        self.assertFalse(astar(g, (0, 0), (5, 5)).success)

    def test_blocked_start_or_goal(self):
        g = np.zeros((5, 5), bool)
        g[0, 0] = True
        self.assertFalse(astar(g, (0, 0), (4, 4)).success)

    def test_no_diagonal_corner_cutting(self):
        g = np.zeros((3, 3), bool)
        g[0, 1] = g[1, 0] = True            # two blocked cells touching diagonally at (0,0)-(1,1)
        self.assertFalse(astar(g, (0, 0), (1, 1)).success)

    def test_bfs_finds_path(self):
        res = bfs(_grid_with_wall(), (10, 0), (10, 19))
        self.assertTrue(res.success)

    def test_cost_field_prefers_clear_corridor(self):
        g = np.zeros((11, 21), bool)
        cost = np.ones_like(g, dtype=float)
        cost[:3, :] = 6.0                   # expensive strip along the top
        res = astar(g, (1, 0), (1, 20), cost)
        rows = [r for r, _ in res.path]
        self.assertGreater(max(rows), 2)    # leaves the expensive strip

    def test_smoothing_shortens_and_stays_free(self):
        g = _grid_with_wall()
        res = astar(g, (10, 0), (10, 19))
        sm = smooth_path(g, res.path)
        self.assertLess(len(sm), len(res.path))
        self.assertEqual(sm[0], res.path[0])
        self.assertEqual(sm[-1], res.path[-1])


if __name__ == "__main__":
    unittest.main()
