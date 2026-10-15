import math
import unittest

import numpy as np

import _path  # noqa: F401
from autonav.config import Config
from autonav.geometry import world_to_cell
from autonav.perception import ObstacleDetector, OccupancyBelief
from autonav.sensors import Camera
from autonav.world import DynamicObstacle, StaticRect, World


def make_world(cfg):
    rects = [StaticRect(10.0, 9.0, 1.5, 2.0)]
    dyn = [DynamicObstacle([(8.0, 12.0), (8.0, 12.0001)], 0.0, 0.4)]
    return World(cfg, "unit", rects, dyn, (3.0, 10.0), (27.0, 10.0))


class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config()
        self.world = make_world(self.cfg)
        self.cam = Camera(self.cfg, np.random.default_rng(0))
        self.pose = (5.0, 10.0, 0.0)      # facing +x; box is 5 m ahead (camera range is 7 m)

    def test_detects_static_and_dynamic_with_correct_class(self):
        det = ObstacleDetector(self.cfg).detect(self.cam.capture(self.world, self.pose))
        classes = sorted(d.cls for d in det.detections)
        self.assertIn("static", classes)
        self.assertIn("dynamic", classes)

    def test_world_position_estimate_is_accurate(self):
        det = ObstacleDetector(self.cfg).detect(self.cam.capture(self.world, self.pose))
        dyn = [d for d in det.detections if d.cls == "dynamic"][0]
        self.assertLess(math.hypot(dyn.world_xy[0] - 8.0, dyn.world_xy[1] - 12.0), 0.3)

    def test_nothing_detected_in_empty_space(self):
        empty = World(self.cfg, "empty", [], [], (3, 10), (27, 10))
        det = ObstacleDetector(self.cfg).detect(self.cam.capture(empty, (15.0, 10.0, 0.0)))
        self.assertEqual(det.detections, [])

    def test_belief_marks_obstacle_cells_occupied_and_free_space_free(self):
        frame = self.cam.capture(self.world, self.pose)
        det = ObstacleDetector(self.cfg).detect(frame)
        belief = OccupancyBelief(self.cfg)
        for k in range(3):
            belief.update(frame, det, 0.1 * k)
        r, c = world_to_cell(self.cfg, 10.75, 10.0)      # centre of the static box
        self.assertTrue(belief.occupied()[r, c])
        r, c = world_to_cell(self.cfg, 6.0, 10.0)        # open floor in view
        self.assertFalse(belief.occupied()[r, c])
        r, c = world_to_cell(self.cfg, 20.0, 3.0)        # never seen
        self.assertFalse(belief.observed[r, c])


if __name__ == "__main__":
    unittest.main()
