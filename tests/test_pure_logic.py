#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""不依赖 ROS2 的核心逻辑回归测试。"""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "kcf_track" / "scripts"))
sys.path.insert(0, str(ROOT / "rescue_console" / "server"))

from kcf_control import FollowController
from bridge import _rle_encode


class PureLogicTests(unittest.TestCase):
    def test_follow_controller_direction_and_limit(self):
        controller = FollowController(
            target_distance=1.2, max_linear_speed=0.3, max_angular_speed=0.4
        )
        self.assertGreater(controller.update(2.0, 320.0)[0], 0.0)
        controller.reset()
        self.assertLess(controller.update(0.5, 320.0)[0], 0.0)
        controller.reset()
        linear, angular = controller.update(2.0, 0.0)
        self.assertEqual((linear, angular), (0.0, 0.0))
        self.assertLessEqual(abs(controller.update(0.0, 320.0)[0]), 0.3)

    def test_rle_encode_preserves_runs(self):
        values = [-1, -1, 0, 0, 100, 100, 100, -1]
        self.assertEqual(_rle_encode(values), [[2, -1], [2, 0], [3, 100], [1, -1]])

    def test_rle_encode_empty_input_is_rejected(self):
        with self.assertRaises(IndexError):
            _rle_encode([])


if __name__ == "__main__":
    unittest.main()
