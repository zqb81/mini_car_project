#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""不启动 ROS2 的配置契约测试，防止安全链路被无意回退。"""

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class ContractTests(unittest.TestCase):
    def _read(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8")

    def test_twist_mux_uses_unstamped_twist(self):
        config = self._read("src/turn_on_wheeltec_robot/config/twist_mux.yaml")
        self.assertIn("use_stamped: false", config)
        self.assertIn("/cmd_vel_estop_lock", config)

    def test_detection_to_kcf_handoff_contract(self):
        detector = self._read("src/rescue_perception/rescue_perception/detect_target.py")
        tracker = self._read("src/kcf_track/src/runtracker.cpp")
        self.assertIn('"/rescue/target_roi"', detector)
        self.assertIn('"/rescue/target_roi"', tracker)
        self.assertIn("if (!tracking_active_", tracker)

    def test_search_start_is_delayed_until_coordinator_runs(self):
        launch = self._read("src/rescue_perception/launch/rescue_search.launch.py")
        self.assertIn("TimerAction", launch)
        self.assertIn("period=2.0", launch)
        self.assertIn('"fusion_state_timeout"', launch)

    def test_web_control_exposes_estop_and_auth(self):
        app = self._read("rescue_console/server/app.py")
        bridge = self._read("rescue_console/server/bridge.py")
        self.assertIn('@app.post("/api/estop")', app)
        self.assertIn("RESCUE_API_TOKEN", app)
        self.assertIn("compare_digest", app)
        self.assertIn('"/cmd_vel_estop_lock"', bridge)

    def test_console_docs_use_the_actual_venv_path(self):
        readme = self._read("rescue_console/README.md")
        verification = self._read("docs/TARGET_VERIFICATION.md")
        self.assertNotIn("./../../venv/bin/python", readme + verification)
        self.assertIn("../venv/bin/python -m uvicorn", readme + verification)


if __name__ == "__main__":
    unittest.main()
