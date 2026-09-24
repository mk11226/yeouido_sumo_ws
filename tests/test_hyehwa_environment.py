"""Opt-in real SUMO integration checks, without starting training.

Run with the dedicated runtime (including SUMO on PATH)::

    RUN_SUMO_TESTS=1 .venv-hyehwa/bin/python -m unittest discover -s tests -v

Ordinary unittest discovery skips these relatively expensive process tests.
"""
import json
import os
from pathlib import Path
import tempfile
import unittest


@unittest.skipUnless(os.environ.get("RUN_SUMO_TESTS") == "1", "set RUN_SUMO_TESTS=1")
class HyehwaEnvironmentIntegrationTests(unittest.TestCase):
    def setUp(self):
        from env.hyehwa_env import HyehwaEnv
        self.root = Path(__file__).resolve().parents[1]
        self.tmp = tempfile.TemporaryDirectory(prefix="hyehwa_integration_")
        self.routes = json.loads((self.root / "env/hyehwa/routes.json").read_text())
        self.kwargs = dict(cfg_path=self.root / "env/hyehwa/hyehwa.sumocfg",
                           ego_routes=self.routes, log_dir=self.tmp.name)
        self.env = HyehwaEnv(**self.kwargs)

    def tearDown(self):
        self.env.close()
        self.tmp.cleanup()

    def test_seeded_reload_and_independent_connections(self):
        import numpy as np
        from env.hyehwa_env import HyehwaEnv
        first, _ = self.env.reset(seed=123, options={"route_index": 0})
        self.assertTrue(self.env.observation_space.contains(first))
        other = HyehwaEnv(**self.kwargs)
        try:
            other.reset(seed=456, options={"route_index": 1})
            self.env.step(None)
            other.step(None)
            replay, _ = self.env.reset(seed=123, options={"route_index": 0})
            np.testing.assert_array_equal(first, replay)
            self.assertNotEqual(self.env.conn.getLabel(), other.conn.getLabel())
        finally:
            other.close()

    def test_timeout_is_truncation_and_safety_modes_remain_enabled(self):
        self.env.max_steps = 3
        self.env.reset(seed=42, options={"route_index": 0})
        self.assertEqual(self.env.conn.vehicle.getSpeedMode(self.env.ego_id), 31)
        self.assertEqual(self.env.conn.vehicle.getLaneChangeMode(self.env.ego_id), 512)
        for _ in range(3):
            obs, _, terminated, truncated, info = self.env.step([-1, 0])
            self.assertTrue(self.env.observation_space.contains(obs))
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertEqual(info["failure_reason"], "timeout")
        self.assertFalse(info["arrived"])

    def test_all_routes_are_driveable_by_sumo(self):
        # The multi-phase urban signal cycle can require several waits. This
        # verifies connectivity/arrival within ten minutes, not driving quality.
        self.env.max_steps = 1200
        for index, route_id in enumerate(self.routes):
            with self.subTest(route=route_id):
                self.env.reset(seed=42, options={"route_index": index})
                done = False
                while not done:
                    obs, _, terminated, truncated, info = self.env.step(None)
                    self.assertTrue(self.env.observation_space.contains(obs))
                    done = terminated or truncated
                self.assertTrue(info["arrived"], info)
                self.assertFalse(info["collided"], info)

    def test_alignment_shaping_changes_rewards_without_changing_driving(self):
        """Identical actions/traffic must retain identical dynamics and signals."""
        import hashlib
        import numpy as np
        from env.hyehwa_env import HyehwaEnv

        paths = [self.root / "env/hyehwa" / filename for filename in
                 ("hyehwa.sumocfg", "hyehwa.net.xml", "traffic.rou.xml")]
        original_hashes = [hashlib.sha256(path.read_bytes()).hexdigest() for path in paths]
        self.env.max_steps = 1200
        self.assertEqual(self.env.route_alignment_reward, 0.0)
        self.assertEqual(self.env.route_misalignment_cost, 0.0)
        shaped = HyehwaEnv(**self.kwargs, max_steps=1200,
                           route_alignment_reward=0.5, route_misalignment_cost=0.05)
        route_index = self.env.route_ids.index("northeast_to_south")
        raw_return = shaped_return = awarded = penalties = 0.0
        positive_awards = penalized_steps = red_wait_penalties = 0
        try:
            obs, _ = self.env.reset(seed=42, options={"route_index": route_index})
            shaped_obs, _ = shaped.reset(seed=42, options={"route_index": route_index})
            np.testing.assert_array_equal(obs, shaped_obs)
            lights = self.env.conn.trafficlight.getIDList()
            self.assertTrue(lights)
            self.assertEqual(lights, shaped.conn.trafficlight.getIDList())
            for _ in range(1200):
                # Prescribed diagnostic actions, not a learned-policy result:
                # first reproduce a bad-lane stop at red, then request the
                # indicated route lane. Both environments get identical actions.
                lane = float(np.sign(obs[6])) if obs[18] == 0 and red_wait_penalties else 0.0
                action = [1.0, lane]
                obs, raw_reward, terminated, truncated, info = self.env.step(action)
                shaped_obs, shaped_reward, shaped_terminated, shaped_truncated, shaped_info = shaped.step(action)
                np.testing.assert_array_equal(obs, shaped_obs)
                self.assertEqual((terminated, truncated), (shaped_terminated, shaped_truncated))
                for key in ("arrived", "collided", "route_distance", "lane_changes",
                            "safety_intervention", "failure_reason"):
                    self.assertEqual(info[key], shaped_info[key], key)
                for light in lights:
                    self.assertEqual(self.env.conn.trafficlight.getRedYellowGreenState(light),
                                     shaped.conn.trafficlight.getRedYellowGreenState(light))
                self.assertEqual(info["route_alignment_reward"], 0.0)
                self.assertEqual(info["route_misalignment_penalty"], 0.0)
                bonus = shaped_info["route_alignment_reward"]
                penalty = shaped_info["route_misalignment_penalty"]
                self.assertAlmostEqual(shaped_reward - raw_reward, bonus - penalty, places=10)
                if obs[6] != 0 and obs[18] == 0:
                    penalized_steps += 1
                    self.assertGreater(penalty, 0.0)
                    red_wait_penalties += int(shaped_info["waiting_red"])
                else:
                    self.assertEqual(penalty, 0.0)
                if bonus > 0:
                    positive_awards += 1
                    self.assertTrue(shaped_info["lane_change_applied"])
                    self.assertEqual(bonus, 0.5)
                    self.assertEqual(obs[6], 0.0)
                    self.assertEqual(penalty, 0.0)  # Successful correction removes the cost immediately.
                raw_return += raw_reward
                shaped_return += shaped_reward
                awarded += bonus
                penalties += penalty
                if terminated or truncated:
                    break
            self.assertTrue(info["arrived"], info)
            self.assertGreater(positive_awards, 0)
            self.assertGreater(awarded, 0.0)
            self.assertGreater(penalized_steps, 0)
            self.assertGreater(red_wait_penalties, 0)
            self.assertGreater(penalties, 0.0)
            self.assertAlmostEqual(shaped_return - raw_return, awarded - penalties, places=9)
        finally:
            shaped.close()
        self.assertEqual(original_hashes,
                         [hashlib.sha256(path.read_bytes()).hexdigest() for path in paths])


if __name__ == "__main__":
    unittest.main()
