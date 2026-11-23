import unittest
from unittest.mock import Mock, patch

import numpy as np

from env.driving_profile import navigation_features, unassisted_config


class UnassistedTests(unittest.TestCase):
    def test_observation_removes_only_two_guidance_columns(self):
        from env.yeouido_env import YeouidoEnv
        from env.sumo_env import SumoHighwayEnv
        env = YeouidoEnv.__new__(YeouidoEnv)
        env.ego_id, env.obs_dim = "ego", 64
        env.navigation_observation = True
        env._navigation_net = Mock()
        env.obs_offsets = (+2, +1, 0, -1, -2)
        base = np.arange(31, dtype=np.float32)
        route = np.arange(11, dtype=np.float32) + 100
        navigation = np.arange(24, dtype=np.float32) + 200
        env._route_features = Mock(return_value=route)
        with patch.object(SumoHighwayEnv, "_get_obs", return_value=base), \
                patch("env.yeouido_env.traci.vehicle") as vehicle, \
                patch("env.driving_profile.navigation_features", return_value=navigation):
            vehicle.getIDList.return_value = ["ego"]
            obs = env._get_obs()
            np.testing.assert_array_equal(obs, np.concatenate([base, route[:3], route[5:], navigation]))
            self.assertEqual(obs.shape, (64,))
            vehicle.getIDList.return_value = []
            self.assertEqual(env._get_obs().shape, (64,))
            env.navigation_observation = False
            env.obs_dim = 42
            vehicle.getIDList.return_value = ["ego"]
            np.testing.assert_array_equal(env._get_obs(), np.concatenate([base, route]))

    def test_profile_disables_helpers_without_changing_source(self):
        road = dict(signal_assist=True, safety_shield=True, obey_speed_limit=True,
                    lane_change_mode=257, reroute={"enabled": True})
        action = {"lane_change_safety": {"enabled": True}}
        r, a = unassisted_config(road, action)
        self.assertFalse(r["signal_assist"] or r["safety_shield"] or r["obey_speed_limit"])
        self.assertFalse(r["route_lane_guidance"] or r["reroute"]["enabled"])
        self.assertEqual(r["lane_change_mode"], 0)
        self.assertFalse(a["lane_change_safety"]["enabled"])
        self.assertTrue(road["reroute"]["enabled"])
        self.assertTrue(action["lane_change_safety"]["enabled"])

    def test_lane_map_is_independent_of_route(self):
        straight, left = Mock(), Mock()
        straight.getDirection.return_value = "s"
        left.getDirection.return_value = "l"
        a, b, edge = Mock(), Mock(), Mock()
        a.getID.return_value, b.getID.return_value = "a", "b"
        edge.getOutgoing.return_value = {a: [straight], b: [left]}
        lanes = [Mock(), Mock()]
        lanes[0].getOutgoing.return_value = [straight]
        lanes[1].getOutgoing.return_value = [left]
        edge.getLanes.return_value = lanes
        net = Mock()
        net.hasEdge.return_value = True
        net.getEdge.return_value = edge
        offsets = (+2, +1, 0, -1, -2)
        x = navigation_features(net, "start", 0, ["start", "a"], 0, offsets)
        y = navigation_features(net, "start", 0, ["start", "b"], 0, offsets)
        np.testing.assert_array_equal(x[:4], [1, 0, 0, 0])
        np.testing.assert_array_equal(y[:4], [0, 1, 0, 0])
        np.testing.assert_array_equal(x[4:], y[4:])
        self.assertEqual(len(x), 24)
        self.assertEqual(x[4:8].sum(), 0)  # 존재하지 않는 좌2 차로

    def test_internal_junction_has_no_lane_recommendation(self):
        net = Mock()
        x = navigation_features(net, ":junction", 0, [], 0, (+2, +1, 0, -1, -2))
        self.assertFalse(x.any())
        net.getEdge.assert_not_called()
