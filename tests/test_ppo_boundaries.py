"""Regression tests for PPO bootstrapping and episode boundaries.

Run with: python3 -m unittest discover -s tests -p 'test_ppo_boundaries.py' -v
"""
import unittest

import numpy as np
import torch

from training.algorithms.ppo import PPO
from utils.buffer import RolloutBuffer


class ScriptedEnv:
    """Each observation's first component is its known value estimate."""

    def __init__(self, transitions, reset_value=100.0):
        self.transitions = iter(transitions)
        self.reset_value = reset_value
        self.resets = 0

    def step(self, action):
        next_value, reward, terminated, truncated = next(self.transitions)
        return (np.array([next_value], dtype=np.float32), reward,
                terminated, truncated, {})

    def reset(self):
        self.resets += 1
        return np.array([self.reset_value], dtype=np.float32), {}


def collector(n_steps):
    # Fixed value estimates make expected targets independent of network weights.
    agent = PPO.__new__(PPO)
    agent.n_steps = n_steps
    agent.gamma = 0.9
    agent.num_timesteps = 0
    agent.buffer = RolloutBuffer()
    agent.get_value = lambda obs: float(obs[0])
    agent.act = lambda obs: (np.zeros(2, dtype=np.float32), 0.0, float(obs[0]))
    return agent


class EpisodeBoundaryTests(unittest.TestCase):
    def test_mid_rollout_timeout_bootstraps_once_and_stops_gae(self):
        agent = collector(3)
        env = ScriptedEnv([
            (3.0, 1.0, False, False),
            (10.0, 2.0, False, True),
            (200.0, 1000.0, False, False),
        ])
        ep_stats = [0.0, 0]
        _, last_value, finished = agent.collect_rollout(
            env, np.array([2.0]), ep_stats=ep_stats)
        advantages, returns = agent.buffer.compute_gae(last_value, 0.9, 0.8)

        # Timeout target: 2 + .9 * 10 = 11. The reset state's value of 100
        # and following episode reward of 1000 must not enter this target.
        np.testing.assert_allclose(agent.buffer.rewards, [1.0, 11.0, 1000.0])
        np.testing.assert_allclose(advantages, [7.46, 8.0, 1080.0], rtol=1e-6)
        np.testing.assert_allclose(returns, [9.46, 11.0, 1180.0], rtol=1e-6)
        self.assertEqual(finished, [3.0])  # Log real rewards, not value estimates.
        self.assertEqual(ep_stats, [1000.0, 1])
        self.assertEqual(env.resets, 1)

    def test_timeout_at_rollout_end_ignores_reset_value(self):
        agent = collector(1)
        env = ScriptedEnv([(10.0, 2.0, False, True)], reset_value=999.0)
        _, last_value, _ = agent.collect_rollout(env, np.array([3.0]))
        advantages, returns = agent.buffer.compute_gae(last_value, 0.9, 0.8)
        self.assertEqual(last_value, 999.0)
        np.testing.assert_allclose(advantages, [8.0])
        np.testing.assert_allclose(returns, [11.0])

    def test_terminal_has_no_bootstrap_even_when_also_truncated(self):
        for truncated in (False, True):
            with self.subTest(truncated=truncated):
                agent = collector(2)
                env = ScriptedEnv([
                    (999.0, -5.0, True, truncated),
                    (200.0, 1000.0, False, False),
                ])
                _, last_value, finished = agent.collect_rollout(env, np.array([3.0]))
                advantages, returns = agent.buffer.compute_gae(last_value, 0.9, 0.8)
                self.assertEqual(agent.buffer.rewards[0], -5.0)
                self.assertEqual(advantages[0], -8.0)
                self.assertEqual(returns[0], -5.0)
                self.assertEqual(finished, [-5.0])

    def test_nonterminal_rollout_cutoff_retains_bootstrap_and_gae(self):
        agent = collector(2)
        env = ScriptedEnv([
            (3.0, 1.0, False, False),
            (10.0, 2.0, False, False),
        ])
        _, last_value, finished = agent.collect_rollout(env, np.array([2.0]))
        advantages, returns = agent.buffer.compute_gae(last_value, 0.9, 0.8)
        np.testing.assert_allclose(advantages, [7.46, 8.0], rtol=1e-6)
        np.testing.assert_allclose(returns, [9.46, 11.0], rtol=1e-6)
        self.assertEqual(finished, [])
        self.assertEqual(env.resets, 0)


class PolicyProbabilityTests(unittest.TestCase):
    def test_sampled_action_log_probability_matches_update(self):
        for policy_type in ("hybrid", "gaussian"):
            with self.subTest(policy_type=policy_type):
                agent = PPO(3, 2, hidden_sizes=(8, 8), policy_type=policy_type,
                            device="cpu", seed=7)
                observations = np.random.default_rng(4).normal(size=(32, 3)).astype(np.float32)
                samples = [agent.act(obs) for obs in observations]
                actions = np.stack([sample[0] for sample in samples])
                old_log_probs = [sample[1] for sample in samples]
                with torch.no_grad():
                    new_log_probs, _, _ = agent.policy.evaluate_actions(
                        torch.from_numpy(observations), torch.from_numpy(actions))
                np.testing.assert_allclose(new_log_probs.numpy(), old_log_probs, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
