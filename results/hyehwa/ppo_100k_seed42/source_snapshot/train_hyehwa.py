"""Train, evaluate, and replay PPO on the imported Hyehwa road network.

Examples (from the repository root):
  .venv-hyehwa/bin/python train_hyehwa.py --mode train --steps 100000
  .venv-hyehwa/bin/python train_hyehwa.py --mode evaluate --run-dir results/hyehwa/RUN
  .venv-hyehwa/bin/python train_hyehwa.py --mode replay --run-dir results/hyehwa/RUN

Validation seeds select checkpoints. Held-out evaluation seeds are used only by
the separate evaluate command, with matched routes/traffic for all controllers.
"""
import argparse
from collections import deque
import csv
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np
import torch

from algorithms.ppo import PPO


ROOT = Path(__file__).resolve().parent
EPISODE_FIELDS = [
    "split", "policy", "checkpoint_step", "episode", "seed", "route_id",
    "return", "steps", "collision", "success", "timeout", "other_failure", "lane_changes",
    "safety_interventions", "safety_intervention_rate", "mean_speed",
    "speed_sum", "speed_count", "mean_gap", "min_gap", "gap_count",
    "elapsed_seconds", "terminal_info",
]


def clean_json(value):
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean_json(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return clean_json(value.tolist())
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def save_json(path, data):
    path = Path(path)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(clean_json(data), ensure_ascii=False, indent=2)
                    + "\n", encoding="utf-8")
    temp.replace(path)


class TableLog:
    """Flush every record so a long run is inspectable before it finishes."""

    def __init__(self, prefix, fields, jsonl=False):
        self.csv_file = Path(prefix).with_suffix(".csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=fields)
        self.writer.writeheader()
        self.csv_file.flush()
        self.json_file = (Path(prefix).with_suffix(".jsonl").open("w", encoding="utf-8")
                          if jsonl else None)

    def write(self, row):
        data = clean_json(row)
        csv_data = {k: json.dumps(v, ensure_ascii=False) if isinstance(v, (dict, list))
                    else v for k, v in data.items()}
        self.writer.writerow(csv_data)
        self.csv_file.flush()
        if self.json_file:
            self.json_file.write(json.dumps(data, ensure_ascii=False) + "\n")
            self.json_file.flush()

    def close(self):
        self.csv_file.close()
        if self.json_file:
            self.json_file.close()


class EpisodeStats:
    def __init__(self, **metadata):
        self.metadata = metadata
        self.started = time.monotonic()
        self.total_reward = 0.0
        self.steps = self.lane_changes = self.interventions = 0
        self.speeds, self.gaps = [], []
        self.collision = self.success = False

    def add(self, reward, info):
        self.total_reward += float(reward)
        self.steps += 1
        self.collision |= bool(info.get("collided", info.get("collision", False)))
        self.success |= bool(info.get("arrived", info.get("success", False)))
        if "lane_changes" in info:
            self.lane_changes = int(info["lane_changes"])
        else:
            self.lane_changes += int(bool(info.get("lane_change_applied", False)))
        self.interventions += int(bool(info.get("safety_intervention", False)))
        for key, values in (("speed", self.speeds), ("gap", self.gaps)):
            value = info.get(key)
            if value is not None and math.isfinite(float(value)):
                values.append(float(value))

    def row(self, terminated, truncated, info):
        return {
            **self.metadata,
            "return": self.total_reward, "steps": self.steps,
            "collision": int(self.collision), "success": int(self.success),
            "timeout": int(bool(truncated) and not self.collision and not self.success),
            "other_failure": int(bool(terminated) and not self.collision and not self.success),
            "lane_changes": self.lane_changes,
            "safety_interventions": self.interventions,
            "safety_intervention_rate": self.interventions / max(self.steps, 1),
            "mean_speed": float(np.mean(self.speeds)) if self.speeds else None,
            "speed_sum": sum(self.speeds), "speed_count": len(self.speeds),
            "mean_gap": float(np.mean(self.gaps)) if self.gaps else None,
            "min_gap": min(self.gaps) if self.gaps else None,
            "gap_count": len(self.gaps),
            "elapsed_seconds": time.monotonic() - self.started,
            "terminal_info": {**info, "terminated": bool(terminated), "truncated": bool(truncated)},
        }


class RecordingEnv:
    """Observe full training episodes while PPO owns its usual rollout loop."""

    def __init__(self, env, sink, agent):
        self.env, self.sink, self.agent = env, sink, agent
        self.episode = 0
        self.stats = None

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self.episode += 1
        self.stats = EpisodeStats(
            split="train", policy="ppo_stochastic", checkpoint_step=self.agent.num_timesteps,
            episode=self.episode, seed=info.get("seed", kwargs.get("seed")),
            route_id=info.get("route_id", info.get("route", "")),
        )
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self.stats.add(reward, info)
        if terminated or truncated:
            self.sink.write(self.stats.row(terminated, truncated, info))
        return obs, reward, terminated, truncated, info


def summarize(rows):
    n = len(rows)
    if not n:
        return {"episodes": 0}
    total_steps = sum(row["steps"] for row in rows)
    speed_count = sum(row["speed_count"] for row in rows)
    collisions = sum(row["collision"] for row in rows)
    p, z = collisions / n, 1.96
    # Wilson interval describes sampling uncertainty, not real-world safety.
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return {
        "episodes": n, "collision_rate": p,
        "collision_rate_wilson95": [max(0, center - half), min(1, center + half)],
        "success_rate": sum(row["success"] for row in rows) / n,
        "timeout_rate": sum(row["timeout"] for row in rows) / n,
        "other_failure_rate": sum(row["other_failure"] for row in rows) / n,
        "mean_return": float(np.mean([row["return"] for row in rows])),
        "std_return": float(np.std([row["return"] for row in rows])),
        "mean_steps": total_steps / n,
        "mean_speed": sum(row["speed_sum"] for row in rows) / max(speed_count, 1),
        "mean_lane_changes": sum(row["lane_changes"] for row in rows) / n,
        "safety_intervention_rate": sum(row["safety_interventions"] for row in rows) / max(total_steps, 1),
        "min_gap": min((row["min_gap"] for row in rows if row["min_gap"] is not None), default=None),
    }


def make_env(config, *, gui=False, baseline=False):
    from env.hyehwa_env import HyehwaEnv
    kwargs = dict(config["environment"])
    kwargs.update(gui=gui, baseline=baseline)
    return HyehwaEnv(**kwargs)


def make_agent(config):
    return PPO(config["obs_dim"], config["act_dim"], **config["ppo"])


def evaluate(agent, env, routes, n_episodes, seed_base, policy_name, split,
             checkpoint_step, sink=None, baseline=False):
    rows = []
    for i in range(n_episodes):
        route_id, seed = routes[i % len(routes)], seed_base + i
        obs, _ = env.reset(seed=seed, options={"route_index": env.route_ids.index(route_id)})
        stats = EpisodeStats(split=split, policy=policy_name, checkpoint_step=checkpoint_step,
                             episode=i + 1, seed=seed, route_id=route_id)
        while True:
            action = None if baseline else agent.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            stats.add(reward, info)
            if terminated or truncated:
                row = stats.row(terminated, truncated, info)
                rows.append(row)
                if sink:
                    sink.write(row)
                break
    return rows, summarize(rows)


def save_checkpoint(agent, run_dir, name, validation=None):
    target = run_dir / (name + ".pt")
    temp = target.with_suffix(".pt.tmp")
    agent.save(str(temp))
    temp.replace(target)
    save_json(target.with_suffix(".json"), {
        "checkpoint": target.name, "timesteps": agent.num_timesteps,
        "saved_at_utc": datetime.now(timezone.utc).isoformat(),
        "validation": validation,
        "selection_rule": "maximize validation success_rate, then mean_return",
        "architecture_config": "config.json",
        "weights_only": True,
    })


def runtime_metadata():
    versions = {}
    for name in ("numpy", "torch", "gymnasium", "traci", "sumolib", "eclipse-sumo", "matplotlib"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    result = subprocess.run(["sumo", "--version"], capture_output=True, text=True, check=False)
    commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                            capture_output=True, text=True, check=False)
    return {"python": sys.version, "executable": sys.executable,
            "packages": versions, "sumo_version": result.stdout.strip(),
            "git_commit": commit.stdout.strip(), "torch_threads": torch.get_num_threads(),
            "cuda_available": torch.cuda.is_available()}


def snapshot_sources(run_dir):
    names = ["train_hyehwa.py", "algorithms/ppo.py", "utils/buffer.py", "utils/networks.py",
             "env/hyehwa_env.py", "build_hyehwa.py", "env/hyehwa/routes.json",
             "env/hyehwa/manifest.json", "env/hyehwa/source.json", "env/hyehwa/hyehwa.sumocfg",
             "env/hyehwa/hyehwa.net.xml", "env/hyehwa/traffic.rou.xml", "env/hyehwa/hyehwa.osm.xml"]
    hashes = {}
    for name in names:
        path = ROOT / name
        if path.is_file():
            hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
            if name in names:
                target = run_dir / "source_snapshot" / name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, target)
    save_json(run_dir / "source_hashes.json", hashes)
    return hashes


def plot_run(run_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        train_path = run_dir / "training_log.csv"
        validation_path = run_dir / "validation_summary.json"
        if train_path.exists():
            with train_path.open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            validation = json.loads(validation_path.read_text()) if validation_path.exists() else []
            fig, axes = plt.subplots(1, 3, figsize=(14, 4))
            if rows:
                axes[0].plot([int(row["steps"]) for row in rows],
                             [float(row["recent_return"]) if row["recent_return"] else np.nan for row in rows])
            axes[0].set_title("Stochastic training return (last 20 episodes)")
            for key, ax, title in (("mean_return", axes[1], "Deterministic validation return"),
                                   ("success_rate", axes[2], "Validation success / collision rate")):
                if validation:
                    ax.plot([row["steps"] for row in validation], [row[key] for row in validation], label=key)
                ax.set_title(title)
            if validation:
                axes[2].plot([row["steps"] for row in validation],
                             [row["collision_rate"] for row in validation], label="collision_rate")
                axes[2].legend()
            for ax in axes:
                ax.set_xlabel("Training transitions")
                ax.grid(alpha=0.25)
            fig.tight_layout()
            fig.savefig(run_dir / "learning_curves.png", dpi=160)
            plt.close(fig)
        test_path = run_dir / "evaluation_summary.json"
        if test_path.exists():
            summary = json.loads(test_path.read_text())["policies"]
            names = list(summary)
            fig, axes = plt.subplots(1, 3, figsize=(13, 4))
            for ax, key in zip(axes, ["mean_return", "success_rate", "safety_intervention_rate"]):
                ax.bar(names, [summary[name][key] for name in names])
                ax.set_title(key)
                ax.tick_params(axis="x", rotation=15)
                ax.grid(axis="y", alpha=0.25)
            fig.tight_layout()
            fig.savefig(run_dir / "evaluation_comparison.png", dpi=160)
            plt.close(fig)
    except ImportError as exc:
        print(f"[plot] unavailable: {exc}", flush=True)


def train(args, run_dir):
    from env.hyehwa_env import HyehwaEnv
    route_data = json.loads((ROOT / "env/hyehwa/routes.json").read_text(encoding="utf-8"))
    routes = sorted(route_data)
    environment = {"cfg_path": str(ROOT / "env/hyehwa/hyehwa.sumocfg"),
                   "ego_routes": {name: route_data[name] for name in routes},
                   "seed": args.seed, "max_steps": args.max_episode_steps,
                   "log_dir": str(run_dir / "sumo_logs"), "step_length": 0.5,
                   "warmup_steps": 40, "visibility": 100.0,
                   "strategic_route_assist": args.strategic_route_assist}
    env = HyehwaEnv(**environment)
    config = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "environment": environment, "routes": routes,
        "obs_dim": int(env.observation_space.shape[0]), "act_dim": int(env.action_space.shape[0]),
        "ppo": {"lr": args.lr, "n_steps": args.rollout_steps, "n_epochs": 6,
                "minibatch_size": 128, "gamma": 0.99, "gae_lambda": 0.95,
                "clip_eps": 0.2, "value_coef": 0.5, "entropy_coef": 0.002,
                "lane_entropy_coef": 0.005, "max_grad_norm": 0.5,
                "policy_type": "hybrid", "lane_keep_bias": 1.0,
                "hidden_sizes": [128, 128], "activation": "tanh", "init_log_std": -0.5,
                "seed": args.seed, "device": "cpu"},
        "training": {"requested_steps": args.steps, "validation_interval": args.eval_interval,
                     "validation_episodes": args.validation_episodes, "validation_seed_base": 10000,
                     "test_seed_base": 20000, "checkpoint_selection": ["success_rate", "mean_return"]},
        "evaluation_policy": "deterministic; initial_untrained uses initial random weights",
        "runtime": runtime_metadata(),
    }
    config["source_hashes"] = snapshot_sources(run_dir)
    manifest_path = ROOT / "env/hyehwa/manifest.json"
    config["network_manifest"] = json.loads(manifest_path.read_text()) if manifest_path.exists() else None
    # Reset reloads this frozen network/traffic snapshot, so a later road rebuild
    # cannot silently alter a running experiment or its evaluation/replay.
    frozen_cfg = run_dir / "source_snapshot/env/hyehwa/hyehwa.sumocfg"
    if frozen_cfg.is_file():
        config["environment"]["cfg_path"] = str(frozen_cfg)
        env.cfg_path = str(frozen_cfg)
    save_json(run_dir / "config.json", config)
    agent = make_agent(config)
    validation_env = make_env(config)
    train_log = TableLog(run_dir / "training_log", [
        "steps", "update", "recent_return", "finished_episodes", "elapsed_seconds", "steps_per_second",
        "policy_loss", "value_loss", "entropy", "approx_kl", "clip_frac", "p_lane_change", "std"])
    episode_log = TableLog(run_dir / "training_episodes", EPISODE_FIELDS, jsonl=True)
    validation_log = TableLog(run_dir / "validation_episodes", EPISODE_FIELDS, jsonl=True)
    recorded = RecordingEnv(env, episode_log, agent)
    validations, recent = [], deque(maxlen=20)
    best_score = (-float("inf"), -float("inf"))
    started = time.monotonic()
    completed, total_episodes, updates = False, 0, 0

    def validate():
        nonlocal best_score
        _, summary = evaluate(agent, validation_env, routes, args.validation_episodes,
                              10000, "ppo", "validation", agent.num_timesteps, validation_log)
        validations.append({"steps": agent.num_timesteps, **summary})
        save_json(run_dir / "validation_summary.json", validations)
        score = (summary["success_rate"], summary["mean_return"])
        if score > best_score:
            best_score = score
            save_checkpoint(agent, run_dir, "best", summary)
        print(f"[validation] steps={agent.num_timesteps} success={summary['success_rate']:.1%} "
              f"collision={summary['collision_rate']:.1%} return={summary['mean_return']:.2f}", flush=True)

    try:
        save_checkpoint(agent, run_dir, "initial")
        validate()
        obs, _ = recorded.reset(seed=args.seed)
        ep_stats = [0.0, 0]
        next_eval = args.eval_interval
        while agent.num_timesteps < args.steps:
            remaining = args.steps - agent.num_timesteps
            agent.n_steps = min(args.rollout_steps, remaining)
            if remaining - agent.n_steps == 1:
                agent.n_steps -= 1  # Leave >=2 transitions for normalized advantages.
            obs, last_value, finished = agent.collect_rollout(recorded, obs, ep_stats=ep_stats)
            advantages, returns = agent.buffer.compute_gae(last_value, agent.gamma, agent.gae_lambda)
            metrics = agent.update(advantages, returns)
            recent.extend(finished)
            total_episodes += len(finished)
            updates += 1
            elapsed = time.monotonic() - started
            row = {"steps": agent.num_timesteps, "update": updates,
                   "recent_return": float(np.mean(recent)) if recent else None,
                   "finished_episodes": total_episodes, "elapsed_seconds": elapsed,
                   "steps_per_second": agent.num_timesteps / elapsed,
                   **metrics, "std": agent.policy.current_std}
            train_log.write(row)
            print(f"[train] steps={agent.num_timesteps}/{args.steps} episodes={total_episodes} "
                  f"return20={row['recent_return']} fps={row['steps_per_second']:.1f}", flush=True)
            if agent.num_timesteps >= next_eval or agent.num_timesteps == args.steps:
                validate()
                next_eval = agent.num_timesteps + args.eval_interval
                save_checkpoint(agent, run_dir, "latest")
        completed = True
    finally:
        save_checkpoint(agent, run_dir, "final" if completed else "interrupted")
        env.close()
        validation_env.close()
        for log in (train_log, episode_log, validation_log):
            log.close()
        save_json(run_dir / "training_status.json", {
            "completed": completed, "timesteps": agent.num_timesteps,
            "requested_steps": args.steps, "finished_episodes": total_episodes,
            "updates": updates, "wall_seconds": time.monotonic() - started,
            "best_validation_score": list(best_score),
            "held_out_evaluation_run": False,
        })
        plot_run(run_dir)
    print(f"[saved] {run_dir}\n"
          f"[next] {sys.executable} train_hyehwa.py --mode evaluate --run-dir {run_dir}", flush=True)


def evaluate_run(args, run_dir, config):
    selected = Path(args.checkpoint) if args.checkpoint else run_dir / "best.pt"
    if not selected.is_file():
        selected = run_dir / selected
    checkpoint_meta_path = selected.with_suffix(".json")
    checkpoint_meta = json.loads(checkpoint_meta_path.read_text()) if checkpoint_meta_path.exists() else {}
    sink = TableLog(run_dir / "evaluation_episodes", EPISODE_FIELDS, jsonl=True)
    summaries, route_summaries = {}, {}
    started = time.monotonic()
    try:
        for policy_name, checkpoint, baseline in (
            ("initial_untrained", run_dir / "initial.pt", False),
            ("sumo_baseline", None, True), ("ppo_best", selected, False),
        ):
            env = make_env(config, baseline=baseline)
            agent = make_agent(config)
            if checkpoint:
                agent.load(str(checkpoint))
            try:
                rows, summary = evaluate(agent, env, config["routes"], args.eval_episodes,
                                         config["training"]["test_seed_base"], policy_name, "test",
                                         checkpoint_meta.get("timesteps", 0) if policy_name == "ppo_best" else 0,
                                         sink, baseline=baseline)
            finally:
                env.close()
            summaries[policy_name] = summary
            route_summaries[policy_name] = {
                route: summarize([row for row in rows if row["route_id"] == route])
                for route in config["routes"]}
            print(f"[test] {policy_name}: {json.dumps(clean_json(summary), ensure_ascii=False)}", flush=True)
    finally:
        sink.close()
    save_json(run_dir / "evaluation_summary.json", {
        "split": "held_out_test", "seed_base": config["training"]["test_seed_base"],
        "episodes_per_policy": args.eval_episodes, "routes": config["routes"],
        "route_schedule": "sorted routes cyclically, identical seed + route per controller",
        "selected_checkpoint": str(selected), "checkpoint_metadata": checkpoint_meta,
        "policies": summaries, "by_route": route_summaries,
        "wall_seconds": time.monotonic() - started,
        "limitations": ["A single training seed is evaluated.",
                        "SUMO safety assistance remains enabled; intervention rates must accompany collision rates.",
                        "SUMO baseline has no external action requests, so its intervention rate is structurally zero and action-request penalties do not apply.",
                        "OSM road geometry and simulated traffic do not reproduce measured real-world traffic."],
    })
    status_path = run_dir / "training_status.json"
    if status_path.exists():
        status = json.loads(status_path.read_text())
        status["held_out_evaluation_run"] = True
        status["held_out_episodes_per_policy"] = args.eval_episodes
        save_json(status_path, status)
    plot_run(run_dir)


def replay(args, run_dir, config):
    selected = Path(args.checkpoint) if args.checkpoint else run_dir / "best.pt"
    if not selected.is_file():
        selected = run_dir / selected
    agent = make_agent(config)
    agent.load(str(selected))
    env = make_env(config, gui=True)
    try:
        routes = [args.route] if args.route else config["routes"]
        rows, summary = evaluate(agent, env, routes, args.replay_episodes, args.replay_seed,
                                 "ppo_replay", "replay", 0)
        print(json.dumps(clean_json(summary), ensure_ascii=False, indent=2))
    finally:
        env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("train", "evaluate", "replay"), default="train")
    parser.add_argument("--steps", type=int, default=100000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--checkpoint")
    parser.add_argument("--rollout-steps", type=int, default=2048)
    parser.add_argument("--lr", type=float, default=3e-4)
    parser.add_argument("--max-episode-steps", type=int, default=1200)
    parser.add_argument("--strategic-route-assist", action="store_true",
                        help="Let SUMO assist with mandatory route lane choices (recorded in config)")
    parser.add_argument("--eval-interval", type=int, default=20000, help="Training transitions between validation runs")
    parser.add_argument("--validation-episodes", type=int, default=8)
    parser.add_argument("--eval-episodes", type=int, default=40, help="Held-out episodes per controller in evaluate mode")
    parser.add_argument("--replay-episodes", type=int, default=3)
    parser.add_argument("--replay-seed", type=int, default=30000)
    parser.add_argument("--route", help="Optional route name for GUI replay")
    args = parser.parse_args()
    if args.steps < 2 or args.rollout_steps < 2:
        parser.error("--steps and --rollout-steps must be at least 2")
    if min(args.eval_interval, args.validation_episodes, args.eval_episodes,
           args.replay_episodes, args.max_episode_steps) < 1:
        parser.error("Episode counts, max steps, and evaluation interval must be positive")
    if args.mode != "train" and args.run_dir is None:
        parser.error("--run-dir is required for evaluate/replay")
    os.environ["PATH"] = str(Path(sys.executable).parent) + os.pathsep + os.environ.get("PATH", "")
    torch.set_num_threads(2)
    run_dir = (args.run_dir or ROOT / "results/hyehwa" / datetime.now().strftime("%Y%m%d_%H%M%S")).resolve()
    if args.mode == "train":
        run_dir.mkdir(parents=True, exist_ok=True)
        if (run_dir / "config.json").exists():
            parser.error(f"Training output already exists: {run_dir}; choose a new run directory")
        train(args, run_dir)
    else:
        config = json.loads((run_dir / "config.json").read_text(encoding="utf-8"))
        # Resolve the archived map relative to this run, including after copying
        # an experiment to another machine or directory.
        frozen_cfg = run_dir / "source_snapshot/env/hyehwa/hyehwa.sumocfg"
        if not frozen_cfg.is_file():
            parser.error(f"Archived road configuration is missing: {frozen_cfg}")
        config["environment"]["cfg_path"] = str(frozen_cfg)
        config["environment"]["log_dir"] = str(run_dir / "sumo_logs")
        if args.mode == "evaluate":
            evaluate_run(args, run_dir, config)
        else:
            replay(args, run_dir, config)


if __name__ == "__main__":
    main()
