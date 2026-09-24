"""Render headless PPO replays on the actual imported Hyehwa road geometry.

Example::

    .venv-hyehwa/bin/python render_hyehwa_rollout.py \
        --run-dir results/hyehwa/ppo_100k_seed42 --gif

The default is one deterministic replay of every configured route, with seeds
30000 onward and the saved episode horizon. These illustrations are separate
from held-out evaluation. A GIF, when requested, shows the first replay route.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import torch

ROOT = Path(__file__).resolve().parent
ATTRIBUTION = "Road geometry: OpenStreetMap contributors (ODbL); imported into SUMO."
SAFETY_LABEL = "Simulation only | SUMO collision, signal and right-of-way assistance enabled"


def prepare_runtime():
    # Keep this callable from an absolute script path without a shell activation.
    bindirs = [Path(sys.executable).parent, ROOT / ".venv-hyehwa/bin"]
    os.environ["PATH"] = os.pathsep.join(str(path) for path in bindirs) + os.pathsep + os.environ.get("PATH", "")
    if shutil.which("sumo") is None:
        raise RuntimeError("SUMO is not available; use .venv-hyehwa/bin/python or put sumo on PATH")
    torch.set_num_threads(1)


def checkpoint_path(run_dir, requested):
    if requested is None:
        return run_dir / "best.pt"
    requested = Path(requested)
    return requested if requested.is_file() else run_dir / requested


def sample_state(env, step, info=None, action=None, reward=0.0, with_neighbors=False):
    """Read a present ego only; arrived/removed vehicles are handled by caller."""
    state = env._state
    tls = state["tls"]
    info = info or {}
    record = {
        "step": int(step), "time_s": step * env.step_length,
        "x_m": float(state["position"][0]), "y_m": float(state["position"][1]),
        "speed_mps": state["speed"], "acceleration_mps2": state["accel"],
        "road_id": state["road"], "lane_index": state["lane_index"],
        "best_lane_offset": state["best_offset"],
        "tls_state": str(tls[3]) if tls else "none",
        "tls_distance_m": float(tls[2]) if tls else None,
        "requested_acceleration_mps2": info.get("requested_acceleration"),
        "requested_speed_mps": info.get("requested_speed"),
        "safety_intervention": bool(info.get("safety_intervention", False)),
        "lane_intervention": bool(info.get("lane_intervention", False)),
        "waiting_red": bool(info.get("waiting_red", False)), "reward": float(reward),
        "acceleration_action": float(action[0]) if action is not None else None,
        "lane_action": float(action[1]) if action is not None else None,
    }
    if with_neighbors:
        import traci.constants as tc
        neighbors = env.conn.vehicle.getContextSubscriptionResults(env.ego_id) or {}
        record["neighbors"] = [list(values[tc.VAR_POSITION]) for vehicle_id, values in neighbors.items()
                               if vehicle_id != env.ego_id]
    return record


def capture_replays(env, agent, routes, seed_base, output, gif):
    episodes = []
    for index, route_id in enumerate(routes):
        seed = seed_base + index
        obs, _ = env.reset(seed=seed, options={"route_index": env.route_ids.index(route_id)})
        records = [sample_state(env, 0, with_neighbors=gif and index == 0)]
        total_reward = 0.0
        while True:
            action = agent.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            # env._state intentionally retains the last observation after ego
            # disappearance; do not duplicate it as a fabricated arrival sample.
            if info["speed"] is not None:
                records.append(sample_state(env, env.step_count, info, action, reward,
                                            with_neighbors=gif and index == 0))
            if terminated or truncated:
                break
        outcome = "arrived" if info["arrived"] else info.get("failure_reason", "ended")
        episode = {"route": route_id, "seed": seed, "outcome": outcome,
                   "elapsed_s": info["elapsed_time"], "steps": env.step_count,
                   "return": total_reward, "collided": info["collided"],
                   "arrived": info["arrived"], "wait_s": info["wait_time"],
                   "lane_changes": info["lane_changes"],
                   "safety_intervention_rate": info["intervention_rate"],
                   "red_light_crossing_candidates": info["red_light_crossing_candidates"],
                   "records": records}
        path = output / f"trajectory_{route_id}.csv"
        fields = [key for key in records[0] if key != "neighbors"]
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(records)
        episode["trajectory_csv"] = path.name
        episodes.append(episode)
        print(f"[replay] {route_id}: {outcome}, {info['elapsed_time']:.1f}s, "
              f"interventions {info['intervention_rate']:.1%}", flush=True)
    return episodes


def map_geometry(net):
    external, internal = [], []
    for edge in net.getEdges(withInternal=True):
        destination = internal if edge.getID().startswith(":") else external
        destination.extend(np.asarray(lane.getShape()) for lane in edge.getLanes()
                           if len(lane.getShape()) >= 2)
    coords = np.concatenate(external)
    low, high = coords.min(axis=0), coords.max(axis=0)
    margin = (high - low).max() * 0.035
    return external, internal, (low[0] - margin, high[0] + margin,
                                low[1] - margin, high[1] + margin)


def draw_map(ax, geometry):
    from matplotlib.collections import LineCollection
    external, internal, bounds = geometry
    ax.add_collection(LineCollection(external, colors="#c7cdd2", linewidths=1.0, zorder=1))
    ax.add_collection(LineCollection(internal, colors="#e0e4e8", linewidths=0.7, zorder=1))
    ax.set(xlim=bounds[:2], ylim=bounds[2:], aspect="equal", facecolor="#fafbfc")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color("#dce1e6")
    xlo, xhi, ylo, yhi = bounds
    anchor_x, anchor_y = xlo + (xhi - xlo) * 0.07, ylo + (yhi - ylo) * 0.07
    ax.plot([anchor_x, anchor_x + 100], [anchor_y, anchor_y], color="#364350", lw=2)
    ax.text(anchor_x + 50, anchor_y + 12, "100 m", ha="center", fontsize=7, color="#364350")
    ax.annotate("N", xy=(.93, .95), xytext=(.93, .84), xycoords="axes fraction",
                ha="center", va="center", fontsize=9,
                arrowprops={"arrowstyle": "->", "color": "#364350"})


def route_title(route):
    return route.replace("northeast", "NE").replace("southwest", "SW").replace(
        "north", "N").replace("south", "S").replace("_to_", " → ")


def draw_trajectories(episodes, geometry, output, metadata):
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.colors import Normalize
    cols = min(4, len(episodes))
    rows = math.ceil(len(episodes) / cols)
    fig, axes = plt.subplots(rows, cols,
                             figsize=(max(10.0, 4.25 * cols), max(5.7, 4.65 * rows)),
                             squeeze=False)
    normalization = Normalize(0, 60)
    for ax, episode in zip(axes.flat, episodes):
        draw_map(ax, geometry)
        records = episode["records"]
        points = np.asarray([[row["x_m"], row["y_m"]] for row in records])
        speeds = np.asarray([row["speed_mps"] * 3.6 for row in records])
        if len(points) >= 2:
            collection = LineCollection(np.stack([points[:-1], points[1:]], axis=1),
                                        cmap="viridis", norm=normalization,
                                        linewidths=3.2, zorder=3)
            collection.set_array(speeds[:-1])
            ax.add_collection(collection)
        ax.scatter(*points[0], marker="^", s=42, color="#087f5b", edgecolors="white", zorder=5)
        ax.scatter(*points[-1], marker="X", s=48,
                   color="#1a365d" if episode["arrived"] else "#c92a2a", zorder=6)
        ax.set_title(f"{route_title(episode['route'])} · {episode['outcome']}\n"
                     f"{episode['elapsed_s']:.0f} s  |  intervention {episode['safety_intervention_rate']:.0%}",
                     fontsize=10, loc="left", pad=10)
        ax.text(.98, .02, f"seed {episode['seed']}", transform=ax.transAxes,
                ha="right", fontsize=7, color="#617080")
    for ax in list(axes.flat)[len(episodes):]:
        ax.axis("off")
    fig.suptitle(f"Hyehwa, Seoul · deterministic PPO trajectories\n"
                 f"Checkpoint: {metadata['checkpoint_name']} · {metadata['checkpoint_step']:,} training steps",
                 fontsize=15, weight="bold", y=.98)
    fig.subplots_adjust(top=.78 if rows == 1 else .89, bottom=.18 if rows == 1 else .11,
                        left=.03, right=.89, wspace=.12, hspace=.25)
    cax = fig.add_axes([.914, .27, .012, .42])
    bar = fig.colorbar(plt.cm.ScalarMappable(norm=normalization, cmap="viridis"), cax=cax)
    bar.set_label("Actual speed (km/h)")
    fig.text(.03, .065 if rows == 1 else .05,
             "▲ start   X last observed ego position   |   Individual replay examples; not evaluation averages.", fontsize=8)
    fig.text(.03, .037 if rows == 1 else .03, SAFETY_LABEL, fontsize=8, color="#52606d")
    fig.text(.03, .012, ATTRIBUTION + "  https://www.openstreetmap.org/copyright", fontsize=7, color="#52606d")
    path = output / "trajectories.png"
    fig.savefig(path, dpi=170, facecolor="white")
    plt.close(fig)
    return path


def draw_speed_signals(episodes, output):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(episodes), 1, figsize=(12, 2.0 * len(episodes)), squeeze=False)
    for ax, episode in zip(axes.flat, episodes):
        records = episode["records"]
        times = np.asarray([row["time_s"] for row in records])
        speeds = np.asarray([row["speed_mps"] * 3.6 for row in records])
        ax.plot(times, speeds, color="#175e9e", lw=1.3, label="Actual speed")
        red_nearby = [str(row["tls_state"]).lower() in ("r", "y")
                      and row["tls_distance_m"] is not None and row["tls_distance_m"] < 100
                      for row in records]
        ax.fill_between(times, 0, 60, where=red_nearby, color="#f8c3bf", alpha=.55,
                        linewidth=0, label="Next signal red/yellow within 100 m")
        interventions = np.asarray([row["safety_intervention"] for row in records])
        ax.scatter(times[interventions], speeds[interventions], color="#d98324", s=4,
                   zorder=3, label="SUMO/action guard intervention")
        ax.set(ylim=(-2, 62), xlim=(0, max(1, episode["elapsed_s"])), ylabel="km/h")
        ax.set_title(f"{route_title(episode['route'])}  |  {episode['outcome']}  |  seed {episode['seed']}",
                     loc="left", fontsize=10)
        ax.grid(axis="y", color="#e8edf1", lw=.5)
    axes[-1, 0].set_xlabel("Time since ego control began (seconds)")
    axes[0, 0].legend(loc="upper right", fontsize=7, ncol=3)
    fig.suptitle("PPO replay · speed, upcoming signal and interventions", fontsize=15, y=.995)
    fig.text(.08, .01, SAFETY_LABEL, fontsize=8, color="#52606d")
    fig.tight_layout(rect=[0, .025, 1, .985])
    path = output / "speed_signals.png"
    fig.savefig(path, dpi=140, facecolor="white")
    plt.close(fig)
    return path


def draw_gif(episode, geometry, output, max_frames, fps):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    records = episode["records"]
    points = np.asarray([[row["x_m"], row["y_m"]] for row in records])
    frame_indices = np.unique(np.linspace(0, len(records) - 1,
                                          min(max_frames, len(records))).astype(int))
    fig, ax = plt.subplots(figsize=(6.4, 7.4))
    draw_map(ax, geometry)
    ax.plot(points[:, 0], points[:, 1], color="#d6e7f4", linewidth=2, zorder=2)
    trail, = ax.plot([], [], color="#287db0", lw=2.5, zorder=4)
    ego = ax.scatter([], [], c=[], cmap="viridis", vmin=0, vmax=60,
                     s=55, edgecolors="white", linewidths=1, zorder=6)
    traffic = ax.scatter([], [], color="#778591", s=13, zorder=3)
    ax.scatter(*points[0], marker="^", s=45, color="#087f5b", zorder=5)
    status = ax.text(.02, .98, "", transform=ax.transAxes, ha="left", va="top",
                     fontsize=9, bbox={"facecolor": "white", "alpha": .88, "edgecolor": "none"})
    ax.set_title(f"Hyehwa · {route_title(episode['route'])}\n"
                 f"PPO deterministic replay · seed {episode['seed']}", fontsize=13)
    fig.subplots_adjust(top=.89, bottom=.12, left=.04, right=.96)
    fig.text(.04, .072, "Time compressed; gray dots: traffic within 100 m of ego.", fontsize=8)
    fig.text(.04, .047, "SUMO safety assistance enabled | Synthetic signal timing/traffic", fontsize=7)
    fig.text(.04, .023, ATTRIBUTION, fontsize=7)

    def frame(index):
        row = records[index]
        trail.set_data(points[:index + 1, 0], points[:index + 1, 1])
        ego.set_offsets(points[index:index + 1])
        ego.set_array(np.asarray([row["speed_mps"] * 3.6]))
        traffic.set_offsets(np.asarray(row.get("neighbors", []), dtype=float).reshape(-1, 2))
        final_text = f"\nFinal outcome: {episode['outcome']}" if index == len(records) - 1 else ""
        status.set_text(f"t = {row['time_s']:.1f} s   speed = {row['speed_mps'] * 3.6:.1f} km/h\n"
                        f"Next signal: {row['tls_state']}   intervention: {row['safety_intervention']}" + final_text)
        return trail, ego, traffic, status

    animation = FuncAnimation(fig, frame, frames=frame_indices, interval=1000 / fps, blit=False)
    path = output / f"replay_{episode['route']}.gif"
    animation.save(path, writer=PillowWriter(fps=fps), dpi=100)
    plt.close(fig)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--checkpoint", help="Default best.pt; path or filename within run-dir")
    parser.add_argument("--output", type=Path, help="Default run-dir/replay")
    parser.add_argument("--route", action="append", help="Repeat to select routes; default all configured routes")
    parser.add_argument("--seed-base", type=int, default=30000)
    parser.add_argument("--gif", action="store_true", help="Animate first selected route")
    parser.add_argument("--max-frames", type=int, default=120)
    parser.add_argument("--fps", type=int, default=10)
    args = parser.parse_args()
    if args.max_frames < 2 or args.fps < 1:
        parser.error("--max-frames must be >= 2 and --fps must be positive")
    prepare_runtime()
    import matplotlib
    matplotlib.use("Agg")
    from train_hyehwa import make_agent, make_env, save_json
    run_dir = args.run_dir.resolve()
    config = json.loads((run_dir / "config.json").read_text())
    frozen_cfg = run_dir / "source_snapshot/env/hyehwa/hyehwa.sumocfg"
    if not frozen_cfg.is_file():
        parser.error(f"Archived road configuration is missing: {frozen_cfg}")
    config["environment"]["cfg_path"] = str(frozen_cfg)
    selected = checkpoint_path(run_dir, args.checkpoint)
    # Read weights once, so a concurrently replaced best.pt cannot change this
    # replay halfway through. The content hash identifies the exact policy.
    weights = selected.read_bytes()
    checkpoint_metadata_path = selected.with_suffix(".json")
    checkpoint_metadata = (json.loads(checkpoint_metadata_path.read_text())
                           if checkpoint_metadata_path.exists() else {})
    output = (args.output or run_dir / "replay").resolve()
    output.mkdir(parents=True, exist_ok=True)
    config["environment"]["log_dir"] = str(output / "sumo_logs")
    config["ppo"]["device"] = "cpu"
    routes = args.route or config["routes"]
    for route in routes:
        if route not in config["routes"]:
            parser.error(f"Unknown route: {route}")
    agent = make_agent(config)
    agent.policy.load_state_dict(torch.load(io.BytesIO(weights), map_location=agent.device, weights_only=True))
    agent.policy.eval()
    metadata = {"created_at_utc": datetime.now(timezone.utc).isoformat(),
                "checkpoint_name": selected.name,
                "checkpoint_step": checkpoint_metadata.get("timesteps", 0),
                "checkpoint_sha256": hashlib.sha256(weights).hexdigest(),
                "run_dir": str(run_dir), "checkpoint_path": str(selected),
                "checkpoint_metadata": checkpoint_metadata,
                "safety_assistance": SAFETY_LABEL, "attribution": ATTRIBUTION,
                "source_url": "https://www.openstreetmap.org/copyright",
                "seed_base": args.seed_base, "deterministic": True,
                "max_steps": config["environment"]["max_steps"],
                "step_length": config["environment"]["step_length"],
                "purpose": "Illustrative replay, separate from held-out evaluation."}
    cfg_path = Path(config["environment"]["cfg_path"])
    net_file = ET.parse(cfg_path).find("./input/net-file").attrib["value"]
    network_path = cfg_path.parent / net_file
    metadata["network_sha256"] = hashlib.sha256(network_path.read_bytes()).hexdigest()
    env = make_env(config)
    try:
        episodes = capture_replays(env, agent, routes, args.seed_base, output, args.gif)
        geometry = map_geometry(env.net)
    finally:
        env.close()
    artifacts = [draw_trajectories(episodes, geometry, output, metadata),
                 draw_speed_signals(episodes, output)]
    if args.gif:
        artifacts.append(draw_gif(episodes[0], geometry, output, args.max_frames, args.fps))
    metadata["episodes"] = [{key: value for key, value in episode.items() if key != "records"}
                            for episode in episodes]
    metadata["artifacts"] = [path.name for path in artifacts]
    save_json(output / "replay_summary.json", metadata)
    print(f"[saved] {output}", flush=True)


if __name__ == "__main__":
    main()
