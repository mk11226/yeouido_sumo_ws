"""Real-map Hyehwa junction driving environment with SUMO safety constraints.

The route is assigned before departure; the policy controls longitudinal
acceleration and lane requests, while SUMO follows the road geometry.  SUMO's
car-following, red-light and junction-priority checks remain enabled.  Thus a
collision-free evaluation measures the *combined policy and safety controller*.

Observation (65 floats, clipped to [-1, 1]):
  0:23 ego speed/acceleration/limit, lane index/count/remaining distance,
       best-lane offset/continuation, route progress/relative next-edge heading,
       next signal distance/red/yellow/green/presence, leader gap/relative speed,
       internal-edge flag, absolute heading sine/cosine, previous action;
  23:65 six nearest vehicles, each: presence, relative forward/left position,
        relative forward/left velocity, relative heading sine/cosine.
Missing neighbors are zero-filled. Missing leader/signal distances are one.
All distances are meters, time seconds, and speeds m/s before normalization.
"""
from __future__ import annotations

import math
import os
from pathlib import Path
import sys
import tempfile
import uuid
import xml.etree.ElementTree as ET

import gymnasium as gym
from gymnasium import spaces
import numpy as np

try:
    import traci
    import traci.constants as tc
    import sumolib
except ImportError:
    _sumo_tools = Path(os.environ.get("SUMO_HOME", "/usr/share/sumo")) / "tools"
    if _sumo_tools.is_dir():
        sys.path.append(str(_sumo_tools))
    import traci
    import traci.constants as tc
    import sumolib


class HyehwaEnv(gym.Env):
    """One ego vehicle in a background-traffic-only SUMO configuration.

    ``ego_routes`` maps readable route IDs to connected external edge IDs.
    ``reset(seed=s, options={"route_index": i})`` supports paired evaluation.
    ``baseline=True`` or ``step(None)`` uses SUMO's own driving behavior.
    Each instance owns a labeled connection; reset reloads the simulation with
    a new seed and unique log, avoiding process startup for every episode.
    """

    metadata = {"render_modes": ["human"], "render_fps": 2}
    OBS_DIM = 65

    def __init__(self, cfg_path, ego_routes, *, net_path=None, gui=False,
                 baseline=False, step_length=0.5, max_steps=600,
                 warmup_steps=40, visibility=100.0, log_dir=None, seed=0,
                 strategic_route_assist=False, route_alignment_reward=0.0,
                 route_misalignment_cost=0.0):
        super().__init__()
        self.cfg_path = str(Path(cfg_path).resolve())
        if net_path is None:
            net_setting = ET.parse(self.cfg_path).find("./input/net-file")
            if net_setting is None:
                raise ValueError("SUMO configuration has no input/net-file")
            net_path = Path(self.cfg_path).parent / net_setting.attrib["value"]
        self.net = sumolib.net.readNet(str(net_path), withInternal=True)
        if isinstance(ego_routes, dict):
            self.route_ids = list(ego_routes)
            self.routes = [list(ego_routes[key]) for key in self.route_ids]
        else:
            self.routes = [list(route) for route in ego_routes]
            self.route_ids = [f"route_{i}" for i in range(len(self.routes))]
        if not self.routes or any(not route for route in self.routes):
            raise ValueError("Provide at least one nonempty ego route")
        for route in self.routes:
            for edge in route:
                self.net.getEdge(edge)  # fail early on stale route files
        self.gui, self.baseline = bool(gui), bool(baseline)
        self.strategic_route_assist = bool(strategic_route_assist)
        self.route_alignment_reward = float(route_alignment_reward)
        if not math.isfinite(self.route_alignment_reward) or self.route_alignment_reward < 0:
            raise ValueError("route_alignment_reward must be finite and nonnegative")
        self.route_misalignment_cost = float(route_misalignment_cost)
        if not math.isfinite(self.route_misalignment_cost) or self.route_misalignment_cost < 0:
            raise ValueError("route_misalignment_cost must be finite and nonnegative")
        self.step_length = float(step_length)
        self.max_steps, self.warmup_steps = int(max_steps), int(warmup_steps)
        self.visibility = float(visibility)
        if self.step_length <= 0 or self.max_steps <= 0 or self.visibility <= 0:
            raise ValueError("step_length, max_steps and visibility must be positive")
        self.max_accel, self.max_decel, self.vmax = 2.6, 4.5, 16.67
        self.log_dir = Path(log_dir or (Path(tempfile.gettempdir()) / "hyehwa_sumo"))
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._instance_id = uuid.uuid4().hex[:10]
        self._label = f"hyehwa_{self._instance_id}"
        self._episode_index, self._initial_seed = 0, int(seed)
        self.conn = None
        self.ego_id = "rl_ego"
        self.obs_dim, self.act_dim = self.OBS_DIM, 2
        self.observation_space = spaces.Box(-1.0, 1.0, (self.OBS_DIM,), np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)
        self._lanes = {lane.getID(): lane for edge in self.net.getEdges(withInternal=True)
                       for lane in edge.getLanes()}
        self._done = True

    def _start(self):
        self._episode_index += 1
        self.episode_log = self.log_dir / (
            f"sumo_{self._instance_id}_{self._episode_index:05d}_seed{self.episode_seed}.log")
        cmd = [sumolib.checkBinary("sumo-gui" if self.gui else "sumo"),
               "-c", self.cfg_path, "--step-length", str(self.step_length),
               "--seed", str(self.episode_seed), "--begin", "0", "--end", "1000000",
               "--collision.action", "remove", "--collision.check-junctions", "true",
               "--time-to-teleport", "-1", "--no-step-log", "true",
               "--no-warnings", "true", "--log", str(self.episode_log)]
        if self.gui:
            cmd += ["--start", "--quit-on-end", "--delay", "40"]
        if self.conn is None:
            traci.start(cmd, label=self._label, doSwitch=False, verbose=False)
            self.conn = traci.getConnection(self._label)
        else:
            self.conn.load(cmd[1:])

    def reset(self, *, seed=None, options=None):
        if self._episode_index == 0 and seed is None:
            seed = self._initial_seed
        super().reset(seed=seed)
        self.episode_seed = int(seed if seed is not None
                                else self.np_random.integers(0, 2**31 - 1))
        options = options or {}
        self.route_index = int(options.get("route_index", self.np_random.integers(len(self.routes))))
        if not 0 <= self.route_index < len(self.routes):
            raise ValueError("route_index is outside the available ego routes")
        self.route_id, self.route = self.route_ids[self.route_index], self.routes[self.route_index]
        self.route_length = sum(self.net.getEdge(edge).getLength() for edge in self.route)
        self._start()
        self.step_count = 0
        self._last_action = np.zeros(2, dtype=np.float32)
        self._distance = self._wait_time = self._return = 0.0
        self._lane_changes = self._interventions = self._lane_interventions = 0
        self._longitudinal_interventions = self._red_crossing_candidates = 0
        self._controlled_steps = 0
        for _ in range(self.warmup_steps):
            self.conn.simulationStep()
        self.conn.vehicletype.copy("DEFAULT_VEHTYPE", "rl_ego_type")
        self.conn.vehicletype.setAccel("rl_ego_type", self.max_accel)
        self.conn.vehicletype.setDecel("rl_ego_type", self.max_decel)
        self.conn.vehicletype.setEmergencyDecel("rl_ego_type", 9.0)
        self.conn.vehicletype.setMaxSpeed("rl_ego_type", self.vmax)
        self.conn.vehicletype.setSpeedFactor("rl_ego_type", 1.0)
        self.conn.vehicletype.setImperfection("rl_ego_type", 0.0)
        self.conn.route.add("rl_ego_route", self.route)
        self.conn.vehicle.add(self.ego_id, "rl_ego_route", typeID="rl_ego_type",
                              depart="now", departLane="best", departPos="base",
                              departSpeed="0")
        # Congested insertions are allowed a finite grace period; failure is an
        # environment error, not a fictitious successful/training episode.
        for _ in range(max(20, int(60 / self.step_length))):
            self.conn.simulationStep()
            if self.ego_id in self.conn.vehicle.getIDList():
                break
        else:
            self.close()
            raise RuntimeError(f"Ego insertion failed on route {self.route_id}; see {self.episode_log}")
        self.conn.vehicle.setColor(self.ego_id, (255, 40, 40, 255))
        self.conn.vehicle.setSpeedMode(self.ego_id, 31)
        self._configure_lane_mode(self.baseline)
        self.conn.vehicle.subscribe(self.ego_id, [
            tc.VAR_SPEED, tc.VAR_ACCELERATION, tc.VAR_ANGLE, tc.VAR_POSITION,
            tc.VAR_ROAD_ID, tc.VAR_LANE_ID, tc.VAR_LANE_INDEX,
            tc.VAR_LANEPOSITION, tc.VAR_DISTANCE, tc.VAR_ROUTE_INDEX,
            tc.VAR_ALLOWED_SPEED,
        ])
        self.conn.vehicle.subscribeContext(self.ego_id, tc.CMD_GET_VEHICLE_VARIABLE,
                                           self.visibility,
                                           [tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_ANGLE])
        if self.gui:
            self.conn.gui.trackVehicle("View #0", self.ego_id)
            self.conn.gui.setZoom("View #0", 850)
        self._state = self._snapshot()
        self._start_distance = self._state["distance"]
        self._done = False
        return self._observation(self._state), self._info(self._state)

    def _configure_lane_mode(self, automatic):
        # 512: respect safe gaps for TraCI requests; disable autonomous changes.
        # +1 optionally restores route-strategic lane changes, but no overtaking.
        mode = 1621 if automatic else 512 + int(self.strategic_route_assist)
        self.conn.vehicle.setLaneChangeMode(self.ego_id, mode)

    def _snapshot(self):
        values = self.conn.vehicle.getSubscriptionResults(self.ego_id)
        lane_id = values[tc.VAR_LANE_ID]
        lane = self._lanes[lane_id]
        road = values[tc.VAR_ROAD_ID]
        best = self.conn.vehicle.getBestLanes(self.ego_id)
        current_best = next((entry for entry in best if entry[0] == lane_id), None)
        tls = self.conn.vehicle.getNextTLS(self.ego_id)
        leader = self.conn.vehicle.getLeader(self.ego_id, self.visibility)
        leader_speed = (self.conn.vehicle.getSpeed(leader[0]) if leader and leader[0] else None)
        return {
            "speed": float(values[tc.VAR_SPEED]),
            "accel": float(values[tc.VAR_ACCELERATION]),
            "angle": math.radians(float(values[tc.VAR_ANGLE])),
            "position": np.asarray(values[tc.VAR_POSITION], dtype=float),
            "road": road, "lane_id": lane_id,
            "lane_index": int(values[tc.VAR_LANE_INDEX]),
            "lane_count": lane.getEdge().getLaneNumber(),
            "lane_remaining": max(0.0, lane.getLength() - values[tc.VAR_LANEPOSITION]),
            "distance": max(0.0, float(values[tc.VAR_DISTANCE])),
            "route_index": max(0, int(values[tc.VAR_ROUTE_INDEX])),
            "allowed_speed": max(0.1, float(values[tc.VAR_ALLOWED_SPEED])),
            "best_offset": int(current_best[3]) if current_best else 0,
            "continuation": bool(current_best[4]) if current_best else True,
            "best_lanes": best, "tls": tls[0] if tls else None,
            "gap": max(0.0, float(leader[1])) if leader and leader[0] else None,
            "leader_speed": leader_speed,
        }

    @staticmethod
    def _ego_axes(angle):
        # SUMO angles are clockwise from north; left is perpendicular to heading.
        forward = np.asarray([math.sin(angle), math.cos(angle)])
        left = np.asarray([-math.cos(angle), math.sin(angle)])
        return forward, left

    def _route_heading(self, state):
        index = min(state["route_index"] + 1, len(self.route) - 1)
        shape = self.net.getEdge(self.route[index]).getShape()
        if len(shape) < 2:
            return state["angle"]
        displacement = np.asarray(shape[-1]) - np.asarray(shape[0])
        return math.atan2(displacement[0], displacement[1])

    def _observation(self, state):
        tls = state["tls"]
        light = str(tls[3]).lower() if tls else ""
        relative_heading = self._route_heading(state) - state["angle"]
        traveled = max(0.0, state["distance"] - self._start_distance)
        leader_relative_speed = (state["leader_speed"] - state["speed"]
                                 if state["leader_speed"] is not None else 0.0)
        obs = [state["speed"] / self.vmax, state["accel"] / self.max_decel,
               state["allowed_speed"] / self.vmax,
               state["lane_index"] / max(1, state["lane_count"] - 1),
               state["lane_count"] / 8.0, state["lane_remaining"] / self.visibility,
               state["best_offset"] / 4.0, float(state["continuation"]),
               traveled / max(1.0, self.route_length),
               math.sin(relative_heading), math.cos(relative_heading),
               float(tls[2]) / self.visibility if tls else 1.0,
               float(light == "r"), float(light == "y"), float(light == "g"),
               float(tls is not None),
               state["gap"] / self.visibility if state["gap"] is not None else 1.0,
               leader_relative_speed / self.vmax, float(state["road"].startswith(":")),
               math.sin(state["angle"]), math.cos(state["angle"]),
               float(self._last_action[0]), float(self._last_action[1])]
        forward, left = self._ego_axes(state["angle"])
        ego_velocity = forward * state["speed"]
        nearby = self.conn.vehicle.getContextSubscriptionResults(self.ego_id) or {}
        neighbors = []
        for vehicle_id, values in nearby.items():
            if vehicle_id == self.ego_id:
                continue
            delta = np.asarray(values[tc.VAR_POSITION]) - state["position"]
            angle = math.radians(float(values[tc.VAR_ANGLE]))
            other_forward, _ = self._ego_axes(angle)
            relative_velocity = other_forward * values[tc.VAR_SPEED] - ego_velocity
            neighbors.append((float(delta @ delta), [
                1.0, float(delta @ forward) / self.visibility,
                float(delta @ left) / self.visibility,
                float(relative_velocity @ forward) / self.vmax,
                float(relative_velocity @ left) / self.vmax,
                math.sin(angle - state["angle"]), math.cos(angle - state["angle"])]))
        neighbors.sort(key=lambda item: item[0])
        for _, features in neighbors[:6]:
            obs.extend(features)
        obs.extend([0.0] * (7 * max(0, 6 - len(neighbors))))
        result = np.clip(np.asarray(obs, dtype=np.float32), -1.0, 1.0)
        if result.shape != (self.OBS_DIM,) or not np.isfinite(result).all():
            raise RuntimeError("Invalid Hyehwa observation")
        return result

    def _lane_request(self, command, state):
        """Return (requested, guard-blocked); SUMO still checks vehicle gaps."""
        if not command:
            return False, False
        if state["road"].startswith(":"):
            return False, True
        target = state["lane_index"] + command
        if not 0 <= target < state["lane_count"]:
            return False, True
        target_lane = self.net.getEdge(state["road"]).getLane(target)
        if not target_lane.allows("passenger"):
            return False, True
        # Near the junction, reject lane requests known to strand the fixed route.
        best_target = next((entry for entry in state["best_lanes"]
                            if entry[0] == target_lane.getID()), None)
        if (state["lane_remaining"] < 60 and best_target and not best_target[4]
                and abs(int(best_target[3])) >= abs(state["best_offset"])):
            return False, True
        self.conn.vehicle.changeLane(self.ego_id, target, self.step_length)
        return True, False

    def _red_crossing_candidate(self, previous, current):
        """Conservative diagnostic at step resolution, not a legal adjudication."""
        tls = previous["tls"]
        if not tls or str(tls[3]).lower() != "r":
            return False
        if current["tls"] and current["tls"][:2] == tls[:2]:
            return False
        advance = max(0.0, current["distance"] - previous["distance"])
        if float(tls[2]) > advance + 1.0:
            return False
        state = self.conn.trafficlight.getRedYellowGreenState(tls[0])
        return state[int(tls[1])].lower() == "r"

    def step(self, action):
        if self._done or self.conn is None:
            raise RuntimeError("Call reset() before stepping a completed environment")
        before = self._state
        automatic = self.baseline or action is None
        lane_command, lane_requested, lane_blocked = 0, False, False
        requested_speed = requested_accel = None
        previous_action = self._last_action.copy()
        if automatic:
            self.conn.vehicle.setSpeed(self.ego_id, -1)
            self._configure_lane_mode(True)
            self._last_action[:] = 0
        else:
            action = np.asarray(action, dtype=np.float32)
            if action.shape != (2,) or not np.isfinite(action).all():
                raise ValueError("Action must be two finite values")
            self._last_action = np.clip(action, -1.0, 1.0)
            requested_accel = float(self._last_action[0]) * (
                self.max_accel if self._last_action[0] >= 0 else self.max_decel)
            requested_speed = float(np.clip(before["speed"] + requested_accel * self.step_length,
                                           0.0, before["allowed_speed"]))
            self.conn.vehicle.setSpeed(self.ego_id, requested_speed)
            self._configure_lane_mode(False)
            lane_command = 1 if action[1] > 0.5 else (-1 if action[1] < -0.5 else 0)
            lane_requested, lane_blocked = self._lane_request(lane_command, before)
            self._controlled_steps += 1
        self.conn.simulationStep()
        self.step_count += 1
        collided = self.ego_id in self.conn.simulation.getCollidingVehiclesIDList()
        arrived = self.ego_id in self.conn.simulation.getArrivedIDList()
        teleported = self.ego_id in self.conn.simulation.getStartingTeleportIDList()
        present = self.ego_id in self.conn.vehicle.getIDList()
        removed = not present and not arrived and not collided
        terminated = bool(collided or arrived or teleported or removed)
        truncated = bool(self.step_count >= self.max_steps and not terminated)
        after = self._snapshot() if present and not teleported else None
        self._done = terminated or truncated
        if after:
            advance = max(0.0, after["distance"] - before["distance"])
            self._distance = max(0.0, after["distance"] - self._start_distance)
        else:
            advance = before["speed"] * self.step_length if arrived else 0.0
            self._distance += advance
        lane_changed = bool(after and before["road"] == after["road"]
                            and before["lane_index"] != after["lane_index"])
        self._lane_changes += int(lane_changed)
        # A denied gap-safe request is also a lane intervention, but an edge
        # transition is not counted as a denied lane change.
        lane_denied = bool(lane_blocked or (lane_requested and after
                           and before["road"] == after["road"] and not lane_changed))
        longitudinal_intervention = bool(not automatic and after and
                                          after["speed"] + 0.2 < requested_speed)
        intervention = bool(lane_denied or longitudinal_intervention)
        self._lane_interventions += int(lane_denied)
        self._longitudinal_interventions += int(longitudinal_intervention)
        self._interventions += int(intervention)
        if after and self._red_crossing_candidate(before, after):
            self._red_crossing_candidates += 1
        waiting = bool(after and after["speed"] < 0.1)
        self._wait_time += self.step_length * waiting
        tls = before["tls"]
        waiting_red = bool(waiting and tls and str(tls[3]).lower() in ("r", "y")
                           and float(tls[2]) < 30.0)
        # Reward distance once, then arrival. No per-step speed bonus that could
        # favor longer routes, and no cost for correctly waiting at a red light.
        reward = 0.03 * advance - (0.0 if waiting_red else 0.01 * self.step_length)
        reward -= 0.02 * lane_changed
        # Give timely feedback for a route-required lane change. Only compare
        # offsets within one external edge, so an edge transition cannot create
        # an artificial alignment bonus. Moving away pays the opposite cost.
        alignment_reward = 0.0
        if lane_changed and not before["road"].startswith(":"):
            alignment_reward = self.route_alignment_reward * (
                abs(before["best_offset"]) - abs(after["best_offset"]))
            reward += alignment_reward
        # A route-incompatible lane is still undesirable while stopped at red.
        # Use the new state so a successful correction removes the cost now.
        misalignment_penalty = (self.route_misalignment_cost * self.step_length
                                * abs(after["best_offset"])
                                if after and not after["road"].startswith(":") else 0.0)
        reward -= misalignment_penalty
        if not automatic:
            reward -= 0.002 * abs(float(self._last_action[0] - previous_action[0]))
            reward -= 0.01 * lane_denied
            if not waiting_red:
                reward -= 0.01 * longitudinal_intervention
        failure_reason = None
        if collided:
            reward -= 50.0
            failure_reason = "collision"
        elif arrived:
            reward += 20.0
        elif teleported or removed:
            reward -= 20.0
            failure_reason = "teleported" if teleported else "removed"
        elif truncated:
            reward -= 10.0
            failure_reason = "timeout"
        self._return += reward
        info = self._info(after)
        info.update(collided=bool(collided), arrived=bool(arrived),
                    timeout=truncated, failure_reason=failure_reason,
                    lane_change=lane_command, lane_change_applied=lane_changed,
                    lane_change_requested=lane_requested, lane_intervention=lane_denied,
                    requested_acceleration=requested_accel, requested_speed=requested_speed,
                    actual_acceleration=after["accel"] if after else None,
                    longitudinal_intervention=longitudinal_intervention,
                    safety_intervention=intervention, waiting_red=waiting_red,
                    route_alignment_reward=alignment_reward,
                    route_misalignment_penalty=misalignment_penalty,
                    progress_m=advance, terminated=terminated, truncated=truncated)
        if self._done:
            info["episode"] = {"r": self._return, "l": self.step_count}
        obs = self._observation(after) if after else np.zeros(self.OBS_DIM, dtype=np.float32)
        if after:
            self._state = after
        return obs, float(reward), terminated, truncated, info

    def _info(self, state):
        return {
            "route_id": self.route_id, "route_index": self.route_index,
            "seed": self.episode_seed, "speed": state["speed"] if state else None,
            "gap": state["gap"] if state else None,
            "collided": False, "arrived": False, "failure_reason": None,
            "route_length": self.route_length, "route_distance": self._distance,
            "route_progress": min(1.0, self._distance / max(1.0, self.route_length)),
            "wait_time": self._wait_time, "elapsed_time": self.step_count * self.step_length,
            "lane_changes": self._lane_changes,
            "intervention_count": self._interventions,
            "lane_intervention_count": self._lane_interventions,
            "longitudinal_intervention_count": self._longitudinal_interventions,
            "intervention_rate": self._interventions / max(1, self._controlled_steps),
            "red_light_crossing_candidates": self._red_crossing_candidates,
            "safety_shield_enabled": True,
            "strategic_route_assist": self.strategic_route_assist,
            "sumo_log": str(self.episode_log),
        }

    def close(self):
        if self.conn is not None:
            try:
                self.conn.close(wait=True)
            finally:
                self.conn = None
