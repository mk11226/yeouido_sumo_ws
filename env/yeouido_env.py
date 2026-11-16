"""실제 도로 RL 환경 — 영등포구 북부(여의도), 랜드마크 간 주행.

SumoHighwayEnv(env/sumo_env.py)를 상속한다. 행동(가감속 + 차선변경)은 같고, 아래가 다르다.

  도로    : env/yeouido/ (국가표준노드링크 도로망 + TOPIS 실측 교통량, yeouido/ 파이프라인 결과)
  리셋    : 시간대(심야/주간/출근)를 고르고, 배경 교통이 이미 흐르고 있는 워밍업 상태를
            불러온다 (traci.simulation.loadState, 약 0.1초). 프리필은 쓰지 않는다.
  ego     : 랜드마크 두 곳을 골라 출발지 → 목적지 최단 경로로 주행. 목적지 지점에 닿으면 완주.
  보조    : 빨간불 정지(signal_assist), 제한속도(obey_speed_limit),
            경로에 필요한 차로 변경(lane_change_mode=1). 설정은 env/road_config.py YEOUIDO_ROAD.
  관측    : 수업 환경 31차원 + 신호·경로·교차로 9개 = 40차원 (env/mdp_config.py OBSERVATION)
  보상    : 진행 거리 + 시간 벌점 + 빨간불 벌점. 정체·신호 대기 중에는 차간거리·막힘 벌점 없음
            (env/mdp_config.py REWARD의 "여의도 전용")

    env.reset(options={"level": "rush", "origin": "영등포역", "destination": "IFC 서울"})
    env.reset(options={"origin": "5", "destination": "A"})   # 번호로도 지정 (랜드마크 1~7, 게이트 A~F·ㄱ~ㄷ)
    env.change_destination("63빌딩")     # 주행 중 목적지 변경 → 경로 재탐색
"""
import json
import os

import numpy as np
import gymnasium as gym

from env.sumo_env import SumoHighwayEnv, traci, checkBinary
from yeouido.config import LANDMARK_NO
import sumolib

import traci.constants as tc

BASE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(BASE, "yeouido")          # yeouido/ 파이프라인 결과 폴더
STATES = os.path.join(DATA, "states")

STOP_GAP = 2.0          # 빨간불일 때 정지선 앞에 남길 거리 (m)
ASSIST_DECEL = 4.5      # 신호 정지 보조가 쓰는 감속도 (m/s^2, 일반 차량 decel과 동일)
ARRIVE_TOL = 5.0        # 목적지 지점 이 거리(m) 앞이면 도착으로 본다
CONTEXT_RADIUS = 100.0  # get_privileged_state가 모으는 주변 차량 반경 (m)
MAX_PROGRESS = 30.0     # 한 스텝 진행 거리 상한 (m). 재탐색·teleport로 남은 거리가 튀는 것 방지
RED = "rRu"             # 정지 신호 (u = 빨강+노랑)


def load_places(kind="landmarks"):
    """출발·도착 후보 {이름: (edge, pos)}. pos는 edge 시작점부터 m (None = edge 끝)."""
    places = {name: (lm["edge"], lm["pos"])
              for name, lm in json.load(open(os.path.join(DATA, "landmarks.json"))).items()}
    if kind == "landmarks+gates":
        gates = json.load(open(os.path.join(DATA, "count_edges.json")))["gates"]
        for g, v in gates.items():
            # 게이트에서 출발하면 범위 안으로 들어오는 도로, 도착하면 나가는 도로를 쓴다.
            places[f"게이트 {g}"] = (v["in"], v["out"])
    return places


def place_name(places, name):
    """번호·글자로 쓴 출발·도착지를 places 의 이름으로. "5" → "영등포역", "A" → "게이트 A"."""
    if not name or name in places:
        return name
    by_no = {no: n for n, no in LANDMARK_NO.items()}
    for cand in (by_no.get(name), f"게이트 {name}"):
        if cand in places:
            return cand
    raise KeyError(f"모르는 출발·도착지: {name!r} (가능: {', '.join(places)})")


class YeouidoEnv(SumoHighwayEnv):

    def __init__(self, cfg_path, road, ego, mdp_sim, mdp_obs, action, reward,
                 traffic=None, gui=False, gui_autostart=True):
        """인자는 SumoHighwayEnv와 같다. cfg_path는 쓰지 않는다 (시간대별 sumocfg를 고름)."""
        super().__init__(cfg_path, road, ego, mdp_sim, mdp_obs, action, reward,
                         traffic=traffic, gui=gui, gui_autostart=gui_autostart)
        self.road = road
        self.max_steps = int(road.get("max_steps", self.max_steps))
        self.gui_zoom = float(road.get("gui_zoom", self.gui_zoom))
        self.levels = {lv: float(w) for lv, w in road["levels"].items() if float(w) > 0}
        self.snapshot_times = [float(t) for t in road["snapshot_times"]]
        self.min_trip = float(road.get("min_trip", 0.0))
        self.signal_assist = bool(road.get("signal_assist", True))
        self.obey_speed_limit = bool(road.get("obey_speed_limit", True))
        self.lane_change_mode = int(road.get("lane_change_mode", 1))
        self.check_junctions = bool(road.get("check_junction_collisions", True))
        self.signal_range = float(mdp_obs.get("signal_range", 100.0))
        self.route_range = float(mdp_obs.get("route_range", 200.0))
        self.junction_range = float(mdp_obs.get("junction_range", 50.0))

        # 관측: 수업 환경 31차원 + 여의도 전용 9차원
        self.obs_dim += 9
        self.observation_space = gym.spaces.Box(-1.0, 1.0, shape=(self.obs_dim,),
                                                dtype=np.float32)
        # 배경 차량이 새로 생기는 도로 = 들어오는 도로가 없는 범위 경계 도로.
        # 차로 변경 입구 보호(spawn_protect)는 이 도로에서만 적용한다.
        net = sumolib.net.readNet(os.path.join(DATA, "yeouido.net.xml"))
        self.spawn_edges = {e.getID() for e in net.getEdges() if not e.getIncoming()}

        self.places = load_places(road.get("places", "landmarks"))
        self.loaded_level = None    # 지금 떠 있는 SUMO가 읽은 시간대
        self.episode = 0
        self.level = self.origin = self.destination = None
        self.dest_edge, self.dest_pos = None, None
        self.route_length = 0.0
        self.destination_changes = 0
        self.red_lights = 0
        self._remaining = self._prev_remaining = None
        self._signal = 1.0      # 마지막 관측의 앞 신호 색 (보상에서 재사용)
        self._limit = self.vmax
        self._ensure_snapshots()

    # ==================================================================
    # 워밍업 상태 (시간대 × snapshot_times)
    # ==================================================================
    @staticmethod
    def _cfg(level):
        return os.path.join(DATA, f"yeouido_{level}.sumocfg")

    @staticmethod
    def _snapshot(level, t):
        return os.path.join(STATES, f"{level}_{int(t)}.xml.gz")

    def _ensure_snapshots(self):
        """없거나 도로망·교통량보다 오래된 워밍업 상태를 새로 만든다."""
        os.makedirs(STATES, exist_ok=True)
        net_time = os.path.getmtime(os.path.join(DATA, "yeouido.net.xml"))
        for level in self.levels:
            source_time = max(net_time, os.path.getmtime(os.path.join(DATA, f"{level}.rou.xml")))
            files = [self._snapshot(level, t) for t in self.snapshot_times]
            if all(os.path.exists(f) and os.path.getmtime(f) > source_time for f in files):
                continue
            print(f"[yeouido_env] {level} 워밍업 상태 생성 중 ({int(max(self.snapshot_times))}초 분량)...")
            traci.start([checkBinary("sumo"), "-c", self._cfg(level),
                         "--step-length", str(self.step_length),
                         "--no-warnings", "true", "--no-step-log", "true"])
            for t, f in sorted(zip(self.snapshot_times, files)):
                while traci.simulation.getTime() < t:
                    traci.simulationStep()
                # 임시 이름으로 저장 후 바꿔 단다 (병렬 수집 워커가 반쯤 쓴 파일을 읽지 않도록)
                traci.simulation.saveState(f + ".tmp.xml.gz")
                os.replace(f + ".tmp.xml.gz", f)
            traci.close()

    # ==================================================================
    # SUMO 시작 / 리셋
    # ==================================================================
    def _start_sumo(self, sumo_seed=None):
        binary = checkBinary("sumo-gui" if self.gui else "sumo")
        if sumo_seed is None:
            sumo_seed = np.random.randint(0, 100000)
        cmd = [binary, "-c", self._cfg(self.level),
               "--step-length", str(self.step_length),
               "--collision.action", "remove",
               "--no-warnings", "true", "--no-step-log", "true",
               "--seed", str(int(sumo_seed))]
        if self.check_junctions:
            # 교차로 안 충돌도 검사 (기본은 차로 위만). 신호 없는 교차로에서 양보하지 않고
            # 끼어들면 충돌로 끝나도록 한다.
            cmd += ["--collision.check-junctions", "true"]
        if self.gui:
            extra = [os.path.join(DATA, f) for f in ("yeouido.poly.xml", "markers.add.xml")]
            extra = [f for f in extra if os.path.exists(f)]
            if extra:
                cmd += ["--additional-files", ",".join(extra)]
            settings = os.path.join(DATA, "gui.view.xml")
            if os.path.exists(settings):
                cmd += ["--gui-settings-file", settings]
            cmd += ["--delay", str(self.gui_delay)]
            if self.gui_autostart:
                cmd += ["--start"]
        traci.start(cmd)
        self._started = True
        self.loaded_level = self.level

    def _pick(self, options):
        """옵션이 없으면 시간대·출발지·목적지를 무작위로 고른다."""
        rng = self.np_random
        levels = list(self.levels)
        weights = np.array([self.levels[lv] for lv in levels])
        self.level = options.get("level") or str(rng.choice(levels, p=weights / weights.sum()))
        if self.level not in ("night", "day", "rush"):
            raise ValueError(f"알 수 없는 시간대: {self.level}")
        names = list(self.places)
        origin = options.get("origin") or str(
            rng.choice([n for n in names if n != options.get("destination")]))
        destination = options.get("destination") or str(
            rng.choice([n for n in names if n != origin]))
        snapshot_time = options.get("snapshot_time") or float(rng.choice(self.snapshot_times))
        return origin, destination, snapshot_time

    def _place_edge(self, name, depart):
        """(edge, pos). 게이트는 출발이면 들어오는 도로, 도착이면 나가는 도로."""
        a, b = self.places[name]
        if isinstance(b, str):                      # 게이트: (in, out)
            return (a, 0.0) if depart else (b, None)
        return a, b

    def reset(self, seed=None, options=None):
        gym.Env.reset(self, seed=seed)
        options = dict(options or {})
        fixed = {k: place_name(self.places, options.get(k)) for k in ("origin", "destination")}
        # 평가 시험지: sumo_seed가 있으면 그 시드로 SUMO를 새로 켠다 (배경 차량의 무작위 움직임까지 재현).
        # SUMO 시드는 켤 때만 적용되므로, 켜 둔 SUMO를 재사용하는 학습과 달리 매번 다시 켠다.
        sumo_seed = options.get("sumo_seed")
        if sumo_seed is not None and self._started:
            traci.close()
            self._started = False

        for _attempt in range(20):
            origin, destination, snapshot_time = self._pick({**options, **fixed})
            if self._started and self.loaded_level != self.level:
                traci.close()
                self._started = False
            if not self._started:
                self._start_sumo(sumo_seed)
            traci.simulation.loadState(self._snapshot(self.level, snapshot_time))
            if self._insert_ego(origin, destination):
                break
            # 경로가 없거나 너무 짧거나 출발 지점이 막혀 있으면 다른 쌍으로 다시
            if fixed["origin"] and fixed["destination"]:
                raise RuntimeError(f"{origin} → {destination}: ego를 넣지 못했습니다 "
                                   "(경로 없음 또는 출발 지점이 계속 막힘)")
        else:
            raise RuntimeError("ego 출발 지점을 20번 연속 찾지 못했습니다")

        self.origin, self.destination = origin, destination
        self.destination_changes = 0
        self.red_lights = 0
        self.step_count = 0
        self._remaining = self._prev_remaining = None

        # 주변 차량만 받아 오는 구독 (get_privileged_state가 전체 2천 대를 훑지 않도록)
        traci.vehicle.subscribeContext(
            self.ego_id, tc.CMD_GET_VEHICLE_VARIABLE, CONTEXT_RADIUS,
            [tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_LANE_INDEX, tc.VAR_LANE_ID,
             tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION])
        traci.simulationStep()
        self._setup_gui()
        obs = self._get_obs()
        self._prev_remaining = self._remaining
        return obs, self._info_extra()

    def _insert_ego(self, origin, destination) -> bool:
        self.episode += 1
        o_edge, o_pos = self._place_edge(origin, depart=True)
        d_edge, d_pos = self._place_edge(destination, depart=False)
        route = traci.simulation.findRoute(o_edge, d_edge, vType="ego_type")
        if not route.edges:
            return False
        o_len = traci.lane.getLength(f"{o_edge}_0")
        d_len = traci.lane.getLength(f"{d_edge}_0")
        # 출발 지점은 edge 끝에서 20 m 이상 남겨 다음 교차로 전에 차로를 고를 여유를 둔다.
        o_pos = min(max(o_pos, 0.0), max(o_len - 20.0, 0.0))
        d_pos = d_len if d_pos is None else min(max(d_pos, ARRIVE_TOL + 1.0), d_len)
        length = route.length - o_pos - (d_len - d_pos)
        if length < self.min_trip:
            return False

        route_id = f"ego_route_{self.episode}"
        traci.route.add(route_id, route.edges)
        traci.vehicle.add(self.ego_id, route_id, typeID="ego_type",
                          departLane="best", departPos=f"{o_pos:.1f}", departSpeed="0")
        traci.vehicle.setSpeedMode(self.ego_id, 0)
        traci.vehicle.setLaneChangeMode(self.ego_id, self.lane_change_mode)
        for _ in range(60):
            traci.simulationStep()
            if self.ego_id in traci.vehicle.getIDList():
                self.dest_edge, self.dest_pos = d_edge, d_pos
                self.route_length = length
                return True
        traci.vehicle.remove(self.ego_id)       # 60스텝(30초) 동안 출발 지점이 막힘
        return False

    def _setup_gui(self):
        if not self.gui or self.ego_id not in traci.vehicle.getIDList():
            return
        try:
            view_id = traci.gui.getIDList()[0]
            if self.gui_track_ego:
                traci.gui.trackVehicle(view_id, self.ego_id)
                traci.gui.setZoom(view_id, self.gui_zoom)
        except (traci.TraCIException, IndexError):
            pass

    # ==================================================================
    # 행동 보조
    # ==================================================================
    def _target_speed(self, v, accel_cmd):
        target = super()._target_speed(v, accel_cmd)
        if self.obey_speed_limit:
            target = min(target, traci.vehicle.getAllowedSpeed(self.ego_id))
        if self.signal_assist:
            target = min(target, self._signal_cap(v))
        return target

    def _signal_cap(self, v):
        """앞 신호가 정지 신호면 정지선 전에 멈출 수 있는 최대 속도 (아니면 inf).

        한 스텝(dt) 동안 그 속도로 간 뒤 ASSIST_DECEL로 감속해도 정지선을 넘지 않는 속도:
            v·dt + v²/(2b) ≤ d − STOP_GAP
        노란불은 이미 그 거리 안에서 멈출 수 없으면(딜레마 구간) 그대로 통과한다.
        """
        tls = traci.vehicle.getNextTLS(self.ego_id)
        if not tls:
            return float("inf")
        _, _, dist, state = tls[0]
        room = dist - STOP_GAP
        if state in "yY" and v * v / (2.0 * ASSIST_DECEL) > room:
            return float("inf")
        if state not in "rRuyY":
            return float("inf")
        b, dt = ASSIST_DECEL, self.step_length
        return max(0.0, -b * dt + float(np.sqrt((b * dt) ** 2 + 2.0 * b * max(room, 0.0))))

    def _lane_change_is_safe(self, direction):
        # 입구 보호(도로 시작 spawn_protect m 안 변경 금지)는 배경 차량이 생기는 경계 도로에만.
        # 도심 도로 조각은 평균 130 m라 모든 조각에 적용하면 시간의 35%에서 변경이 막힌다.
        protect = self.lc_spawn_protect
        if traci.vehicle.getRoadID(self.ego_id) not in self.spawn_edges:
            self.lc_spawn_protect = 0.0
        try:
            return super()._lane_change_is_safe(direction)
        finally:
            self.lc_spawn_protect = protect

    def _apply_lane_change(self, lane_change):
        # 유지(0)일 때 현재 차로로 changeLane을 걸면 SUMO의 경로상 차로 변경까지 막히므로
        # 아무 명령도 내리지 않는다 (lane_change_mode=1에서 SUMO가 필요한 변경을 한다).
        if lane_change == 0 and self.lane_change_mode != 0:
            return self.ego_id in traci.vehicle.getIDList()
        return super()._apply_lane_change(lane_change)

    # ==================================================================
    # 목적지
    # ==================================================================
    def change_destination(self, name):
        """주행 중 목적지를 바꾼다. SUMO가 현재 위치에서 새 목적지까지 경로를 다시 찾는다."""
        if self.ego_id not in traci.vehicle.getIDList():
            return False
        name = place_name(self.places, name)
        d_edge, d_pos = self._place_edge(name, depart=False)
        try:
            traci.vehicle.changeTarget(self.ego_id, d_edge)
        except traci.TraCIException:
            return False                         # 지금 위치에서 갈 수 없는 목적지
        d_len = traci.lane.getLength(f"{d_edge}_0")
        self.dest_edge = d_edge
        self.dest_pos = d_len if d_pos is None else min(max(d_pos, ARRIVE_TOL + 1.0), d_len)
        self.destination = name
        self.destination_changes += 1
        self._remaining = self._prev_remaining = self.remaining_distance()
        return True

    def remaining_distance(self):
        """목적지까지 남은 도로 거리 (m). ego가 없거나 경로가 끊기면 None."""
        if self.ego_id not in traci.vehicle.getIDList():
            return None
        d = traci.vehicle.getDrivingDistance(self.ego_id, self.dest_edge, self.dest_pos)
        return None if d < -1e9 else float(d)    # SUMO는 도달 불가를 큰 음수로 돌려준다

    def _reached_destination(self):
        if self.ego_id not in traci.vehicle.getIDList():
            return False
        return (traci.vehicle.getRoadID(self.ego_id) == self.dest_edge
                and traci.vehicle.getLanePosition(self.ego_id) >= self.dest_pos - ARRIVE_TOL)

    def _info_extra(self):
        return {"level": self.level, "origin": self.origin, "destination": self.destination,
                "route_length": self.route_length, "remaining": self._remaining,
                "destination_changes": self.destination_changes, "red_lights": self.red_lights}

    def step(self, action):
        alive = self.ego_id in traci.vehicle.getIDList()
        tls_before = traci.vehicle.getNextTLS(self.ego_id) if alive else ()
        reach = (traci.vehicle.getSpeed(self.ego_id) * self.step_length + 1.0) if alive else 0.0
        obs, reward, terminated, truncated, info = super().step(action)

        # 빨간불 통과: 직전에 정지선이 이번 스텝 이동 거리 안에 있었고, 그 신호가 앞에서
        # 사라졌으면 넘은 것. (거리 조건이 없으면 교차로 앞 차로 변경으로 신호 번호만 바뀐 것을
        #  통과로 잘못 센다)
        info["red_light"] = False
        if tls_before and self.ego_id in traci.vehicle.getIDList():
            now = traci.vehicle.getNextTLS(self.ego_id)
            passed = (tls_before[0][2] <= reach
                      and (not now or now[0][:2] != tls_before[0][:2]))
            if passed and tls_before[0][3] in RED:
                self.red_lights += 1
                info["red_light"] = True
                reward -= float(self.rw.get("red_light_penalty", 0.0))
        # collision.action=remove 로 지워진 차도 SUMO 도착 목록에 들어가므로 충돌이면 도착이 아니다.
        info["arrived"] = info["arrived"] and not info["collided"]
        # 목적지 지점에 닿으면 SUMO의 edge 끝 도착을 기다리지 않고 바로 완주 처리
        if not terminated and self._reached_destination():
            traci.vehicle.remove(self.ego_id)
            reward += float(self.rw["arrival_bonus"])
            terminated = True
            info["arrived"] = True
            obs = np.zeros(self.obs_dim, dtype=np.float32)
        info.update(self._info_extra())
        return obs, float(reward), terminated, truncated, info

    # ==================================================================
    # 관측 (31 + 9) / 보상
    # ==================================================================
    def _get_obs(self):
        base = super()._get_obs()
        if self.ego_id not in traci.vehicle.getIDList():
            return np.zeros(self.obs_dim, dtype=np.float32)
        return np.concatenate([base, self._route_features()]).astype(np.float32)

    def _route_features(self):
        e = self.ego_id
        # [31][32] 앞 신호
        signal, signal_dist = 1.0, 1.0
        tls = traci.vehicle.getNextTLS(e)
        if tls and tls[0][2] < self.signal_range:
            state = tls[0][3]
            signal = -1.0 if state in RED else 0.0 if state in "yY" else 1.0
            signal_dist = tls[0][2] / self.signal_range
        self._signal = signal

        # [33] 다음 교차로까지, [34][35] 경로 차로
        lane_id = traci.vehicle.getLaneID(e)
        pos = traci.vehicle.getLanePosition(e)
        to_junction = 0.0 if lane_id.startswith(":") else traci.lane.getLength(lane_id) - pos
        offset, continues = 0, self.route_range
        for best in traci.vehicle.getBestLanes(e):
            if best[0] == lane_id:
                offset, continues = best[3], best[1] - pos
                break

        # [36] 남은 경로, [37] 제한속도
        self._remaining = self.remaining_distance()
        left = 1.0 if self._remaining is None else self._remaining / max(self.route_length, 1.0)
        self._limit = traci.vehicle.getAllowedSpeed(e)

        # [38][39] 다음 교차로에서 경로가 겹치는 차 (신호로 분리된 이동은 SUMO가 빼고 준다)
        foe_dist, yield_to = 1.0, 0.0
        foes = traci.vehicle.getJunctionFoes(e, self.junction_range)
        if foes:
            foe = min(foes, key=lambda f: f[2])            # 충돌 지점에 가장 가까운 상대
            foe_dist = max(foe[2], 0.0) / self.junction_range
            yield_to = 1.0 if foe[7] else -1.0 if foe[8] else 0.0

        return np.clip([signal, signal_dist,
                        to_junction / self.route_range,
                        offset / 2.0,
                        continues / self.route_range,
                        left,
                        self._limit / self.vmax,
                        foe_dist, yield_to], -1.0, 1.0)

    def _driving_reward(self, ego_speed, leader):
        """진행 거리 + 시간 벌점 + (정체·신호 대기가 아닐 때만) 차간거리·막힘 벌점."""
        rw = self.rw
        reward = float(rw["speed_weight"]) * ego_speed / self.vmax
        reward -= float(rw.get("time_penalty", 0.0))
        if self._remaining is not None and self._prev_remaining is not None:
            progress = np.clip(self._prev_remaining - self._remaining, -MAX_PROGRESS, MAX_PROGRESS)
            reward += float(rw.get("progress_weight", 0.0)) * progress
        self._prev_remaining = self._remaining

        if leader is None:
            return reward
        gap, lead_speed = leader
        queued = lead_speed < float(rw.get("queue_speed", 1.0)) or self._signal < 0
        if (gap < float(rw["close_gap_threshold"])
                and ego_speed > float(rw.get("moving_speed", 3.0))):
            reward -= float(rw["close_gap_penalty"])
        if (not queued and gap < float(rw.get("blocked_gap", 20.0))
                and ego_speed < float(rw.get("blocked_speed_frac", 0.7)) * self._limit):
            reward -= float(rw.get("blocked_penalty", 0.0))
        return reward

    # ==================================================================
    # 데이터 수집용 상태 — 전체 차량 대신 ego 반경 CONTEXT_RADIUS 안의 차량만
    # ==================================================================
    def get_privileged_state(self):
        if self.ego_id not in traci.vehicle.getIDList():
            return {"ego_alive": False}
        near = traci.vehicle.getContextSubscriptionResults(self.ego_id) or {}
        vehicles = []
        for vid, r in near.items():
            x, y = r[tc.VAR_POSITION]
            vehicles.append({"id": vid, "x": float(x), "y": float(y),
                             "speed": float(r[tc.VAR_SPEED]),
                             "lane": int(r[tc.VAR_LANE_INDEX]),
                             "lane_id": r[tc.VAR_LANE_ID],
                             "road_id": r[tc.VAR_ROAD_ID],
                             "lane_pos": float(r[tc.VAR_LANEPOSITION])})
        x, y = traci.vehicle.getPosition(self.ego_id)
        return {"ego_alive": True,
                "ego_x": float(x), "ego_y": float(y),
                "ego_speed": float(traci.vehicle.getSpeed(self.ego_id)),
                "ego_lane": int(traci.vehicle.getLaneIndex(self.ego_id)),
                "ego_lane_id": traci.vehicle.getLaneID(self.ego_id),
                "ego_road_id": traci.vehicle.getRoadID(self.ego_id),
                "ego_lane_pos": float(traci.vehicle.getLanePosition(self.ego_id)),
                "vehicles": vehicles,
                **self._info_extra()}


def make_env(**kwargs):
    """ROAD["type"]에 맞는 환경을 만든다. 인자는 SumoHighwayEnv와 같다
    (train.py / test.py / collect_pomdp_data.py 공용)."""
    cls = YeouidoEnv if kwargs["road"].get("type") == "yeouido" else SumoHighwayEnv
    return cls(**kwargs)
