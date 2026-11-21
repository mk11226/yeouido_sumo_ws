"""SUMO 기본 운전자 모델을 "선생님"으로 쓰는 규칙 정책 (BC 데이터 수집용).

ego는 speedMode=0이라 SUMO가 직접 운전하지 않는다. 대신 매 스텝 SUMO의 운전자 모델에게
"지금 이 상황이면 다음 스텝 속도를 얼마로 하겠는가"를 물어보고, 그 속도가 되도록 하는
가감속을 행동 [-1, 1]로 바꿔 낸다. 그래서 수집 데이터는 env.step()의 행동 형식 그대로다.

목표 속도 = 아래 중 가장 낮은 값
  - 자유 주행 : 현재 + 편안한 가속(accel) 한 스텝, 제한속도 이하
  - 차간 유지 : 앞차(경로를 따라 lookahead m까지)에 대한 SUMO 차량추종 속도 (getFollowSpeed)
  - 신호     : 빨간불(또는 멈출 수 있는 노란불)이면 정지선 앞에서 멈추는 속도 (env._signal_cap)
  - 양보     : 다음 교차로에서 내가 양보해야 할 차가 있으면 충돌 지점 앞에서 멈추는 속도
차로 변경 : SUMO 차로변경 모델이 "속도 이득"으로 바꾸고 싶어 하고 지금 바꿀 수 있을 때만 ±1.
            (경로상 꼭 필요한 변경은 lane_change_mode=1에서 SUMO가 직접 한다)

    expert = SumoExpert(env)
    action = expert.predict(obs)      # obs는 쓰지 않고 SUMO 상태를 직접 본다
"""
import numpy as np
import traci
import traci.constants as tc


_CACHE = {}   # (env, 에피소드, ego, 시뮬레이션 시각) → 가속 한계를 뺀 목표 속도 (같은 시각 두 번 계산 방지)


class SumoExpert:

    def __init__(self, env, lookahead=100.0, accel=2.6, leader_decel=4.5,
                 yield_margin=3.0):
        """
        env          : SumoHighwayEnv / YeouidoEnv (ego id, step 길이, 가감속 한계를 가져온다)
        lookahead    : 앞차를 찾는 거리 (m). 정책 관측(50 m)보다 길게 봐서 미리 감속한다
        accel        : 자유 주행 가속도 (m/s^2). 배경 차량 Krauss의 accel과 같게
        leader_decel : 앞차가 낼 수 있다고 가정하는 최대 감속도 (m/s^2)
        yield_margin : 양보할 때 충돌 지점 앞에 남길 거리 (m)
        """
        self.env = env
        self.ego = env.ego_id
        self.dt = env.step_length
        self.lookahead = lookahead
        self.accel = accel
        self.leader_decel = leader_decel
        self.yield_margin = yield_margin

    def target_speed(self, accel_limited=True):
        """다음 스텝 목표 속도. accel_limited=False 면 가속 한계를 빼고 "지금 상황에서 낼 수 있는 속도"
        (제한속도·앞차·신호·양보만 반영) — env/yeouido_env.py 의 불필요한 정지 벌점 기준으로 쓴다.

        모든 항목이 최솟값(min)이라 가속 한계는 마지막에 걸어도 같다. 그래서 가속 한계를 뺀 값을
        시뮬레이션 시각마다 한 번만 계산해 두고, 선생님 행동(가속 한계 있음)과 env 의 안전 쉴드·정지 벌점
        기준(없음)이 같이 쓴다 (예전엔 같은 계산을 스텝마다 두 번 했다).
        """
        e = self.ego
        v = traci.vehicle.getSpeed(e)
        key = (id(self.env), getattr(self.env, "episode", 0), e, traci.simulation.getTime())
        free = _CACHE.get(key)
        if free is None:
            free = self._free_target(v)
            _CACHE.clear()
            _CACHE[key] = free
        if accel_limited:
            return max(min(free, v + self.accel * self.dt), 0.0)
        return free

    def _free_target(self, v):
        """가속 한계를 뺀 목표 속도 (제한속도·앞차·신호·교차로 충돌 지점·양보)."""
        e, env = self.ego, self.env
        target = min(traci.vehicle.getAllowedSpeed(e), env.vmax)

        lead = traci.vehicle.getLeader(e, self.lookahead)
        if lead is not None and lead[0]:
            lid, gap = lead
            target = min(target, traci.vehicle.getFollowSpeed(
                e, v, max(gap, 0.0), traci.vehicle.getSpeed(lid), self.leader_decel, lid))

        if hasattr(env, "_signal_cap"):
            target = min(target, env._signal_cap(v))

        jr = getattr(env, "junction_range", 50.0)
        # 충돌 지점을 차지했거나 먼저 지나갈 차 (우선권과 무관, 교차로 안에서도 계속 본다)
        for ego_dist in self._blocked_conflicts(jr, v):
            target = min(target, traci.vehicle.getStopSpeed(e, v, max(ego_dist - self.yield_margin, 0.0)))
        lane = traci.vehicle.getLaneID(e)
        if lane.startswith(":"):
            return max(target, 0.0)                 # 교차로 안에서는 양보하지 않는다 (_yield_points 와 같음)
        stop_line = traci.lane.getLength(lane) - traci.vehicle.getLanePosition(e)
        # 양보 계산(주변 차마다 getJunctionFoes)이 가장 비싸다. 정지선이 지금 속도의 제동 거리보다
        # 충분히 멀면 양보해도 지금 속도를 낮출 일이 없으므로 건너뛴다.
        brake = v * self.dt + v * v / (2.0 * self.leader_decel) + self.yield_margin + 10.0
        points = self._yield_points(jr) if stop_line <= max(jr, brake) else []
        if points:
            # 양보할 때는 교차로에 들어가기 전 정지선에서 선다 (충돌 지점은 교차로 안 깊숙이 있을 수 있다)
            for ego_dist in points:
                dist = min(ego_dist - self.yield_margin, stop_line - 0.5)
                target = min(target, traci.vehicle.getStopSpeed(e, v, max(dist, 0.0)))
        return max(target, 0.0)

    def _path_ahead(self, length):
        """ego가 앞으로 지나갈 경로 모양(좌표 목록)과 각 점까지의 거리. 지금 차로 남은 부분 + 다음 교차로 내부 차로."""
        e = self.ego
        lane = traci.vehicle.getLaneID(e)
        pos = traci.vehicle.getLanePosition(e)
        lanes = [lane]
        if not lane.startswith(":") and traci.lane.getLength(lane) - pos > length:
            return None                                    # 교차로가 멀다
        links = traci.vehicle.getNextLinks(e)
        if links:
            via = next((x for x in links[0][1:5] if isinstance(x, str) and x.startswith(":")), None)
            if via and via != lane:
                lanes.append(via)
            lanes.append(links[0][0])
        pts, dist, travelled = [], [], -pos
        for ln in lanes:
            shape = np.asarray(traci.lane.getShape(ln), dtype=float)
            seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(shape, axis=0), axis=1))]
            for s0 in np.arange(0.0, seg[-1], 1.0):          # 1 m 간격으로 다시 찍는다
                d = travelled + s0
                if d < 0:
                    continue
                if d > length:
                    break
                i = min(np.searchsorted(seg, s0, side="right") - 1, len(shape) - 2)
                t = (s0 - seg[i]) / max(seg[i + 1] - seg[i], 1e-6)
                pts.append(shape[i] + t * (shape[i + 1] - shape[i]))
                dist.append(d)
            travelled += seg[-1]
            if travelled > length:
                break
        return (np.asarray(pts), np.asarray(dist)) if pts else None

    def _stopped_on_path(self, v, slow=2.0, clearance=2.2):
        """(쓰지 않음 — 진단용으로 남김) 교차로 근처에서, 경로 모양 위에 차체가 걸친 느린 차(slow m/s 미만)까지의 거리. 없으면 None.

        2026-11-20: 여러 교차점을 묶은 대형 교차로(내부 78 m 곡선)에서 교차로 모델로는 충돌 지점을
        이미 지났는데, 양보 대기선까지 다가와 선 상대 차의 실제 차체가 ego 경로에 걸쳐 충돌했다.
        그래서 교차로 모델과 별개로 실제 모양으로 본다. 움직이는 차는 _blocked_conflicts 가 본다.
        → 그런데 같은 모양 오차 때문에 다른 방향 정지선에 선 차가 경로에 걸쳐 보여 교차로 안에서 수백 스텝
          기다렸다 (#509 시간초과). 교차로 안 충돌 검사를 끄기로 하면서(env/road_config.py) 빼냈다.
        clearance: 차 폭(1.8 m) + 여유. 경로 중심선과 차체 중심선이 이보다 가까우면 걸친 것.
        """
        e = self.ego
        look = v * self.dt + v * v / (2.0 * self.leader_decel) + 15.0
        path = self._path_ahead(max(look, 25.0))
        if path is None:
            return None
        pts, dist = path
        best = None
        self.last_block = None                              # 진단용: (막은 차, 거리, 그 차의 차로, 속도, 경로와 최소 간격)
        for vid, r in (traci.vehicle.getContextSubscriptionResults(e) or {}).items():
            if vid == e or r[tc.VAR_SPEED] >= slow:
                continue
            x, y = r[tc.VAR_POSITION]
            if np.min(np.hypot(pts[:, 0] - x, pts[:, 1] - y)) > 8.0:
                continue                                    # 멀리 있는 차는 차체 계산 생략
            ang = np.radians(traci.vehicle.getAngle(vid))
            back = np.array([-np.sin(ang), -np.cos(ang)])   # SUMO 각도: 북쪽 0°, 시계 방향 → 뒤쪽 방향
            body = np.array([x, y]) + np.outer(np.linspace(0.0, traci.vehicle.getLength(vid), 6), back)
            dmin = np.min(np.hypot(pts[:, None, 0] - body[None, :, 0], pts[:, None, 1] - body[None, :, 1]), axis=1)
            hit = np.flatnonzero(dmin < clearance)
            if hit.size:
                s = dist[hit[0]]
                if best is None or s < best:
                    best = s
                    self.last_block = (vid, round(float(s), 1), r[tc.VAR_LANE_ID], round(r[tc.VAR_SPEED], 2),
                                       round(float(dmin.min()), 2))
        return best

    def _blocked_conflicts(self, jr, v, clear_margin=1.0):
        """경로가 겹치는 차가 충돌 지점을 막고 있을 때, ego에서 그 충돌 지점까지의 거리들.

        양보 규칙(_yield_points)은 "누가 양보하나"만 본다. 그런데 2026-11-20 선생님 데이터 충돌은
          - 우선권이 ego인데 상대가 충돌 지점 위에 멈춰 서 있음 (대형 교차로, 노란불 진입)
          - 서로 양보 관계가 없는 차가 ego보다 먼저 충돌 지점을 지나가는 중 (같은 교차로 다른 출구)
        였다. 둘 다 앞차(getLeader)로도 안 잡힌다. 그래서 다음 상대는 장애물로 보고 그 앞에서 선다:
          - 이미 충돌 지점에 있거나 (상대 거리 ≤ 0.5 m, 아직 다 빠져나가지 않음)
          - 양보하지 않는 상대가 ego보다 충돌 지점에 가까움
        단, ego가 도착하기 clear_margin 초 전에 다 빠져나갈 상대는 무시한다 (불필요한 정지 방지).
        getJunctionFoes: (id, ego 거리, 상대 거리, ego 출구 거리, 상대 출구 거리, ego 차로, 상대 차로,
                          ego가 양보?, 상대가 양보?)
        """
        e = self.ego
        out = []
        for f in traci.vehicle.getJunctionFoes(e, jr):
            fid, ego_dist, foe_dist, _, foe_exit, _, _, _, foe_yields = f[:9]
            if ego_dist < 0:
                continue                                    # ego가 이미 충돌 지점에 들어감 (설 수 없음)
            try:
                flen, fv = traci.vehicle.getLength(fid), traci.vehicle.getSpeed(fid)
            except traci.TraCIException:
                continue
            left = foe_exit + flen                           # 상대 꼬리가 충돌 구역을 벗어나기까지 거리
            if left <= 0:
                continue
            occupying = foe_dist <= 0.5
            first = (not foe_yields) and foe_dist < ego_dist
            if not (occupying or first):
                continue
            if left / max(fv, 0.1) + clear_margin < ego_dist / max(v, 0.1):
                continue                                     # ego가 닿기 전에 빠져나감
            out.append(ego_dist)
        return out

    def _yield_points(self, jr):
        """ego가 양보해야 하는 상대가 있을 때, ego에서 충돌 지점까지의 거리들.

        ego(speedMode=0)에게 getJunctionFoes를 물으면 늘 빈 목록이 온다. 상대 차 쪽에서 물으면
        ego를 '양보해야 할 차'로 정확히 본다. 그래서 주변 차들 쪽 결과도 모은다.
        (2026-11-16: 1→5 주간 sumo_seed 7 에서 비보호(g) 좌회전 중 직진 차와 충돌하던 문제)
          - ego가 이미 교차로 안이면 양보하지 않는다 (안에서 멈추면 서로 기다리는 교착).
          - 빨간불에 서 있는 차, 충돌 지점을 이미 다 지나간 차는 뺀다. 교차로 안을 지나는 차는 양보 대상.
          - 양보할 때는 충돌 지점이 아니라 교차로 앞 정지선에서 선다 (target_speed).
        """
        e = self.ego
        if traci.vehicle.getLaneID(e).startswith(":"):
            return []
        out = []
        for f in traci.vehicle.getJunctionFoes(e, jr):
            # f = (id, ego 거리, 상대 거리, ego 출구 거리, 상대 출구 거리, …, ego가 양보?, 상대가 양보?)
            # 상대가 충돌 구역을 완전히 빠져나갔으면(출구 거리 + 차 길이 ≤ 0) 양보하지 않는다.
            # 2026-11-20: 교차로를 지나 정체로 선 차에게 초록불 내내 양보 → 300 s 뒤 ego 순간이동 (#515, BC #19)
            if not f[7]:
                continue
            try:
                if f[4] + traci.vehicle.getLength(f[0]) <= 0:
                    continue
            except traci.TraCIException:
                continue
            out.append(f[1])
        for vid in traci.vehicle.getContextSubscriptionResults(e) or {}:
            if vid == e:
                continue
            for f in traci.vehicle.getJunctionFoes(vid, jr):
                # 상대 입장: (foeId=ego, 상대 거리, ego 거리, …, 상대가 양보?, ego가 양보?)
                if f[0] != e or not f[8] or f[3] < 0:
                    continue                       # f[3] < 0: 상대가 이미 충돌 지점을 완전히 지나감
                if not traci.vehicle.getLaneID(vid).startswith(":"):
                    links = traci.vehicle.getNextLinks(vid)
                    if links and links[0][5] in "rRu":
                        continue
                out.append(f[2])
        return out

    def lane_change(self):
        for direction in (1, -1):                       # +1 왼쪽, -1 오른쪽
            state = traci.vehicle.getLaneChangeState(self.ego, direction)[0]
            if (state & tc.LCA_SPEEDGAIN and not state & tc.LCA_BLOCKED
                    and state & (tc.LCA_LEFT | tc.LCA_RIGHT)):
                return direction
        return 0

    def predict(self, obs=None, deterministic=True):
        if self.ego not in traci.vehicle.getIDList():
            return np.zeros(2, dtype=np.float32)
        v = traci.vehicle.getSpeed(self.ego)
        accel = (self.target_speed() - v) / self.dt     # m/s^2
        env = self.env
        raw = accel / env.max_accel if accel >= 0 else accel / env.max_decel
        return np.array([np.clip(raw, -1.0, 1.0), self.lane_change()], dtype=np.float32)
