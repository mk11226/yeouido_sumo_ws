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

    def target_speed(self):
        e, env = self.ego, self.env
        v = traci.vehicle.getSpeed(e)
        target = min(v + self.accel * self.dt, traci.vehicle.getAllowedSpeed(e), env.vmax)

        lead = traci.vehicle.getLeader(e, self.lookahead)
        if lead is not None and lead[0]:
            lid, gap = lead
            target = min(target, traci.vehicle.getFollowSpeed(
                e, v, max(gap, 0.0), traci.vehicle.getSpeed(lid), self.leader_decel, lid))

        if hasattr(env, "_signal_cap"):
            target = min(target, env._signal_cap(v))

        jr = getattr(env, "junction_range", 50.0)
        points = self._yield_points(jr)
        if points:
            # 양보할 때는 교차로에 들어가기 전 정지선에서 선다 (충돌 지점은 교차로 안 깊숙이 있을 수 있다)
            lane = traci.vehicle.getLaneID(e)
            stop_line = traci.lane.getLength(lane) - traci.vehicle.getLanePosition(e)
            for ego_dist in points:
                dist = min(ego_dist - self.yield_margin, stop_line - 0.5)
                target = min(target, traci.vehicle.getStopSpeed(e, v, max(dist, 0.0)))
        return max(target, 0.0)

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
        out = [f[1] for f in traci.vehicle.getJunctionFoes(e, jr) if f[7]]
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
