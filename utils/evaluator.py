"""정책 평가 유틸 — 주행 지표 측정.

알고리즘과 분리해둔 이유: 평가는 "정책이 무엇으로 학습됐는지"와 무관하다.
`predict(obs, deterministic=True) -> action` 메서드만 있으면
PPO든 SAC든, 심지어 규칙 기반 컨트롤러든 똑같이 평가할 수 있다.

왜 별도 평가가 필요한가:
    학습 중 에피소드는 탐험 노이즈(σ)가 섞인 확률적 행동의 결과라
    "지금 정책의 실제 운전 실력"을 정확히 보여주지 못한다.
    그래서 주기적으로 노이즈 없는(deterministic) 정책으로 주행시켜
    충돌률 / 완주율 / 평균속도 / 차간거리를 따로 측정한다.
"""
from dataclasses import dataclass, asdict

import numpy as np


@dataclass
class DrivingMetrics:
    """평가 1회(n_episodes 평균)의 주행 지표."""
    collision_rate: float   # 충돌로 끝난 에피소드 비율 (0~1, 낮을수록 좋음)
    success_rate: float     # 도로 끝까지 완주한 비율 (0~1, 높을수록 좋음)
    timeout_rate: float     # 시간초과로 끝난 비율 (= 1 - 충돌 - 완주)
    mean_speed: float       # 전 스텝 평균 주행속도 (m/s)
    mean_gap: float         # 선행차 감지 스텝에서의 평균 차간거리 (m)
    min_gap: float          # 관측된 최소 차간거리 (m) — 위험 운전 지표
    ep_return: float        # 에피소드당 평균 누적 보상
    ep_length: float        # 에피소드당 평균 스텝 수
    lane_changes: float     # 에피소드당 평균 "실행된" 차선변경 횟수
                            # (추월 시나리오에서 정책이 실제로 차선을
                            #  활용하는지 보는 핵심 지표)

    def as_dict(self) -> dict:
        return asdict(self)

    def summary(self) -> str:
        """콘솔 한 줄 출력용 문자열."""
        return (f"충돌률={self.collision_rate:.0%}  "
                f"완주율={self.success_rate:.0%}  "
                f"평균속도={self.mean_speed:.1f}m/s  "
                f"평균차간={self.mean_gap:.1f}m  "
                f"최소차간={self.min_gap:.1f}m  "
                f"차선변경={self.lane_changes:.1f}회  "
                f"보상={self.ep_return:.1f}")


def run_episode(policy, env, deterministic: bool = True, options=None) -> dict:
    """정책으로 에피소드 1회를 주행하고 결과를 dict로 반환 (aggregate()가 합친다).

    options는 env.reset(options=...)에 그대로 넘긴다 (평가 시험지의 한 문제).
    """
    obs, _ = env.reset(options=options) if options is not None else env.reset()
    done, ep_ret, ep_len, ep_lc = False, 0.0, 0, 0
    ended_by = None
    speed_sum = speed_n = gap_sum = gap_n = 0
    hv_tot_sum = hv_near_sum = hv_n = 0                 # 주변 일반 차량 수 (info 에 있을 때만)
    min_gap = None
    info = {}

    while not done:
        action = policy.predict(obs, deterministic=deterministic)
        obs, reward, terminated, truncated, info = env.step(action)
        ep_ret += reward
        ep_len += 1
        done = terminated or truncated

        # 실행된 차선변경 집계 (유지 명령의 changeLane 핀 동작은 제외)
        if info.get("lane_change", 0) != 0 and info.get("lane_change_applied"):
            ep_lc += 1

        if info.get("collided"):
            ended_by = "collision"
        elif info.get("teleported"):
            ended_by = "teleport"             # 오래 못 움직여 SUMO 가 ego 를 순간이동시킴 (실패)
        elif info.get("arrived"):
            ended_by = "arrival"

        # 속도/차간거리는 env가 info로 넘겨주는 원시값(m/s, m)을 쓴다.
        # 관측(obs) 레이아웃이 바뀌어도 평가 코드는 영향받지 않는다.
        # 종료 스텝은 None (ego가 이미 제거됨) → 통계에서 제외.
        if info.get("speed") is not None:
            speed_sum += float(info["speed"])
            speed_n += 1
        if info.get("hv_total") is not None:
            hv_tot_sum += info["hv_total"]
            hv_near_sum += info["hv_near"] or 0
            hv_n += 1
        if info.get("gap") is not None:
            g = float(info["gap"])
            gap_sum += g
            gap_n += 1
            min_gap = g if min_gap is None else min(min_gap, g)

    return {
        "ended_by": ended_by or "timeout",
        "return": float(ep_ret),
        "length": ep_len,
        "lane_changes": ep_lc,
        "speed_sum": speed_sum, "speed_n": speed_n,
        "gap_sum": gap_sum, "gap_n": gap_n, "min_gap": min_gap,
        # 여의도 환경만 주는 값 (직선 도로에서는 None)
        "red_lights": info.get("red_lights"),
        "route_length": info.get("route_length"),
        "remaining": info.get("remaining"),       # 끝날 때 목적지까지 남은 거리 (시간 초과면 얼마나 갔는지)
        "shield_steps": info.get("shield_steps"),   # 안전 쉴드가 속도를 깎은 스텝 수
        "jerk_sum": info.get("jerk_sum"),           # |가속도 변화| 합 (m/s^2, 승차감)
        "junction_overlaps": info.get("junction_overlaps"),  # 교차로 안 차체 겹침 스텝 수 (진단용)
        "reroutes": info.get("reroutes"),           # 경로 재탐색으로 경로가 바뀐 횟수 (ROAD["reroute"])
        "hv_total_mean": round(hv_tot_sum / hv_n, 1) if hv_n else None,   # 도로망 전체 일반 차량 수 (주행 중 평균)
        "hv_near_mean": round(hv_near_sum / hv_n, 2) if hv_n else None,   # ego 반경 100 m 안 일반 차량 수 (평균)
    }


def aggregate(records, visibility: float) -> DrivingMetrics:
    """run_episode() 결과 여러 개를 주행 지표 평균으로 합친다."""
    n = len(records)
    speed_n = sum(r["speed_n"] for r in records)
    gap_n = sum(r["gap_n"] for r in records)
    gaps = [r["min_gap"] for r in records if r["min_gap"] is not None]
    return DrivingMetrics(
        collision_rate=sum(r["ended_by"] == "collision" for r in records) / n,
        success_rate=sum(r["ended_by"] == "arrival" for r in records) / n,
        timeout_rate=sum(r["ended_by"] == "timeout" for r in records) / n,
        mean_speed=sum(r["speed_sum"] for r in records) / speed_n if speed_n else 0.0,
        mean_gap=sum(r["gap_sum"] for r in records) / gap_n if gap_n else float(visibility),
        min_gap=min(gaps) if gaps else float(visibility),
        ep_return=float(np.mean([r["return"] for r in records])),
        ep_length=float(np.mean([r["length"] for r in records])),
        lane_changes=float(np.mean([r["lane_changes"] for r in records])),
    )


def evaluate_policy(policy, env, n_episodes: int = 5,
                    deterministic: bool = True) -> DrivingMetrics:
    """정책을 n_episodes 주행시키고 주행 지표 평균을 반환.

    Args:
        policy: `predict(obs, deterministic) -> action` 을 가진 객체
                (PPOAgent, 또는 같은 인터페이스의 무엇이든)
        env:    SumoHighwayEnv (mean_gap 기본값용 visibility 속성만 사용)
        n_episodes: 평가 에피소드 수. 많을수록 지표가 안정적이지만 느리다
        deterministic: True면 탐험 노이즈 없이 평균 행동만 사용

    Note:
        모든 지표는 env.step()의 info(collided/arrived/speed/gap)에서 읽는다.
        관측(obs) 벡터의 구성과 무관하므로, 관측을 바꿔도 이 코드는 안 깨진다.
    """
    records = [run_episode(policy, env, deterministic) for _ in range(n_episodes)]
    return aggregate(records, env.visibility)


# 시험지 한 문제 중 env.reset(options=...)에 넘기는 칸. 나머지(category 등)는 결과 분류용.
PROBLEM_KEYS = ("level", "origin", "destination", "snapshot_time", "sumo_seed")


def evaluate_problems(policy, env, problems, deterministic: bool = True, on_result=None) -> list:
    """고정 시험지(eval_set/eval_set.json)의 문제를 차례로 주행한다.

    문제마다 같은 조건(시간대·출발·도착·워밍업 시점·SUMO 시드)으로 리셋하므로
    어떤 정책이든 같은 문제를 풀고, 같은 정책을 다시 평가하면 결과가 같다.
    반환: 문제 칸 + run_episode() 결과를 합친 dict 목록. on_result(record)는 문제마다 호출.
    """
    records = []
    for p in problems:
        try:
            result = run_episode(policy, env, deterministic,
                                 options={k: p[k] for k in PROBLEM_KEYS if k in p})
        except RuntimeError as e:
            # 출발 지점이 계속 막혀 ego를 넣지 못한 문제: 전체 평가를 멈추지 않고 "출발 실패"로 남긴다
            result = {"ended_by": "start_failed", "error": str(e), "return": 0.0, "length": 0,
                      "lane_changes": 0, "speed_sum": 0, "speed_n": 0, "gap_sum": 0, "gap_n": 0,
                      "min_gap": None, "red_lights": None, "route_length": None, "remaining": None,
                      "shield_steps": None, "jerk_sum": None, "junction_overlaps": None,
                      "reroutes": None, "hv_total_mean": None, "hv_near_mean": None}
        rec = {**p, **result}
        records.append(rec)
        if on_result:
            on_result(rec)
    return records
