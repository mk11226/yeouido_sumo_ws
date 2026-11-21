"""보상 점검: 보상 설계가 바라는 순서대로 총 보상이 나오는지 실제 주행으로 확인한다 (학습 전 점검).

    python -m inference.eval_set.reward_audit                                   # 무작위 시험지에서 시간대별 4문제씩
    python -m inference.eval_set.reward_audit --per-level 2 --workers 8

같은 문제를 정책 네 가지로 주행해 총 보상을 비교한다. 바라는 것:
    선생님이 문제마다 가장 높다 (쉴드에 기대 계속 밟는 throttle 보다도)
    서 있기 > 무작위 (충돌 ≪ 서 있기 < 조금씩 전진 < 도착)
  expert    SUMO 운전자 모델 (algorithms/sumo_expert.py)
  random    가감속 무작위, 차로 변경 가끔 (collect_pomdp_data.py 의 random 과 같은 분포)
  stand     계속 최대 감속 (출발 지점에서 서 있기)
  throttle  계속 최대 가속 (앞차와 거리를 무시 → 대개 추돌)
보상은 train.py 의 REWARD (env/mdp_config.py "여의도 전용") 그대로다.
결과: results/reward_audit/<시험지 이름>.json
"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))))  # 저장소 최상위
import argparse
import json
import multiprocessing
import os
import time
from collections import defaultdict

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # 저장소 최상위
POLICIES = ("expert", "random", "stand", "throttle")
LEVELS = ("night", "day", "rush")


class FixedPolicy:
    def __init__(self, accel):
        self.action = np.array([accel, 0.0], dtype=np.float32)

    def predict(self, obs, deterministic=True):
        return self.action


class RandomPolicy:
    def __init__(self, seed):
        self.rng = np.random.default_rng(seed)

    def predict(self, obs, deterministic=True):
        lane = self.rng.choice([-1.0, 0.0, 1.0], p=[0.15, 0.70, 0.15])
        return np.array([self.rng.uniform(-1.0, 1.0), lane], dtype=np.float32)


def _job(args):
    policy_name, problem = args
    import torch
    torch.set_num_threads(1)
    os.chdir(BASE)
    from inference.test import build_env
    from utils.evaluator import evaluate_problems
    env = build_env(gui=False)
    try:
        if policy_name == "expert":
            from training.algorithms.sumo_expert import SumoExpert
            policy = SumoExpert(env)
        elif policy_name == "random":
            policy = RandomPolicy(problem["sumo_seed"])
        else:
            policy = FixedPolicy(-1.0 if policy_name == "stand" else 1.0)
        rec = evaluate_problems(policy, env, [problem], deterministic=True)[0]
    finally:
        env.close()
    rec["policy"] = policy_name
    print(f"  {policy_name:8s} #{problem['id']:<3d} {problem['level']:5s} {rec['ended_by']:9s} "
          f"보상 {rec['return']:7.2f}", flush=True)
    return rec


def pick(problems, per_level):
    out = []
    for lv in LEVELS:
        out += [p for p in problems if p["level"] == lv][:per_level]
    return out


def report(records, problems):
    by = defaultdict(list)
    for r in records:
        by[(r["policy"], r["level"])].append(r)
    print("\n정책별 평균 총 보상 (끝난 방식: 도착/충돌/시간초과)")
    print(f"{'정책':10s}" + "".join(f"{lv:>24s}" for lv in LEVELS) + f"{'전체':>12s}")
    for pol in POLICIES:
        cells, alls = [], []
        for lv in LEVELS:
            rs = by[(pol, lv)]
            alls += rs
            if not rs:
                cells.append(f"{'-':>24s}")
                continue
            ends = [sum(r["ended_by"] == k for r in rs) for k in ("arrival", "collision", "timeout")]
            cells.append(f"{np.mean([r['return'] for r in rs]):8.2f} ({ends[0]}/{ends[1]}/{ends[2]})".rjust(24))
        print(f"{pol:10s}" + "".join(cells) + f"{np.mean([r['return'] for r in alls]):12.2f}")
    # 문제마다 순서가 지켜졌는지
    ret = {(r["policy"], r["id"]): r["return"] for r in records}
    best = sum(all(ret.get(("expert", p["id"]), -1e9) >= ret.get((pol, p["id"]), -1e9)
                   for pol in POLICIES) for p in problems)
    stand_gt_random = sum(ret.get(("stand", p["id"]), -1e9) > ret.get(("random", p["id"]), 1e9)
                          for p in problems)
    print(f"\n선생님이 가장 높은 문제: {best} / {len(problems)}")
    print(f"서 있기 > 무작위 인 문제: {stand_gt_random} / {len(problems)}")
    print("\n정책별 평균: 쉴드 개입 스텝 / 저크 합")
    for pol in POLICIES:
        rs = [r for r in records if r["policy"] == pol and r.get("shield_steps") is not None]
        if rs:
            print(f"  {pol:10s} 쉴드 {np.mean([r['shield_steps'] for r in rs]):7.0f}   "
                  f"저크 {np.mean([r['jerk_sum'] for r in rs]):7.1f}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-set", default="inference/eval_set/random_60_s1.json")
    ap.add_argument("--per-level", type=int, default=4, help="시간대마다 문제 수")
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args(argv)
    os.chdir(BASE)
    with open(args.eval_set, encoding="utf-8") as f:
        problems = pick(json.load(f), args.per_level)
    jobs = [(pol, p) for p in problems for pol in POLICIES]
    print(f"보상 점검: {len(problems)}문제 × 정책 {len(POLICIES)}개 = {len(jobs)}회 주행 (workers {args.workers})")
    t0 = time.time()
    with multiprocessing.get_context("spawn").Pool(args.workers) as pool:
        records = pool.map(_job, jobs, chunksize=1)
    from training.train import REWARD
    out = os.path.join(BASE, "results", "reward_audit",
                       os.path.splitext(os.path.basename(args.eval_set))[0] + ".json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"eval_set": args.eval_set, "seconds": round(time.time() - t0),
                   "reward": REWARD, "records": records}, f, ensure_ascii=False, indent=1)
    report(records, problems)
    print(f"\n{round(time.time() - t0)}초, 결과: {out}")


if __name__ == "__main__":
    main()
