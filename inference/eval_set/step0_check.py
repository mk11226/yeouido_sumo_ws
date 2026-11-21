"""0단계 점검: 안전 쉴드·저크를 실제 주행으로 확인한다 (학습 전).

    python -m inference.eval_set.step0_check --workers 8

  1) #7 (출근 당산역→게이트 C) 를 선생님(쉴드 끔/켬), 들이받기(쉴드 켬)로 주행 → 쉴드가 도착을 막는지
  2) 시간대별 2문제를 선생님(쉴드 켬)으로 주행 → 에피소드당 저크 합 (jerk_penalty 크기 정하는 기준)
각 주행의 걸린 시간(초)도 남긴다 (양보 계산 생략으로 빨라졌는지).
결과: results/step0_check.json
"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))))  # 저장소 최상위
import argparse
import json
import multiprocessing
import os
import time

import numpy as np

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # 저장소 최상위
LEVELS = ("night", "day", "rush")


def _job(args):
    policy_name, shield, problem = args
    import torch
    torch.set_num_threads(1)
    os.chdir(BASE)
    from inference.test import build_env
    from utils.evaluator import PROBLEM_KEYS
    env = build_env(gui=False)
    env.safety_shield = shield
    t0 = time.time()
    try:
        if policy_name == "expert":
            from training.algorithms.sumo_expert import SumoExpert
            policy = SumoExpert(env)
        else:
            from inference.eval_set.reward_audit import FixedPolicy
            policy = FixedPolicy(1.0)
        obs, info = env.reset(options={k: problem[k] for k in PROBLEM_KEYS if k in problem})
        done, ret, n, ended = False, 0.0, 0, "timeout"
        speeds, jerks, stuck = [], [], []
        while not done:
            obs, r, term, trunc, info = env.step(policy.predict(obs))
            ret += r
            n += 1
            done = term or trunc
            if info.get("collided"):
                ended = "collision"
            elif info.get("arrived"):
                ended = "arrival"
            if info.get("speed") is not None:
                speeds.append(info["speed"])
                jerks.append(info["jerk"])
            if n % 600 == 0:                 # 5분마다 어디 있는지
                stuck.append({"step": n, "remaining": info.get("remaining"),
                              "speed": info.get("speed")})
    finally:
        env.close()
    rec = dict(policy=policy_name, shield=shield, id=problem["id"], level=problem["level"],
               origin=problem["origin"], destination=problem["destination"],
               ended_by=ended, steps=n, minutes=round(n * 0.5 / 60, 1), ret=round(ret, 2),
               remaining=info.get("remaining"), route_length=info.get("route_length"),
               shield_steps=info.get("shield_steps"), jerk_sum=info.get("jerk_sum"),
               jerk_mean=float(np.mean(jerks)) if jerks else None,
               jerk_p95=float(np.percentile(jerks, 95)) if jerks else None,
               mean_speed=float(np.mean(speeds)) if speeds else None,
               trace=stuck, seconds=round(time.time() - t0))
    print(f"  {policy_name:8s} 쉴드{'O' if shield else 'X'} #{problem['id']:<3d} {problem['level']:5s} "
          f"{ended:9s} {rec['minutes']:5.1f}분 보상 {rec['ret']:7.2f} 저크합 {rec['jerk_sum']} "
          f"쉴드 {rec['shield_steps']} ({rec['seconds']}초)", flush=True)
    return rec


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-set", default="inference/eval_set/random_60_s1.json")
    ap.add_argument("--workers", type=int, default=8)
    args = ap.parse_args(argv)
    os.chdir(BASE)
    problems = json.load(open(args.eval_set, encoding="utf-8"))
    p7 = next(p for p in problems if p["id"] == 7)
    jobs = [("expert", False, p7), ("expert", True, p7), ("throttle", True, p7)]
    for lv in LEVELS:
        jobs += [("expert", True, p) for p in [p for p in problems if p["level"] == lv and p["id"] != 7][:2]]
    print(f"0단계 점검: {len(jobs)}회 주행 (workers {args.workers})")
    t0 = time.time()
    with multiprocessing.get_context("spawn").Pool(args.workers) as pool:
        records = pool.map(_job, jobs, chunksize=1)
    out = os.path.join(BASE, "results", "step0_check.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump({"seconds": round(time.time() - t0), "records": records},
              open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"\n{round(time.time() - t0)}초, 결과: {out}")


if __name__ == "__main__":
    main()
