"""효과 확인: 막히면 경로를 다시 찾게(내비게이션 재탐색) 하면 선생님이 못 풀던 문제가 풀리는가.

    LIBSUMO_AS_TRACI=quiet python -m inference.eval_set.reroute_check --workers 8

환경 코드는 건드리지 않고 env.step 을 감싸서 재탐색만 덧붙인다 (효과가 확인되면 그때 환경 옵션으로 넣는다).
규칙: ego 속도가 --slow m/s 미만인 스텝이 --stuck 초 쌓이면, 지금 도로 상황(도로마다 현재 통행 시간)으로
      목적지까지 가장 빠른 경로를 다시 찾는다 (env.reroute_by_current_times — 환경 옵션과 같은 방법). 재탐색 뒤 --cooldown 초는 다시 안 한다.
      경로가 바뀌면 change_destination 과 같이 남은 거리 기준을 다시 잡는다 (진행 보상이 튀지 않게).
대상: 선생님 기준선(results/sumo_expert/random_60_s1_results.json)에서 실패한 문제 전부
      + 도착했던 문제 몇 개 (재탐색이 멀쩡하던 주행을 망치지 않는지 대조군).
결과: results/reroute_check/expert_reroute.json + 기준선과 문제별 비교표.
"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))))  # 저장소 최상위
import argparse
import json
import multiprocessing
import os
import time

import torch

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # 저장소 최상위
_W = {}


def _init(cfg):
    from inference import test
    from training.algorithms.sumo_expert import SumoExpert
    from multiprocessing import util
    torch.set_num_threads(1)
    env = test.build_env(gui=False)
    util.Finalize(env, env.close, exitpriority=10)
    _wrap(env, cfg)
    _W.update(env=env, agent=SumoExpert(env))


def _wrap(env, cfg):
    import traci
    orig_step, orig_reset = env.step, env.reset
    dt = env.step_length
    st = {}

    def reset(*a, **k):
        st.update(slow=0.0, cool=0.0, tries=0, changed=0, log=[])
        return orig_reset(*a, **k)

    def step(action):
        obs, reward, terminated, truncated, info = orig_step(action)
        if terminated or truncated or info.get("speed") is None:
            return obs, reward, terminated, truncated, info
        st["slow"] = st["slow"] + dt if info["speed"] < cfg["slow"] else 0.0
        st["cool"] = max(0.0, st["cool"] - dt)
        if st["slow"] >= cfg["stuck"] and st["cool"] <= 0.0:
            ego = env.ego_id
            before = traci.vehicle.getRoute(ego)[traci.vehicle.getRouteIndex(ego):]
            rem_before = env.remaining_distance()
            env.reroute_by_current_times(ego)                  # 환경 옵션과 같은 방법 (현재 통행 시간으로)
            after = traci.vehicle.getRoute(ego)[traci.vehicle.getRouteIndex(ego):]
            st["tries"] += 1
            st["cool"] = cfg["cooldown"]
            st["slow"] = 0.0
            if tuple(after) != tuple(before):
                st["changed"] += 1
                env._remaining = env._prev_remaining = env.remaining_distance()
                st["log"].append(dict(t=env.step_count, rem_before=rem_before, rem_after=env._remaining))
        info["reroute_tries"], info["reroute_changed"] = st["tries"], st["changed"]
        return obs, reward, terminated, truncated, info

    env.step, env.reset = step, reset
    env._reroute_state = st


def _one(problem):
    from utils.evaluator import evaluate_problems
    env = _W["env"]
    rec = evaluate_problems(_W["agent"], env, [problem], deterministic=True)[0]
    st = env._reroute_state
    rec.update(reroute_tries=st.get("tries", 0), reroute_changed=st.get("changed", 0),
               reroute_log=st.get("log", []))
    print(f"  #{rec['id']:<3d} {rec['level']:5s} {rec['ended_by']:9s} 재탐색 {rec['reroute_tries']}회"
          f" (경로 바뀜 {rec['reroute_changed']}) {rec['origin']} → {rec['destination']}", flush=True)
    return rec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-set", default="inference/eval_set/random_60_s1.json")
    ap.add_argument("--baseline", default="results/sumo_expert/random_60_s1_results.json")
    ap.add_argument("--controls", type=int, default=6, help="대조군: 기준선에서 도착한 문제 몇 개")
    ap.add_argument("--ids", default=None, help="이 문제들만 (쉼표)")
    ap.add_argument("--slow", type=float, default=2.0, help="이 속도(m/s) 미만이면 '거의 못 감'")
    ap.add_argument("--stuck", type=float, default=60.0, help="거의 못 간 시간이 이만큼(초) 쌓이면 재탐색")
    ap.add_argument("--cooldown", type=float, default=60.0, help="재탐색 뒤 이 시간(초)은 다시 안 함")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--out", default="results/reroute_check/expert_reroute.json")
    args = ap.parse_args()

    with open(os.path.join(BASE, args.eval_set), encoding="utf-8") as f:
        problems = {p["id"]: p for p in json.load(f)}
    with open(os.path.join(BASE, args.baseline), encoding="utf-8") as f:
        base = {r["id"]: r for r in json.load(f)["records"]}
    if args.ids:
        ids = [int(i) for i in args.ids.split(",")]
    else:
        fails = [i for i, r in base.items() if r["ended_by"] not in ("arrival", "start_failed")]
        arr = [i for i, r in base.items() if r["ended_by"] == "arrival"]
        # 대조군: 시간대별로 고르게 (출근 우선 — 재탐색이 가장 자주 일어나는 곳)
        ctrl = []
        for lv in ("rush", "day", "night"):
            ctrl += [i for i in arr if base[i]["level"] == lv][: max(1, args.controls // 3)]
        ids = fails + ctrl[: args.controls]
    jobs = [problems[i] for i in ids]
    cfg = dict(slow=args.slow, stuck=args.stuck, cooldown=args.cooldown)
    print(f"재탐색 효과 확인: {len(jobs)}문제 (기준선 실패 {len(jobs) - min(args.controls, len(jobs))} + 대조군), "
          f"규칙 속도<{args.slow}m/s {args.stuck:.0f}초 → 재탐색, 쉬는 시간 {args.cooldown:.0f}초", flush=True)

    t0 = time.time()
    pool = multiprocessing.get_context("spawn").Pool(min(args.workers, len(jobs)), initializer=_init, initargs=(cfg,))
    try:
        recs = list(pool.imap_unordered(_one, jobs, chunksize=1))
        pool.close()
    except BaseException:
        pool.terminate()
        raise
    finally:
        pool.join()
    recs.sort(key=lambda r: r["id"])

    out = os.path.join(BASE, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(dict(cfg=cfg, seconds=round(time.time() - t0), records=recs), f, ensure_ascii=False, indent=1)

    print(f"\n{'id':>3} {'시간대':5} {'기준선':10} {'재탐색':10} {'재탐색 횟수(바뀜)':>14}  출발 → 도착")
    fixed = broke = 0
    for r in recs:
        b = base[r["id"]]["ended_by"]
        mark = ""
        if b != "arrival" and r["ended_by"] == "arrival":
            fixed += 1
            mark = "  ← 풀림"
        elif b == "arrival" and r["ended_by"] != "arrival":
            broke += 1
            mark = "  ← 망가짐"
        print(f"{r['id']:>3} {r['level']:5} {b:10} {r['ended_by']:10} {r['reroute_tries']:>8}({r['reroute_changed']})"
              f"  {r['origin']} → {r['destination']}{mark}")
    n_fail = sum(base[r["id"]]["ended_by"] != "arrival" for r in recs)
    print(f"\n기준선 실패 {n_fail}문제 중 풀림 {fixed}, 대조군(기준선 도착) 망가짐 {broke}, "
          f"충돌 {sum(r['ended_by'] == 'collision' for r in recs)}, "
          f"빨간불 {sum(r.get('red_lights') or 0 for r in recs)} ({round(time.time() - t0) // 60}분)")
    print(f"→ {out}")


if __name__ == "__main__":
    main()
