"""재검증: 경로 재탐색을 켠 환경(YEOUIDO_REROUTE=1)이 안전하고 정상인지 선생님으로 확인한다.

    LIBSUMO_AS_TRACI=quiet YEOUIDO_REROUTE=1 python -m eval_set.reroute_verify --workers 8

1단계 선생님 검증과 같은 기준을 시험지 60문제(random_60_s1) 전체에 적용하고, 재탐색 전용 점검을 더한다:
  안전   충돌 0, 빨간불 0, 순간이동이 기준선보다 늘지 않음, 교차로 겹침(진단)
  성능   도착이 기준선(재탐색 없는 선생님)보다 줄지 않음, 문제별 풀림/망가짐
  재탐색 경로가 바뀐 직후 보상이 튀지 않음(|보상| ≤ 평소 최대), 관측에 NaN 없음,
         바뀐 뒤 남은 거리가 비정상적으로 늘지 않음(빙빙 도는 경로 방지: 2배 넘게 늘면 경고)
결과: results/reroute_check/verify_expert.json + 판정 (통과/실패 사유).
"""
import argparse
import json
import multiprocessing
import os
import time

import numpy as np
import torch

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_W = {}


def _init():
    import test
    from algorithms.sumo_expert import SumoExpert
    from multiprocessing import util
    torch.set_num_threads(1)
    env = test.build_env(gui=False)
    assert env.reroute, "YEOUIDO_REROUTE=1 로 실행하세요"
    util.Finalize(env, env.close, exitpriority=10)
    _W.update(env=env, agent=SumoExpert(env))


def _one(p):
    from utils.evaluator import PROBLEM_KEYS
    env, ag = _W["env"], _W["agent"]
    try:
        obs, _ = env.reset(options={k: p[k] for k in PROBLEM_KEYS if k in p})
    except RuntimeError as e:
        return dict(id=p["id"], level=p["level"], ended_by="start_failed", error=str(e))
    rews, events, nan_obs, n, info = [], [], 0, 0, {}
    ended = None
    while True:
        rem0 = env._remaining
        obs, r, te, tr, info = env.step(ag.predict(obs))
        n += 1
        rews.append(r)
        nan_obs += int(not np.isfinite(obs).all())
        if info.get("rerouted"):
            events.append(dict(step=n, rem_before=rem0, rem_after=info.get("remaining")))
        if info.get("collided"):
            ended = "collision"
        elif info.get("teleported"):
            ended = "teleport"
        elif info.get("arrived"):
            ended = "arrival"
        if te or tr:
            break
    rews = np.asarray(rews)
    after = [float(rews[e["step"]]) for e in events if e["step"] < len(rews)]   # 재탐색 다음 스텝 보상
    body = rews[:-1] if len(rews) > 1 else rews                                 # 마지막(도착 보너스 등) 제외
    rec = dict(id=p["id"], level=p["level"], origin=p["origin"], destination=p["destination"],
               ended_by=ended or "timeout", length=n, red_lights=info.get("red_lights"),
               junction_overlaps=info.get("junction_overlaps"), reroutes=info.get("reroutes"),
               nan_obs=nan_obs, max_abs_reward=float(np.abs(body).max()) if len(body) else 0.0,
               max_abs_after_reroute=float(np.max(np.abs(after))) if after else None, events=events)
    print(f"  #{rec['id']:<3d} {rec['level']:5s} {rec['ended_by']:9s} 경로 바뀜 {rec['reroutes']}회", flush=True)
    return rec


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval-set", default="eval_set/random_60_s1.json")
    ap.add_argument("--baseline", default="results/sumo_expert/random_60_s1_results.json")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--ids", default=None, help="이 문제들만 (쉼표, 동작 확인용)")
    ap.add_argument("--out", default="results/reroute_check/verify_expert.json")
    args = ap.parse_args()
    if os.environ.get("YEOUIDO_REROUTE") != "1":
        raise SystemExit("YEOUIDO_REROUTE=1 로 실행하세요")
    with open(os.path.join(BASE, args.eval_set), encoding="utf-8") as f:
        problems = json.load(f)
    if args.ids:
        keep = {int(i) for i in args.ids.split(",")}
        problems = [p for p in problems if p["id"] in keep]
    with open(os.path.join(BASE, args.baseline), encoding="utf-8") as f:
        base = {r["id"]: r for r in json.load(f)["records"]}

    t0 = time.time()
    pool = multiprocessing.get_context("spawn").Pool(min(args.workers, len(problems)), initializer=_init)
    try:
        recs = list(pool.imap_unordered(_one, problems, chunksize=1))
        pool.close()
    except BaseException:
        pool.terminate()
        raise
    finally:
        pool.join()
    recs.sort(key=lambda r: r["id"])

    cnt = lambda rs, k: sum(r["ended_by"] == k for r in rs)
    b_recs = [base[r["id"]] for r in recs]
    ok = [r for r in recs if r["ended_by"] != "start_failed"]
    fixed = [r["id"] for r in recs if base[r["id"]]["ended_by"] != "arrival" and r["ended_by"] == "arrival"]
    broke = [r["id"] for r in recs if base[r["id"]]["ended_by"] == "arrival" and r["ended_by"] != "arrival"]
    reward_cap = max(r["max_abs_reward"] for r in ok)            # 평소(전체 주행) 보상 최대
    after = [r["max_abs_after_reroute"] for r in ok if r["max_abs_after_reroute"] is not None]
    events = [e for r in ok for e in r["events"]]
    loops = [e for e in events if e["rem_before"] and e["rem_after"] and e["rem_after"] > 2 * e["rem_before"]]

    checks = {
        "충돌 0": cnt(recs, "collision") == 0,
        "빨간불 0": sum(r.get("red_lights") or 0 for r in ok) == 0,
        "순간이동 기준선 이하": cnt(recs, "teleport") <= cnt(b_recs, "teleport"),
        "도착 기준선 이상": cnt(recs, "arrival") >= cnt(b_recs, "arrival"),
        "관측 NaN 없음": sum(r["nan_obs"] for r in ok) == 0,
        "재탐색 직후 보상 정상": (max(after) <= reward_cap + 1e-6) if after else True,
    }
    by_lv = lambda rs, lv: f"{sum(r['ended_by'] == 'arrival' for r in rs if r['level'] == lv)}/" \
                           f"{sum(r['level'] == lv for r in rs)}"
    summary = dict(
        seconds=round(time.time() - t0), checks=checks, passed=all(checks.values()),
        arrival=cnt(recs, "arrival"), arrival_base=cnt(b_recs, "arrival"),
        by_level={lv: by_lv(recs, lv) for lv in ("night", "day", "rush")},
        by_level_base={lv: by_lv(b_recs, lv) for lv in ("night", "day", "rush")},
        collision=cnt(recs, "collision"), teleport=cnt(recs, "teleport"), teleport_base=cnt(b_recs, "teleport"),
        timeout=cnt(recs, "timeout"), fixed=fixed, broke=broke, reroute_events=len(events),
        long_detours=len(loops), max_abs_after_reroute=max(after) if after else None, reward_cap=reward_cap,
        junction_overlaps=sum(r.get("junction_overlaps") or 0 for r in ok),
        junction_overlaps_base=sum(r.get("junction_overlaps") or 0 for r in b_recs),
    )
    out = os.path.join(BASE, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(dict(summary=summary, records=recs), f, ensure_ascii=False, indent=1)

    print(f"\n재검증 (선생님 + 재탐색, {len(recs)}문제, {summary['seconds'] // 60}분)")
    print(f"  도착 {summary['arrival']} (기준선 {summary['arrival_base']})  시간대 {summary['by_level']} "
          f"(기준선 {summary['by_level_base']})")
    print(f"  충돌 {summary['collision']}  순간이동 {summary['teleport']} (기준선 {summary['teleport_base']})  "
          f"시간초과 {summary['timeout']}  풀림 {fixed}  망가짐 {broke}")
    print(f"  경로 바뀜 {len(events)}회, 남은 거리 2배 넘게 는 우회 {len(loops)}회, 교차로 겹침 "
          f"{summary['junction_overlaps']} (기준선 {summary['junction_overlaps_base']})")
    for k, v in checks.items():
        print(f"  [{'통과' if v else '실패'}] {k}")
    print(f"판정: {'통과' if summary['passed'] else '실패'} → {out}")


if __name__ == "__main__":
    main()
