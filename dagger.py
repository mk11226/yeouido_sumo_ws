"""3단계 DAgger: 학습한 BC 정책이 직접 운전하며 빠지는 상황에 선생님 정답을 붙여 보충 학습한다.

    LIBSUMO_AS_TRACI=quiet python dagger.py --init results/bc_hybrid_…/model.pt \\
        --base-data data/expert_….npz --iters 3 --episodes 100 --betas 0.5,0.25,0

반복 i (Ross et al., AISTATS 2011):
  1) 수집  collect_pomdp_data.py --policy sumo-expert --dagger-model <지금 모델> --beta β_i
           정책이 운전(스텝마다 확률 β_i 로 선생님이 대신 운전), 라벨은 매 순간 선생님 행동.
           안전 쉴드는 켜 둔다 → 정책이 서툴러도 충돌로 끝나지 않고 데이터가 모인다.
  2) 검증  관측 42차원 · NaN 없음 · 에피소드 수 (어긋나면 멈춘다)
  3) 학습  train_bc_hybrid.py --data <처음 선생님 데이터 + 지금까지 DAgger 데이터 전부> --init <지금 모델>
  4) 평가  test.py --eval-set (BC 와 같은 시험지) → 도착률·충돌률
  가장 좋은 모델 = 충돌률이 낮고, 같으면 도착률이 높은 것. --patience 번 연속 나아지지 않으면 일찍 멈춘다.
결과: results/<--out>/summary.json, 반복별 데이터 data/<--out>_iter<i>.npz, 모델 results/<--out>/iter<i>/model.pt
화면 없는 실행이라 LIBSUMO_AS_TRACI=quiet (libsumo) 로 돌리면 약 2배 빠르다 (결과는 같음).
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable


def run(cmd, log):
    print(f"$ {' '.join(cmd)}  (기록: {os.path.relpath(log, BASE)})", flush=True)
    with open(log, "w") as f:
        r = subprocess.run(cmd, cwd=BASE, stdout=f, stderr=subprocess.STDOUT, env=os.environ.copy())
    if r.returncode != 0:
        raise SystemExit(f"실패 (exit {r.returncode}): {log}")


def check_data(path, episodes, obs_dim):
    with np.load(path) as d:
        obs, act, ep = d["observation"], d["action_raw"], d["episode"]
        exe = d["action_executed"]
    problems = []
    if obs.ndim != 2 or obs.shape[1] != obs_dim:
        problems.append(f"관측 차원 {obs.shape} != {obs_dim}")
    for k, x in (("observation", obs), ("action_raw", act), ("action_executed", exe)):
        if not np.isfinite(x).all():
            problems.append(f"{k} 에 NaN/Inf")
    if (np.abs(act) > 1).any():
        problems.append("라벨이 [-1, 1] 밖")
    n_ep = len(np.unique(ep))
    if n_ep != episodes:
        problems.append(f"에피소드 {n_ep} != {episodes}")
    expert_share = float(np.mean(np.all(np.isclose(exe, act), axis=1)))
    return problems, dict(samples=int(len(obs)), episodes=n_ep, expert_executed_share=round(expert_share, 3))


def score(result_json):
    with open(result_json, encoding="utf-8") as f:
        recs = json.load(f)["records"]
    n = len(recs)
    out = dict(problems=n,
               arrival=sum(r["ended_by"] == "arrival" for r in recs) / n,
               collision=sum(r["ended_by"] == "collision" for r in recs) / n,
               timeout=sum(r["ended_by"] == "timeout" for r in recs) / n)
    for lv in ("night", "day", "rush"):
        rs = [r for r in recs if r["level"] == lv]
        if rs:
            out[f"arrival_{lv}"] = sum(r["ended_by"] == "arrival" for r in rs) / len(rs)
    out["collision_ids"] = [r["id"] for r in recs if r["ended_by"] == "collision"]
    return out


def better(a, b):
    """a 가 b 보다 나은가: 충돌률 낮은 쪽, 같으면 도착률 높은 쪽."""
    if b is None:
        return True
    if abs(a["collision"] - b["collision"]) > 1e-9:
        return a["collision"] < b["collision"]
    return a["arrival"] > b["arrival"] + 1e-9


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--init", required=True, help="출발 모델 (train_bc_hybrid.py 의 model.pt)")
    ap.add_argument("--init-result", default=None,
                    help="출발 모델의 평가 결과 json (없으면 처음에 평가한다)")
    ap.add_argument("--base-data", nargs="+", required=True, help="처음 선생님 데이터 .npz")
    ap.add_argument("--iters", type=int, default=3)
    ap.add_argument("--episodes", type=int, default=100, help="반복마다 수집할 에피소드 수")
    ap.add_argument("--betas", default="0.5,0.25,0", help="반복별 β (선생님이 대신 운전할 확률)")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--eval-set", default="eval_set/random_60_s1.json")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--lr", type=float, default=5e-4)
    ap.add_argument("--patience", type=int, default=2, help="연속으로 나아지지 않으면 멈출 반복 수")
    ap.add_argument("--out", default="dagger")
    ap.add_argument("--seed", type=int, default=3000)
    args = ap.parse_args()

    betas = [float(b) for b in args.betas.split(",")]
    betas += [betas[-1]] * max(0, args.iters - len(betas))
    out_dir = os.path.join(BASE, "results", args.out)
    os.makedirs(out_dir, exist_ok=True)
    set_name = os.path.splitext(os.path.basename(args.eval_set))[0]
    summary_path = os.path.join(out_dir, "summary.json")
    summary = dict(started=time.strftime("%Y-%m-%d %H:%M"), args=vars(args), iters=[])

    from train import HPARAMS  # noqa: F401  (환경 설정을 읽어 관측 차원 확인)
    from env.mdp_config import OBSERVATION  # noqa: F401
    obs_dim = None

    cur = os.path.abspath(args.init)
    if args.init_result:
        best_score = score(args.init_result)
    else:
        log = os.path.join(out_dir, "eval_init.log")
        run([PY, "test.py", cur, "--eval-set", args.eval_set, "--workers", str(args.workers), "--nogui"], log)
        best_score = score(os.path.join(os.path.dirname(cur), f"{set_name}_results.json"))
    best_model = cur
    summary["init"] = dict(model=cur, score=best_score)
    print(f"[DAgger] 출발 모델 도착 {best_score['arrival']:.0%} 충돌 {best_score['collision']:.1%}", flush=True)

    data = [os.path.abspath(p) for p in args.base_data]
    with np.load(data[0]) as d:
        obs_dim = d["observation"].shape[1]
    stale = 0
    for i in range(1, args.iters + 1):
        t0 = time.time()
        name = f"{args.out}_iter{i}"
        print(f"\n[DAgger] 반복 {i}/{args.iters}: β={betas[i - 1]}, 운전 모델 {os.path.relpath(cur, BASE)}", flush=True)
        run([PY, "collect_pomdp_data.py", "--policy", "sumo-expert", "--dagger-model", cur,
             "--beta", str(betas[i - 1]), "--episodes", str(args.episodes), "--workers", str(args.workers),
             "--seed", str(args.seed + 1000 * i), "--name", name],
            os.path.join(out_dir, f"collect_iter{i}.log"))
        npz = os.path.join(BASE, "data", name + ".npz")
        problems, info = check_data(npz, args.episodes, obs_dim)
        print(f"[DAgger] 데이터 {info}", flush=True)
        if problems:
            raise SystemExit(f"수집 데이터 검증 실패: {problems}")
        data.append(npz)

        model_dir = os.path.join(out_dir, f"iter{i}")
        run([PY, "train_bc_hybrid.py", "--data", *data, "--init", cur, "--epochs", str(args.epochs),
             "--lr", str(args.lr), "--out-dir", model_dir],
            os.path.join(out_dir, f"train_iter{i}.log"))
        model = os.path.join(model_dir, "model.pt")
        run([PY, "test.py", model, "--eval-set", args.eval_set, "--workers", str(args.workers), "--nogui"],
            os.path.join(out_dir, f"eval_iter{i}.log"))
        sc = score(os.path.join(model_dir, f"{set_name}_results.json"))
        improved = better(sc, best_score)
        summary["iters"].append(dict(iter=i, beta=betas[i - 1], data=info, model=model, score=sc,
                                     improved=improved, minutes=round((time.time() - t0) / 60, 1)))
        print(f"[DAgger] 반복 {i}: 도착 {sc['arrival']:.0%} (심야 {sc.get('arrival_night', 0):.0%} / 주간 "
              f"{sc.get('arrival_day', 0):.0%} / 출근 {sc.get('arrival_rush', 0):.0%}) 충돌 {sc['collision']:.1%} "
              f"{'← 최고' if improved else ''} ({summary['iters'][-1]['minutes']}분)", flush=True)
        if improved:
            best_score, best_model, stale = sc, model, 0
        else:
            stale += 1
        cur = model                               # 다음 반복은 방금 모델이 운전 (DAgger 원래 방식)
        summary.update(best_model=best_model, best_score=best_score)
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=1)
        if stale >= args.patience:
            print(f"[DAgger] {args.patience}번 연속 나아지지 않아 멈춤", flush=True)
            break
    print(f"\n[DAgger] 끝. 최고 모델 {os.path.relpath(best_model, BASE)}: 도착 {best_score['arrival']:.0%} "
          f"충돌 {best_score['collision']:.1%} → {summary_path}", flush=True)


if __name__ == "__main__":
    main()
