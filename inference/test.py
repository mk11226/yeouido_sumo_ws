"""학습된 모델을 sumo-gui로 재생하고 주행 지표를 출력한다.

사용법:
    python -m inference.test results/run_20260824_153012/model.pt   ← 모델 경로 지정
    python -m inference.test                                        ← 최근 run 자동 선택
    python -m inference.test --episodes 5 --nogui                   ← 화면 없이 지표만 측정
    python -m inference.test results/bc_example/model.pt --algorithm bc --nogui

고정 시험지 평가 (모든 모델을 같은 150문제로, 결과는 문제별로 저장):
    python -m inference.test results/bc_…/model.pt --eval-set inference/eval_set/eval_set.json --workers 4
    python -m inference.test --expert --eval-set inference/eval_set/eval_set.json --workers 4   ← 선생님 기준선
    python -m inference.test results/bc_…/model.pt --eval-set inference/eval_set/eval_set.json --limit 6  ← 빠른 점검
  결과: 모델 폴더(선생님은 results/sumo_expert/)의 eval_set_results.json + 종류별 표 출력.
  여러 결과 비교: python -m utils.eval_report <결과1.json> <결과2.json> ...

빨간 차가 ego. 앞차가 느리면 감속하고 앞이 비면 가속하는지 관찰해보자.
"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))  # 저장소 최상위 (env, utils 등)
import argparse
import glob
import json
import multiprocessing
import os
import time

import numpy as np
import torch

from env.road_config import ROAD, EGO, TRAFFIC
from env.mdp_config import SIMULATION, OBSERVATION, ACTION
from env import road_builder
from env.yeouido_env import make_env   # ROAD['type']에 맞는 환경 (직선/여의도)

from training.algorithms.ppo import PPO
from training.algorithms.bc import BCPolicy
from utils.evaluator import evaluate_policy, evaluate_problems
from utils import eval_report

# train.py와 동일한 네트워크 구조로 만들어야 가중치를 불러올 수 있으므로
# 하이퍼파라미터를 그대로 가져온다 (구조 관련 항목만 실제로 쓰임).
from training.train import HPARAMS, RESULTS_DIR, REWARD  # ← 보상도 train과 동일 조건

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # 저장소 최상위


def find_latest_model() -> str:
    """results/*/model.pt 중 가장 최근에 저장된 것을 찾는다."""
    candidates = glob.glob(os.path.join(BASE, RESULTS_DIR, "*", "model.pt"))
    if not candidates:
        raise FileNotFoundError(
            f"'{RESULTS_DIR}/' 안에 model.pt가 없습니다. "
            f"먼저 train.py 또는 train_bc.py로 학습하세요.")
    return max(candidates, key=os.path.getmtime)


def load_agent(model_path, obs_dim, act_dim, algorithm="auto"):
    """BC 체크포인트와 기존 PPO state_dict를 구분해 평가 정책을 복원한다."""
    checkpoint = torch.load(model_path, map_location="cpu", weights_only=True)
    if not isinstance(checkpoint, dict):
        raise ValueError("지원하지 않는 모델 파일 형식입니다.")
    if checkpoint.get("algorithm") == "bc":
        detected = "bc"
    elif "trunk.0.weight" in checkpoint and "mu_head.weight" in checkpoint:
        detected = "ppo"
    else:
        raise ValueError("현재 BC 체크포인트 또는 PPO state_dict가 아닙니다.")
    if algorithm != "auto" and algorithm != detected:
        raise ValueError(f"--algorithm {algorithm}과 모델 유형 {detected}가 다릅니다.")

    if detected == "bc":
        if not {"state_dim", "action_dim", "state_dict"}.issubset(checkpoint):
            raise ValueError("현재 BCPolicy 형식이 아닙니다. 현재 train_bc.py로 학습한 모델을 사용하세요.")
        dims = (checkpoint["state_dim"], checkpoint["action_dim"])
        if dims != (obs_dim, act_dim):
            raise ValueError(f"BC 모델 입출력 차원 {dims} != 환경 {(obs_dim, act_dim)}")
        agent = BCPolicy.load(model_path, device="cpu")
    else:
        agent = PPO(obs_dim=obs_dim, act_dim=act_dim, **HPARAMS)
        try:
            agent.policy.load_state_dict(checkpoint)
        except RuntimeError as exc:
            raise ValueError("PPO 모델과 현재 환경 차원 또는 HPARAMS의 네트워크 구조가 다릅니다.") from exc
        agent.policy.eval()
    print(f"평가 알고리즘: {detected.upper()}")
    return agent


def build_env(gui):
    # 주의: 현재 env/ 설정과 다른 설정으로 학습된 모델이면 성능이 이상할 수 있다.
    # 그 run의 설정 스냅샷은 model.pt와 같은 폴더의 *_config.py 에서 확인 가능.
    sumocfg = road_builder.build(ROAD, EGO, TRAFFIC,
                                 os.path.join(BASE, "env", "sumo"))
    return make_env(
        cfg_path=sumocfg,
        road=ROAD, ego=EGO,
        mdp_sim=SIMULATION, mdp_obs=OBSERVATION,
        action=ACTION, reward=REWARD,
        traffic=TRAFFIC,
        gui=gui,
        gui_autostart=False,   # ▶(플레이) 버튼을 눌러야 에피소드 시작
    )


def make_agent(model_path, env, algorithm, verbose=True):
    """model_path가 None이면 선생님 정책(SUMO 운전자 모델, algorithms/sumo_expert.py)."""
    if model_path is None:
        from training.algorithms.sumo_expert import SumoExpert
        if verbose:
            print("평가 알고리즘: SUMO-EXPERT (선생님)")
        return SumoExpert(env)
    if verbose:
        return load_agent(model_path, env.observation_space.shape[0],
                          env.action_space.shape[0], algorithm)
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        return load_agent(model_path, env.observation_space.shape[0],
                          env.action_space.shape[0], algorithm)


_W = {}   # 워커 프로세스마다 환경·정책 하나 (Pool initializer 가 만든다)


def _eval_init(model_path, algorithm, total):
    """--workers > 1: 워커마다 SUMO 하나를 켜 두고, 문제를 하나씩 받아 푼다.
    (미리 나눠 주면 막히는 출근 문제를 받은 워커만 늦게까지 돌고 나머지는 논다 → 끝난 워커가 남은 문제를 가져감)"""
    from multiprocessing import util
    torch.set_num_threads(1)
    env = build_env(gui=False)
    util.Finalize(env, env.close, exitpriority=10)   # pool.close/join 으로 정상 종료할 때 SUMO 정리
    _W.update(env=env, agent=make_agent(model_path, env, algorithm, verbose=False), total=total)


def _eval_one(problem):
    # 문제마다 sumo_seed 로 SUMO 를 새로 켜므로 어느 워커가 어떤 순서로 풀어도 결과는 같다
    return evaluate_problems(_W["agent"], _W["env"], [problem], deterministic=True,
                             on_result=lambda r: _progress(r, _W["total"]))[0]


def _progress(r, total):
    mark = {"arrival": "도착", "collision": "충돌", "timeout": "시간초과",
            "start_failed": "출발실패", "teleport": "순간이동"}[r["ended_by"]]
    print(f"  #{r['id']:<3d} (전체 {total}문제) {mark:4s} {r.get('level', ''):5s} "
          f"{r.get('origin', '')} → {r.get('destination', '')}", flush=True)


def run_eval_set(args, model_path):
    with open(args.eval_set, encoding="utf-8") as f:
        problems = json.load(f)
    if args.limit:
        # 앞에서부터 자르면 한 쌍(시간대 3개)만 나오므로, 쌍은 고르게 건너뛰고 시간대는 돌아가며 고른다
        # (시험지는 쌍마다 night/day/rush 3문제가 연달아 있다)
        n = len(problems)
        idx = {min((k * n // args.limit) // 3 * 3 + k % 3, n - 1) for k in range(args.limit)}
        problems = [problems[i] for i in sorted(idx)]
    name = "sumo_expert" if model_path is None else \
        f"{os.path.basename(os.path.dirname(model_path))}/{os.path.basename(model_path)}"
    out = args.out or os.path.join(
        os.path.join(BASE, RESULTS_DIR, "sumo_expert") if model_path is None
        else os.path.dirname(model_path),
        f"{os.path.splitext(os.path.basename(args.eval_set))[0]}_results"
        + ("" if not args.limit else f"_limit{args.limit}") + ".json")
    workers = 1 if not args.nogui else max(1, min(args.workers, len(problems)))
    print(f"시험지: {args.eval_set} ({len(problems)}문제, workers={workers})")

    t0 = time.time()
    if workers == 1:
        env = build_env(gui=not args.nogui)
        try:
            agent = make_agent(model_path, env, args.algorithm)
            records = evaluate_problems(agent, env, problems, deterministic=True,
                                        on_result=lambda r: _progress(r, len(problems)))
        finally:
            env.close()
    else:
        # 시험지 순서 그대로 (출근 문제를 먼저 몰아 주면 8개가 동시에 무거운 출근 SUMO 를 돌려 오히려
        # 2배 넘게 느려졌다 — 2026-11-20 PPO 검증 2회차). 시험지는 쌍마다 심야/주간/출근이 섞여 있다.
        order = list(problems)
        pool = multiprocessing.get_context("spawn").Pool(
            workers, initializer=_eval_init, initargs=(model_path, args.algorithm, len(problems)))
        try:
            records = list(pool.imap_unordered(_eval_one, order, chunksize=1))
            pool.close()
        except BaseException:
            pool.terminate()
            raise
        finally:
            pool.join()
        records = sorted(records, key=lambda r: r["id"])

    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump({"name": name, "model": model_path, "eval_set": args.eval_set,
                   "seconds": round(time.time() - t0), "records": records},
                  f, ensure_ascii=False, indent=1)
    return records, out


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="학습된 모델 재생/평가")
    parser.add_argument("model", nargs="?", default=None,
                        help="model.pt 경로 (생략 시 최근 run 자동 선택)")
    parser.add_argument("--episodes", type=int, default=3,
                        help="재생/평가할 에피소드 수")
    parser.add_argument("--nogui", action="store_true",
                        help="화면 없이 주행 지표만 측정 (빠름)")
    parser.add_argument("--algorithm", choices=["auto", "ppo", "bc"], default="auto",
                        help="기본은 모델 파일에서 PPO/BC 자동 판별")
    parser.add_argument("--expert", action="store_true",
                        help="모델 대신 선생님 정책(sumo-expert)을 평가 (기준선)")
    parser.add_argument("--eval-set", default=None,
                        help="고정 시험지 JSON (inference/eval_set/eval_set.json). "
                             "주면 --episodes 대신 시험지의 문제를 모두 푼다")
    parser.add_argument("--workers", type=int, default=1,
                        help="--eval-set에서 SUMO를 몇 개 병렬로 돌릴지 (--nogui일 때만)")
    parser.add_argument("--limit", type=int, default=0,
                        help="--eval-set에서 문제 일부만 (고르게 건너뛰며) 빠른 점검용")
    parser.add_argument("--out", default=None,
                        help="--eval-set 결과 JSON 경로 (기본: 모델 폴더/eval_set_results.json)")
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("--episodes는 1 이상이어야 합니다.")
    if args.expert and args.model:
        parser.error("--expert와 모델 경로는 함께 쓸 수 없습니다.")
    if args.eval_set and not args.nogui and args.workers > 1:
        parser.error("--workers는 --nogui와 함께 쓰세요 (GUI는 창 하나로 차례로 푼다).")

    if args.expert:
        model_path = None
    else:
        model_path = args.model or find_latest_model()
        if not os.path.isabs(model_path):
            model_path = os.path.join(BASE, model_path)
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"모델 파일이 없습니다: {model_path}")
        print(f"모델 로드: {model_path}")

    if args.eval_set:
        if not args.nogui:
            print("※ 문제마다 SUMO 창이 새로 뜹니다. 창이 뜨면 ▶(플레이) 버튼을 누르세요.")
        records, out = run_eval_set(args, model_path)
        print(f"\n결과 저장: {out}")
        eval_report.print_single(eval_report.load(out)[0], records)
        raise SystemExit(0)

    env = build_env(gui=not args.nogui)

    if not args.nogui:
        print("※ 창이 뜨면 ▶(플레이) 버튼을 눌러야 에피소드가 시작됩니다.")
        print("  에피소드가 끝나면 다음 에피소드 창이 다시 뜨고, 또 ▶를 누르면 됩니다.")

    try:
        agent = make_agent(model_path, env, args.algorithm)
        metrics = evaluate_policy(agent, env, n_episodes=args.episodes,
                                  deterministic=True)
    finally:
        env.close()

    print(f"\n─── 주행 지표 ({args.episodes} 에피소드 평균) ───")
    print(f"  충돌률       : {metrics.collision_rate:.0%}")
    print(f"  완주율       : {metrics.success_rate:.0%}")
    print(f"  시간초과율   : {metrics.timeout_rate:.0%}")
    print(f"  평균 속도    : {metrics.mean_speed:.2f} m/s "
          f"({metrics.mean_speed * 3.6:.1f} km/h)")
    print(f"  평균 차간거리: {metrics.mean_gap:.2f} m")
    print(f"  최소 차간거리: {metrics.min_gap:.2f} m")
    print(f"  차선변경     : {metrics.lane_changes:.1f} 회/에피소드")
    print(f"  평균 보상    : {metrics.ep_return:.2f}")
    print(f"  평균 길이    : {metrics.ep_length:.0f} 스텝")
