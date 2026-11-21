"""모든 모델을 같은 조건으로 평가하기 위한 고정 시험지(150문제)를 만든다.

    python -m eval_set.build_eval_set             # → eval_set/eval_set.json
    python -m eval_set.build_eval_set --check     # 만든 뒤 150문제 모두 env.reset 이 되는지 확인

한 문제 = env.reset(options=...) 에 그대로 넣는 조건:
  level          심야/주간/출근 (night/day/rush)
  origin         출발지 (랜드마크 이름 또는 "게이트 A"~"게이트 F", "게이트 ㄱ"~"게이트 ㄷ")
  destination    도착지
  snapshot_time  워밍업 시점 (배경 교통이 몇 초 흐른 상태에서 출발할지: 600/1200/1800/2400)
  sumo_seed      SUMO 난수 시드 (배경 차량의 무작위 움직임까지 같게)
나머지 칸(category, bridges, route_m)은 결과를 종류별로 묶어 보기 위한 참고값이다.

구성: 출발·도착 50쌍 × 시간대 3개 = 150문제
  섬↔본토 20쌍  랜드마크끼리, 샛강을 건너는 쌍 (섬→본토 10, 본토→섬 10)
  섬 안·본토 안 15쌍  랜드마크끼리, 샛강을 건너지 않는 쌍
  게이트 15쌍  게이트 9곳이 모두 한 번씩 출발지가 되고, 6곳은 도착지로도 한 번 더
워밍업 시점은 쌍마다 시간대별로 돌아가며 배정해 네 시점이 고르게 쓰인다.
"""
import argparse
import json
import random

from yeouido.config import NET, sumo_home
from yeouido.landmarks import RESULT as LANDMARK_FILE
from yeouido.map_counts import MAPPING
from pathlib import Path

from yeouido_cross.build_routes import BRIDGES, ISLAND, MAINLAND, island_edges

OUT = Path(__file__).resolve().parent     # eval_set/

LEVELS = ["night", "day", "rush"]
SNAPSHOTS = [600, 1200, 1800, 2400]
MIN_ROUTE = 1200.0       # env 의 min_trip(1000 m)보다 여유 있게 (env 는 findRoute 길이로 다시 잰다)
SEED = 2026              # 쌍 고르기용
SUMO_SEED_BASE = 1000    # 문제 id 0 → sumo_seed 1000, id 1 → 1001, ...


def places(net, lms):
    """{이름: (출발 edge, 출발 pos, 도착 edge, 도착 pos)} — YeouidoEnv._place_edge 와 같은 규칙."""
    out = {n: (lm["edge"], lm["pos"], lm["edge"], lm["pos"]) for n, lm in lms.items()}
    for g, v in json.loads(MAPPING.read_text())["gates"].items():
        out[f"게이트 {g}"] = (v["in"], 0.0, v["out"], net.getEdge(v["out"]).getLength())
    return out


def candidates(net, lms):
    pl = places(net, lms)
    island = island_edges(net, net.getEdge(lms["여의도공원"]["edge"]))
    side = {n: (ISLAND if net.getEdge(lm["edge"]) in island else MAINLAND) for n, lm in lms.items()}
    rows = []
    for o, (oe, op, _, _) in pl.items():
        for d, (_, _, de, dp) in pl.items():
            if o == d:
                continue
            path, _ = net.getFastestPath(net.getEdge(oe), net.getEdge(de), vClass="passenger")
            if not path:
                continue
            length = sum(e.getLength() for e in path) - op - (net.getEdge(de).getLength() - dp)
            ids = {e.getID() for e in path}
            bridges = [b for b, edges in BRIDGES.items() if ids & set(edges)]
            if o.startswith("게이트") or d.startswith("게이트"):
                cat = "게이트"
            elif side[o] != side[d]:
                cat = f"섬↔본토"
            else:
                cat = f"{side[o]} 안"
            rows.append(dict(origin=o, destination=d, category=cat, bridges=bridges,
                             route_m=round(length), direction=f"{side.get(o, '-')}→{side.get(d, '-')}"))
    return [r for r in rows if r["route_m"] >= MIN_ROUTE]


def choose(rows, rng):
    by = lambda f: [r for r in rows if f(r)]  # noqa: E731
    picked = []
    # 섬↔본토 20쌍: 방향별 10쌍
    for direction in (f"{ISLAND}→{MAINLAND}", f"{MAINLAND}→{ISLAND}"):
        pool = by(lambda r: r["category"] == "섬↔본토" and r["direction"] == direction)
        picked += rng.sample(pool, min(10, len(pool)))
    # 섬 안·본토 안 15쌍: 본토 안을 먼저 다 넣고(쌍이 적음) 나머지를 섬 안으로
    inner_main = by(lambda r: r["category"] == f"{MAINLAND} 안")
    inner_isl = by(lambda r: r["category"] == f"{ISLAND} 안")
    take_main = rng.sample(inner_main, min(6, len(inner_main)))
    picked += take_main + rng.sample(inner_isl, min(15 - len(take_main), len(inner_isl)))
    # 게이트 15쌍: 게이트마다 출발 1번 (도착은 랜드마크), 그다음 6곳은 도착 1번 (출발은 랜드마크)
    gates = sorted({r["origin"] for r in rows if r["origin"].startswith("게이트")})
    for g in gates:
        pool = by(lambda r: r["origin"] == g and not r["destination"].startswith("게이트"))
        picked.append(rng.choice(pool))
    for g in rng.sample(gates, 6):
        pool = by(lambda r: r["destination"] == g and not r["origin"].startswith("게이트"))
        picked.append(rng.choice(pool))
    return picked


def build():
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET))
    lms = json.loads(LANDMARK_FILE.read_text())
    rng = random.Random(SEED)
    pairs = choose(candidates(net, lms), rng)
    problems = []
    for p_idx, pair in enumerate(pairs):
        for l_idx, level in enumerate(LEVELS):
            pid = len(problems)
            problems.append(dict(id=pid, level=level, origin=pair["origin"],
                                 destination=pair["destination"],
                                 snapshot_time=SNAPSHOTS[(p_idx + l_idx) % len(SNAPSHOTS)],
                                 sumo_seed=SUMO_SEED_BASE + pid,
                                 category=pair["category"], bridges=pair["bridges"],
                                 route_m=pair["route_m"]))
    return problems


def build_random(n, seed):
    """무작위 시험지: 가능한 출발·도착 쌍 전체(1.2 km 이상)에서 n문제를 고르게 뽑는다.

    쌍·시간대·워밍업 시점을 모두 무작위로 정하되 seed 로 고정해, 다른 모델도 같은 문제로 비교할 수 있다.
    기준선 측정처럼 "어떤 출발·도착이 나와도" 성능을 보고 싶을 때 쓴다.
    """
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET))
    lms = json.loads(LANDMARK_FILE.read_text())
    rng = random.Random(seed)
    rows = candidates(net, lms)
    problems = []
    for pid in range(n):
        pair = rng.choice(rows)
        problems.append(dict(id=pid, level=rng.choice(LEVELS), origin=pair["origin"],
                             destination=pair["destination"],
                             snapshot_time=rng.choice(SNAPSHOTS),
                             sumo_seed=SUMO_SEED_BASE + seed * 10_000 + pid,
                             category=pair["category"], bridges=pair["bridges"],
                             route_m=pair["route_m"]))
    print(f"[eval_set] 후보 쌍 {len(rows)}개에서 {n}문제를 무작위로 뽑음 (seed {seed})")
    return problems


def check(problems):
    """150문제 모두 env.reset 이 되는지 (경로 있음, 1 km 이상, 출발 지점이 막히지 않음)."""
    import os
    import sys
    from pathlib import Path
    root = Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(root))
    os.chdir(root)
    from env.mdp_config import ACTION, OBSERVATION, REWARD, SIMULATION
    from env.road_config import EGO, ROAD, TRAFFIC
    from env.yeouido_env import make_env
    env = make_env(cfg_path=None, road=ROAD, ego=EGO, mdp_sim=SIMULATION, mdp_obs=OBSERVATION,
                   action=ACTION, reward=REWARD, traffic=TRAFFIC, gui=False)
    missing = [n for p in problems for n in (p["origin"], p["destination"]) if n not in env.places]
    if missing:
        raise RuntimeError(f"env 의 출발·도착 후보에 없는 이름: {sorted(set(missing))} "
                           "(env/road_config.py places 가 landmarks+gates 인지 확인)")
    failed = []
    for p in problems:
        keys = ("level", "origin", "destination", "snapshot_time", "sumo_seed")
        try:
            _, info = env.reset(options={k: p[k] for k in keys})
            p["route_m"] = round(info["route_length"])      # env(findRoute) 기준 길이로 갱신
        except RuntimeError as e:
            failed.append((p["id"], str(e)))
    env.close()
    return failed


def summary(problems):
    from collections import Counter
    print(f"[eval_set] {len(problems)}문제 / {len(problems) // len(LEVELS)}쌍")
    for title, key in (("종류", "category"), ("시간대", "level"), ("워밍업 시점", "snapshot_time")):
        print(f"  {title}: " + ", ".join(f"{k} {v}" for k, v in Counter(p[key] for p in problems).items()))
    br = Counter(b for p in problems for b in p["bridges"])
    print("  지나는 샛강 연결로: " + ", ".join(f"{k} {v}" for k, v in br.items()))
    ends = Counter(n for p in problems[::3] for n in (p["origin"], p["destination"]))
    print("  장소별 등장 (쌍 기준): " + ", ".join(f"{k} {v}" for k, v in sorted(ends.items())))
    print(f"  경로 길이: {min(p['route_m'] for p in problems)}~{max(p['route_m'] for p in problems)} m")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="150문제 모두 env.reset 이 되는지 확인")
    ap.add_argument("--random", type=int, metavar="N",
                    help="고정 150문제 대신 무작위 N문제 (→ eval_set/random_<N>_s<seed>.json)")
    ap.add_argument("--seed", type=int, default=1, help="--random 의 난수 시드")
    args = ap.parse_args(argv)
    if args.random:
        problems = build_random(args.random, args.seed)
        path = OUT / f"random_{args.random}_s{args.seed}.json"
        path.write_text("[\n" + ",\n".join(json.dumps(p, ensure_ascii=False) for p in problems) + "\n]\n")
        summary(problems)
        print(f"→ {path}")
        return
    problems = build()
    if args.check:
        failed = check(problems)
        if failed:
            for pid, msg in failed:
                print(f"   ! {pid}: {msg}")
            raise SystemExit(f"{len(failed)}문제가 reset 되지 않습니다 (파일을 쓰지 않음)")
        print("[eval_set] 150문제 모두 reset 확인")
    OUT.mkdir(exist_ok=True)
    path = OUT / "eval_set.json"
    path.write_text("[\n" + ",\n".join(json.dumps(p, ensure_ascii=False) for p in problems) + "\n]\n")
    summary(problems)
    print(f"→ {path}")


if __name__ == "__main__":
    main()
