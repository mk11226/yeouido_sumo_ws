"""교통 수요 점검: 시간대 sumocfg 를 끝까지 돌리며 10분마다 일반 차량 배치를 잰다.

    LIBSUMO_AS_TRACI=quiet python -m inference.eval_set.demand_check --demand uniform --level rush

재는 것: 도로 위 차량 수, 정지 비율, 차가 있는 도로 수, 500 m 격자 상위 10 % 칸의 차량 비중(작을수록 고르게),
진입 대기(못 들어온 차), 순간이동 누적, 충돌, 차량 종류 비율, 가장 붐비는 도로 5개.
"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))))  # 저장소 최상위
import argparse, collections, json, os, sys
import traci
from sumolib import checkBinary

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # 저장소 최상위
DATA = os.path.join(BASE, "env", "yeouido")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demand", default="uniform", help="measured 또는 버전 이름 (uniform, uniform_s0.4 ...)")
    ap.add_argument("--level", required=True)
    ap.add_argument("--end", type=float, default=4200)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    suffix = "" if a.demand == "measured" else f"_{a.demand}"
    cfg = os.path.join(DATA, f"yeouido_{a.level}{suffix}.sumocfg")
    traci.start([checkBinary("sumo"), "-c", cfg, "--step-length", "0.5", "--no-warnings", "true",
                 "--no-step-log", "true", "--collision.action", "remove"])
    edges = [e for e in traci.edge.getIDList() if not e.startswith(":")]
    tele = coll = 0
    rows = []
    while traci.simulation.getTime() < a.end:
        traci.simulationStep()
        tele += traci.simulation.getStartingTeleportNumber()
        coll += traci.simulation.getCollidingVehiclesNumber()
        t = traci.simulation.getTime()
        if t % 600 == 0:
            vs = traci.vehicle.getIDList()
            n = len(vs)
            per_edge = collections.Counter(traci.vehicle.getRoadID(v) for v in vs)
            cells = collections.Counter()
            for v in vs:
                x, y = traci.vehicle.getPosition(v)
                cells[(int(x // 500), int(y // 500))] += 1
            top = sorted(cells.values(), reverse=True)
            types = collections.Counter(traci.vehicle.getTypeID(v) for v in vs)
            r = dict(t=int(t), n=n, halting=round(sum(traci.vehicle.getSpeed(v) < 0.1 for v in vs) / max(n, 1), 3),
                     edges_with_cars=sum(1 for e in edges if per_edge.get(e)), edges=len(edges),
                     top10_cells=round(sum(top[:max(1, len(top) // 10)]) / max(n, 1), 3),
                     pending=len(traci.simulation.getPendingVehicles()), teleports=tele, collisions=coll,
                     types={k: round(c / max(n, 1), 3) for k, c in types.items()},
                     busiest=[(traci.edge.getStreetName(e)[:12], c) for e, c in per_edge.most_common(5)])
            rows.append(r)
            print(f"{a.demand} {a.level:5} {int(t):5d}s 차량 {n:5d} 정지 {r['halting']:.0%} 차 있는 도로 "
                  f"{r['edges_with_cars']}/{len(edges)} 상위10%격자 {r['top10_cells']:.0%} 진입대기 {r['pending']} "
                  f"순간이동 {tele} 충돌 {coll} 붐빔 {r['busiest'][:3]}", flush=True)
    traci.close()
    if a.out:
        json.dump(rows, open(a.out, "w"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
