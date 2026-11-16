"""6단계: 시뮬레이션을 실제로 돌려 실측과 비교하고 안정성을 검사한다.

    python -m yeouido.validate              # 3개 수준 모두
    python -m yeouido.validate --level rush

수준마다 SUMO를 화면 없이 끝까지 돌린 뒤
  - 측정 도로 14곳: 측정 구간(WARMUP~END) 동안 실제로 지나간 대수 vs 목표
    목표 = TOPIS 실측 × DEMAND_SCALE (실측 원값도 함께 기록)
    GEH = sqrt(2(M-C)^2 / (M+C)),  M = 시뮬레이션 대/시, C = 목표 대/시
    교통공학 관례: GEH < 5 인 지점이 85% 이상이면 목표 교통량을 잘 재현한 것으로 본다.
  - teleport(끼어서 강제 이동된 차), 충돌, 출발하지 못하고 대기 중인 차, 평균 속도
결과는 env/yeouido/validation.json 에 저장한다.
"""
import argparse
import json
import math
import subprocess
import xml.etree.ElementTree as ET

from yeouido.config import (DEMAND_SCALE, END, LEVELS, MEASURE, NAME, OUT, WARMUP,
                            sumo_bin)
from yeouido.map_counts import MAPPING
from yeouido.prepare_counts import COUNTS
from yeouido.tdata_signals import additional_files, controlled_ids

REPORT = OUT / "validation.json"
STEP_LENGTH = 0.5   # env/mdp_config.py SIMULATION["step_length"] 와 동일


def geh(m, c):
    return math.sqrt(2 * (m - c) ** 2 / (m + c)) if m + c > 0 else 0.0


def run_level(level, hours, strict=False):
    edgedata_add = OUT / f"measure_{level}.add.xml"
    edgedata_out = OUT / f"measure_{level}.xml"
    stats_out = OUT / f"stats_{level}.xml"
    edgedata_add.write_text(
        f'<additional>\n    <edgeData id="m" file="{edgedata_out.name}" '
        f'begin="{WARMUP}" end="{END}" excludeEmpty="true"/>\n</additional>\n')
    cmd = [sumo_bin("sumo"), "-c", str(OUT / f"{NAME}_{level}.sumocfg"),
           "--additional-files", additional_files(edgedata_add, level=level),
           "--step-length", str(STEP_LENGTH),
           "--statistic-output", str(stats_out),
           # 충돌 판정은 수업 환경(env/sumo_env.py)과 같은 SUMO 기본값: 차로 위 충돌만 본다.
           # --strict 면 교차로 안 충돌까지 본다 (OSM·표준노드링크 교차로 모양 오차로 생기는
           # 경로 겹침까지 잡혀 더 많이 나온다).
           "--collision.action", "warn", "--collision.check-junctions", str(strict).lower(),
           "--no-step-log", "--no-warnings", "--duration-log.statistics"]
    subprocess.run(cmd, check=True, cwd=OUT, stdout=subprocess.DEVNULL)

    counts = json.loads(COUNTS.read_text())
    mapping = json.loads(MAPPING.read_text())["counts"]
    # 그 도로에서 출발한 차(departed)도 그 도로를 지나간 차로 센다
    sim = {e.get("id"): float(e.get("entered", 0)) + float(e.get("departed", 0))
           for e in ET.parse(edgedata_out).getroot().iter("edge")}
    points = []
    for key, c in counts.items():
        obs = sum(c["hourly"][h] for h in hours) / len(hours)
        target = obs * DEMAND_SCALE
        got = sim.get(mapping[key]["edge"], 0.0) * 3600.0 / MEASURE
        points.append(dict(key=key, name=c["name"], observed=round(obs), target=round(target),
                           simulated=round(got), geh=round(geh(got, target), 2)))

    st = ET.parse(stats_out).getroot()
    veh, tele = st.find("vehicles"), st.find("teleports")
    trips, safety = st.find("vehicleTripStatistics"), st.find("safety")
    ok = sum(p["geh"] < 5 for p in points)
    return dict(
        level=level, demand_scale=DEMAND_SCALE, strict_collisions=strict,
        tdata_controllers=len(controlled_ids(level)),
        geh_under5=f"{ok}/{len(points)}",
        geh_under5_ratio=round(ok / len(points), 3),
        inserted=int(veh.get("inserted")), waiting_to_insert=int(veh.get("waiting")),
        teleports=int(tele.get("total")), jam_teleports=int(tele.get("jam")),
        collisions=int(safety.get("collisions")),
        mean_speed_kmh=round(float(trips.get("speed")) * 3.6, 1),
        mean_trip_s=round(float(trips.get("duration")), 1),
        mean_wait_s=round(float(trips.get("waitingTime")), 1),
        points=points)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--level", choices=list(LEVELS), help="하나만 검사 (기본: 전부)")
    ap.add_argument("--strict", action="store_true", help="교차로 안 충돌까지 검사")
    args = ap.parse_args(argv)
    report = json.loads(REPORT.read_text()) if REPORT.exists() else {}
    for level in [args.level] if args.level else LEVELS:
        print(f"[validate] {level} 시뮬레이션 중...")
        r = run_level(level, LEVELS[level]["hours"], args.strict)
        report[level] = r
        print(f"   GEH<5: {r['geh_under5']}  투입 {r['inserted']}대  투입대기 {r['waiting_to_insert']}대  "
              f"teleport {r['teleports']}(정체 {r['jam_teleports']})  충돌 {r['collisions']}  "
              f"평균속도 {r['mean_speed_kmh']}km/h  평균대기 {r['mean_wait_s']}s")
        for p in r["points"]:
            flag = "" if p["geh"] < 5 else "  ←"
            print(f"     {p['key']:10s} {p['name']:12s} 실측 {p['observed']:5d}  "
                  f"목표 {p['target']:5d}  시뮬 {p['simulated']:5d}  GEH {p['geh']:5.2f}{flag}")
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=1))
    print(f"[validate] → {REPORT}")


if __name__ == "__main__":
    main()
