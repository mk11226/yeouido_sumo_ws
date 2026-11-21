"""고르게 분포한 일반 차량(HV) 수요: 시간대별 차량 대수는 그대로, 출발·도착은 도로망 전체에 고르게.

    python -m yeouido.build_demand_uniform              # 세 시간대 모두
    python -m yeouido.build_demand_uniform --scale 0.8  # 대수에 배율 (정체가 너무 심할 때)

왜 (2026-11-21): 기존 수요(build_demand.py)는 출발·도착을 범위 경계로만 잡고 실측 지점 20곳의 대수에 맞춰
경로를 골라서, 주간·출근에 경계 간선도로(노량진로·경인로·여의대방로·선유로·국회대로)에만 차가 몰리고 진입 대기
(출근 2400초 5,002대)와 순간이동(5분에 139대)이 계속 늘며, 도로 1,641개 중 57 %에는 차가 한 대도 없었다.

어떻게:
  - 시간대별 대수 = 기존 수요 파일의 대수 (심야 4,779 · 주간 28,259 · 출근 36,752대 / 70분) × --scale
  - randomTrips 로 출발·도착 도로를 도로망 전체에서 뽑는다. 뽑힐 확률 = 도로 길이 × 차로 수 (차로-km 에 비례)
    → 도로 면적당 차량이 고르다. 50 m 보다 짧은 도로·승용차 금지 도로에서는 출발·도착하지 않는다
    (짧은 도로에서 넣고 빼면 그 자리가 막혀 교착이 생겼던 문제, build_demand.py 주석).
  - 출발 위치는 도로 위 빈 곳(random_free), 도착 위치도 도로 위 아무 곳(random) → 도로 끝에 몰리지 않는다.
  - 차량 종류 4가지(hv 분포)는 기존과 같다 (yeouido.vtypes.xml).
결과: env/yeouido/<level>_uniform.rou.xml, yeouido_<level>_uniform.sumocfg.
환경에서 쓰기: env/road_config.py ROAD["demand"] = "uniform" (워밍업 상태는 env 가 처음 쓸 때 만든다).
"""
import argparse
import subprocess
import sys

import sumolib

from yeouido.build_demand import VTYPES, write_vtypes
from yeouido.config import END, LEVELS, NAME, NET, OUT, sumo_home
from yeouido.tdata_signals import signal_file

SEED = 7
MIN_EDGE = 50.0          # 이보다 짧은 도로에서는 출발·도착하지 않음 (m)
MIN_TRIP = 300.0         # 출발~도착 최소 직선거리 (m)


def planned_counts():
    """기존 수요 파일의 시간대별 차량 대수 (= 들어와야 하는 대수)."""
    return {lv: (OUT / f"{lv}.rou.xml").read_text().count("<vehicle ") for lv in LEVELS}


def write_weights(prefix):
    net = sumolib.net.readNet(str(NET))
    lines = [f'<edgedata>\n    <interval begin="0" end="{END}">']
    n = 0
    for e in net.getEdges():
        if e.getFunction() == "internal" or not e.allows("passenger") or e.getLength() < MIN_EDGE:
            continue
        lanes = sum(1 for ln in e.getLanes() if ln.allows("passenger"))
        lines.append(f'        <edge id="{e.getID()}" value="{e.getLength() * lanes:.1f}"/>')
        n += 1
    lines.append("    </interval>\n</edgedata>\n")
    text = "\n".join(lines)
    for kind in ("src", "dst"):
        (OUT / f"{prefix}.{kind}.xml").write_text(text)
    return n


RELIEF = ('        <ignore-junction-blocker value="20"/>\n'      # 교차로 안에 선 차 때문에 20초 넘게 막히면 무시하고 진행
          '        <time-to-impatience value="30"/>\n')         # 양보하며 30초 넘게 기다리면 더 적극적으로 끼어듦


def write_sumocfg(level, name="uniform", relief=False):
    cfg = OUT / f"{NAME}_{level}_{name}.sumocfg"
    signals = f'        <additional-files value="{signal_file(level).name}"/>\n' if signal_file(level).exists() else ""
    cfg.write_text(f"""<configuration>
    <input>
        <net-file value="{NET.name}"/>
        <route-files value="{VTYPES.name},{level}_{name}.rou.xml"/>
{signals.rstrip()}
    </input>
    <time>
        <begin value="0"/>
        <end value="{END}"/>
    </time>
    <processing>
        <time-to-teleport value="300"/>
{RELIEF.rstrip() if relief else ""}
        <ignore-route-errors value="true"/>
    </processing>
    <random_number>
        <seed value="{SEED}"/>
    </random_number>
</configuration>
""")
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scale", type=float, default=1.0, help="시간대별 대수에 곱할 배율")
    ap.add_argument("--levels", nargs="+", default=list(LEVELS))
    ap.add_argument("--relief", action="store_true",
                    help="교착 완화 옵션 (ignore-junction-blocker 20 s, time-to-impatience 30 s). 일반 차량에만 영향")
    ap.add_argument("--name", default="uniform", help="버전 이름 (파일 <level>_<name>.rou.xml, 환경 ROAD['demand'])")
    args = ap.parse_args()
    tools = sumo_home() / "tools"
    if not VTYPES.exists():
        write_vtypes()
    prefix = "uniform_weights"
    n_edges = write_weights(prefix)
    counts = planned_counts()
    print(f"[uniform] 출발·도착 후보 도로 {n_edges}개 (차로-km 비례), 대수 {counts} × {args.scale}")
    for i, level in enumerate(args.levels):
        n = int(round(counts[level] * args.scale))
        trips = OUT / f"{level}_{args.name}.trips.xml"
        rou = OUT / f"{level}_{args.name}.rou.xml"
        cmd = [sys.executable, str(tools / "randomTrips.py"), "-n", str(NET), "-o", str(trips), "-r", str(rou),
               "-b", "0", "-e", str(END), "-p", f"{END / n:.5f}",
               "--weights-prefix", str(OUT / prefix), "--min-distance", str(MIN_TRIP),
               "--remove-loops", "--validate", "--vclass", "passenger", "--seed", str(SEED + i), "--prefix", f"{level}_",
               "--trip-attributes", 'type="hv" departLane="best" departSpeed="max" departPos="random_free" '
                                    'arrivalPos="random"']
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
        cfg = write_sumocfg(level, args.name, args.relief)
        got = rou.read_text().count("<vehicle ")
        print(f"[uniform] {level:5s} ({LEVELS[level]['label']}): 차량 {got}대 (목표 {n}) → {rou.name}, {cfg.name}")


if __name__ == "__main__":
    main()
