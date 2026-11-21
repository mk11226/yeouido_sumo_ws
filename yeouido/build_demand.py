"""5단계: 실측 교통량 → 시간대별 일반 차량(HV) 수요 + sumocfg.

    python -m yeouido.build_demand

  1) randomTrips.py 로 "후보 경로"를 많이 만든다.
     출발/도착은 범위 경계(다리 끝, 잘린 도로 끝)로만 잡는다. 섬 안 짧은 도로에서
     차를 넣고 빼면 그 자리가 막혀 교착이 생겼기 때문이다.
  2) 교통 수준(night/day/rush)마다 측정 도로별 목표 대수를 edgedata 파일로 쓴다.
     값 = TOPIS 평일 평균(대/시) × DEMAND_SCALE × (시뮬레이션 길이 / 1시간)
  3) routeSampler.py 가 후보 경로 중에서 측정 도로의 목표 대수에 맞는 조합을 고른다.
  4) 수준마다 <level>.rou.xml 과 yeouido_<level>.sumocfg 를 만든다.

도로망은 하나이고 수준마다 차량 파일만 다르다 → "같은 도로, 다른 교통량".
"""
import json
import subprocess
import sys

from yeouido.config import DEMAND_SCALE, END, LEVELS, NAME, NET, OUT, sumo_home
from yeouido.map_counts import MAPPING
from yeouido.prepare_counts import COUNTS
from yeouido.tdata_signals import signal_file

VTYPES = OUT / f"{NAME}.vtypes.xml"
CANDIDATES = OUT / "candidates.rou.xml"
SEED = 42

# 일반 차량 구성: 수업 환경(env/road_config.py TRAFFIC["controllers"])과 같은 차량추종모델 4종·비율·색.
# 도심이라 차종 최고속도 대신 speedFactor(제한속도 대비 배율)로 속도를 준다.
HV_TYPES = {
    "krauss": dict(probability=0.35, carFollowModel="Krauss", accel=2.6, decel=4.5,   # 산만한 운전자 (노랑)
                   sigma=0.5, tau=1.0, color="1,1,0"),
    "idm": dict(probability=0.30, carFollowModel="IDM", accel=2.0, decel=4.0,         # 부드러운 운전자 (하늘)
                tau=1.2, color="0,0.8,1"),
    "eidm": dict(probability=0.20, carFollowModel="EIDM", accel=2.0, decel=4.0,       # 반응 지연 있는 운전자 (주황)
                 tau=1.1, color="1,0.6,0"),
    "acc": dict(probability=0.15, carFollowModel="ACC", accel=2.0, decel=4.0,         # 크루즈 컨트롤 차량 (자주)
                tau=1.5, color="1,0,1"),
}
# 끼어들기 관련 파라미터. SUMO 기본 운전자는 매우 양보적이라 차로가 줄어드는 곳이나
# 회전 차로로 옮겨야 하는 곳에서 빈틈을 오래 기다린다. 끼어들기를 조금 더 적극적으로
# 하되(lcAssertive), 안전 간격을 무시하는 옵션(impatience, 교차로 막힘 무시)은 쓰지 않는다
# → 정체는 생겨도 일반 차량끼리 충돌은 없어야 한다.
HV_LANE_CHANGE = dict(lcAssertive=1.5, lcCooperative=1.0, lcStrategic=1.0,
                      lcLookaheadLeft=2.0)
EGO = dict(accel=3.0, decel=4.5, maxSpeed=22.22)   # AV 기본값 (80 km/h). 학습 단계에서 조정


def write_vtypes():
    lines = [f'    <vType id="ego_type" accel="{EGO["accel"]}" decel="{EGO["decel"]}" '
             f'maxSpeed="{EGO["maxSpeed"]}" color="1,0,0" vClass="passenger" '
             f'guiShape="passenger/sedan"/>',
             '    <vTypeDistribution id="hv">']
    for name, t in HV_TYPES.items():
        attrs = " ".join(f'{k}="{v}"' for k, v in {**t, **HV_LANE_CHANGE}.items())
        lines.append(f'        <vType id="hv_{name}" {attrs} speedFactor="normc(1.0,0.1,0.7,1.3)" '
                     f'minGap="2.5" vClass="passenger" guiShape="passenger/sedan"/>')
    lines.append("    </vTypeDistribution>")
    VTYPES.write_text("<routes>\n" + "\n".join(lines) + "\n</routes>\n")


def write_candidates(tools):
    trips = OUT / "candidates.trips.xml"
    cmd = [sys.executable, str(tools / "randomTrips.py"),
           "-n", str(NET), "-o", str(trips), "-r", str(CANDIDATES),
           "-b", "0", "-e", "3600", "-p", "0.25",          # 14,400개 후보
           "--fringe-factor", "max", "--min-distance", "400",
           "--remove-loops", "--validate", "--seed", str(SEED),
           "--vclass", "passenger"]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)


def write_edgedata(level, hours):
    counts = json.loads(COUNTS.read_text())
    mapping = json.loads(MAPPING.read_text())["counts"]
    path = OUT / f"counts_{level}.xml"
    scale = DEMAND_SCALE * END / 3600.0
    lines = [f'<data>\n    <interval id="{level}" begin="0" end="{END}">']
    for key, c in counts.items():
        vph = sum(c["hourly"][h] for h in hours) / len(hours)
        lines.append(f'        <edge id="{mapping[key]["edge"]}" entered="{vph * scale:.0f}"/>'
                     f'  <!-- {key} {c["name"]} 실측 {vph:.0f}대/시 -->')
    lines.append("    </interval>\n</data>\n")
    path.write_text("\n".join(lines))
    return path


def write_sumocfg(level):
    cfg = OUT / f"{NAME}_{level}.sumocfg"
    signals = (f'        <additional-files value="{signal_file(level).name}"/>\n'
               if signal_file(level).exists() else '')
    cfg.write_text(f"""<configuration>
    <input>
        <net-file value="{NET.name}"/>
        <route-files value="{VTYPES.name},{level}.rou.xml"/>
{signals.rstrip()}
    </input>
    <time>
        <begin value="0"/>
        <end value="{END}"/>
    </time>
    <processing>
        <time-to-teleport value="300"/>
        <ignore-route-errors value="true"/>
    </processing>
    <random_number>
        <seed value="{SEED}"/>
    </random_number>
</configuration>
""")
    return cfg


def main():
    tools = sumo_home() / "tools"
    write_vtypes()
    print("[demand] 후보 경로 생성 중 (randomTrips + duarouter)...")
    write_candidates(tools)
    for level, spec in LEVELS.items():
        data = write_edgedata(level, spec["hours"])
        rou = OUT / f"{level}.rou.xml"
        cmd = [sys.executable, str(tools / "routeSampler.py"),
               "-r", str(CANDIDATES), "--edgedata-files", str(data), "-o", str(rou),
               "--prefix", f"{level}_", "--seed", str(SEED),
               "--attributes", 'type="hv" departLane="best" departSpeed="max"']
        r = subprocess.run(cmd, check=True, capture_output=True, text=True)
        cfg = write_sumocfg(level)
        summary = [ln for ln in r.stdout.splitlines() if "GEH" in ln or "vehicles" in ln.lower()]
        n = rou.read_text().count("<vehicle ")
        print(f"[demand] {level:5s} ({spec['label']}): 차량 {n}대 → {cfg.name}")
        for ln in summary:
            print("         " + ln.strip())


if __name__ == "__main__":
    main()
