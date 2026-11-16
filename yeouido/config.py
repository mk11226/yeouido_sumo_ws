"""여의도 도로 환경의 공통 설정 — 범위, 측정 지점, 교통 수준, 파일 경로.

파이프라인의 모든 단계가 이 파일 하나를 기준으로 동작한다.
범위나 시간대를 바꾸고 싶으면 여기만 수정한 뒤 run_all.py를 다시 실행한다.
"""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "env" / "yeouido"            # 생성 결과 (net/rou/sumocfg 등)
SOURCE = OUT / "source"                   # 다운로드한 원본 (TOPIS 엑셀, OSM)

NAME = "yeouido"
NET = OUT / f"{NAME}.net.xml"

# ── 도로 범위 (경도/위도) ─────────────────────────────────────
# 영등포구 북부 전체: 여의도 + 당산·양화대교 + 영등포역·문래 + 오목교 입구.
# TOPIS 측정 지점 10곳이 들어간다. 다리는 강 중간에서 잘리며, 잘린 끝이
# 차량이 들어오고 나가는 입구가 된다. (약 6.2 km × 4.4 km)
BBOX = dict(west=126.880, south=37.505, east=126.950, north=37.545)
# 한강 남쪽(영등포구)만 쓰기 위한 경계선 (경도, 위도). 이 선보다 북쪽(마포구) 도로는 뺀다.
# 강 수면 폴리곤에서 구한 한강 중앙선을 약 45 m 북쪽으로 올린 것 → 다리는 강 중간에서
# 잘리고, 다리 위 측정 지점(양화·서강·마포·원효대교)은 남쪽 조각에 남는다.
# 북쪽 마포 도로(토정로 등)를 넣었을 때 범위 경계 신호 교차로에서 충돌이 잦았다.
RIVER_CUT = [(126.880, 37.5504), (126.9035, 37.5438), (126.915, 37.5396), (126.9253, 37.5391),
             (126.9366, 37.5345), (126.944, 37.5276), (126.950, 37.5243)]


def south_of_river(lon, lat):
    pts = RIVER_CUT
    if lon <= pts[0][0]:
        return lat < pts[0][1]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if lon <= x1:
            return lat < y0 + (y1 - y0) * (lon - x0) / (x1 - x0)
    return lat < pts[-1][1]

# OSM에서 가져올 도로 등급. 주거지 골목(residential, service)은 빼서
# 학습에 필요 없는 작은 길로 도로망이 복잡해지지 않게 한다.
HIGHWAY_CLASSES = [
    "motorway", "motorway_link", "trunk", "trunk_link",
    "primary", "primary_link", "secondary", "secondary_link",
    "tertiary", "tertiary_link",
]
# 다운로드는 하되 도로망에서는 뺄 등급: 도시고속도로(올림픽대로·강변북로·신월여의지하도로)와 램프.
# 측정값이 없어 교통량을 정할 수 없고, 복잡한 입체 램프 합류부가 SUMO 교착의 주원인이었다.
# 다리 교통량은 다리 위 측정 지점에서 맞추므로 고속도로에서 들어오는 차도 그대로 반영된다.
EXCLUDED_CLASSES = {"motorway", "motorway_link", "trunk", "trunk_link"}

# ── TOPIS 교통량 조사 ────────────────────────────────────────
# 서울시 TOPIS 자료실의 월별 "서울시 교통량 조사자료" 엑셀 (지점별·시간대별 실측).
TOPIS_YEAR = 2026
TOPIS_MONTHS = [7, 8, 9]

# AV 출발지·목적지 랜드마크. 위치는 OSM 객체(건물 윤곽, 역, 공원)를 직접 지정해 정한다.
# (자동 추출 목록은 "국회의사당"을 지하철역으로 잡는 등 틀릴 수 있어 쓰지 않는다)
# 좌표는 그 객체의 중심(건물·공원은 면적 중심)이고, landmarks.py 가 가장 가까운 차도에 연결한다.
LANDMARKS = {
    "국회의사당": ("way", 270596342),        # 국회 본관 건물
    "IFC 서울": ("node", 13438315405),        # IFC몰
    "63빌딩": ("way", 64989671),
    "영등포역": ("way", 624851882),           # 영등포역 역사 (타임스퀘어 바로 옆)
    "당산역": ("node", 6033312683),           # 2호선 당산역
    "노량진수산시장": ("way", 606419988),
    "여의도공원": ("way", 682334253),
}

# 범위 안에 있는 측정 지점. gate가 있는 지점은 AV의 출발지/목적지(A~I)로 쓴다.
# (D-46 서부간선도로는 도시고속도로라 도로망에서 빠지므로 제외)
COUNT_POINTS = {
    "C-06": dict(name="서강대교", gate="A"),
    "C-07": dict(name="마포대교", gate="B"),
    "C-08": dict(name="원효대교", gate="C"),
    "D-29": dict(name="여의대방로(여의교)", gate="D"),
    "D-28": dict(name="경인로(서울교)", gate="E"),
    "D-27": dict(name="국회대로(여의2교)", gate="F"),
    "C-05": dict(name="양화대교", gate="G"),
    "D-25": dict(name="영등포로(오목교)", gate="H"),
    "D-23": dict(name="경인로(거리공원)", gate="I"),
    "D-41": dict(name="노들로", gate=None),
}

# TOPIS의 방향 정의: 유입 = 외곽 → 서울시청, 유출 = 서울시청 → 외곽.
# 측정 지점의 두 방향 도로 중 어느 쪽이 유입인지 정할 때 쓴다.
SEOUL_CITY_HALL = (126.9780, 37.5665)
# 범위 중심. 게이트 두 도로 중 "범위 안으로 들어오는 쪽"을 정할 때 쓴다.
AREA_CENTER = ((BBOX["west"] + BBOX["east"]) / 2, (BBOX["south"] + BBOX["north"]) / 2)

# ── 교통 수준 (평일 평균의 어느 시간대를 쓸지) ─────────────────
LEVELS = {
    "night": dict(hours=[3], label="심야 03시"),
    "day": dict(hours=[12], label="주간 12시"),
    "rush": dict(hours=[8], label="출근 08시"),
}

# 실측 교통량에 곱하는 배율 (세 수준 모두 같은 값 → 시간대 간 비율은 그대로 유지).
# 신호 시간 계획이 공개되지 않아 SUMO가 신호를 추정하므로, 이 도로망이 실제로
# 처리할 수 있는 교통량은 실제 도로보다 적다. 특히 여의도로 드나드는 다리 양끝 교차로가
# 병목이다. 70분 시뮬레이션 결과 (충돌은 세 배율 모두 0건):
#   0.40 → 주간 teleport 35, 출근 92 (정체가 계속 쌓임)
#   0.30 → 주간 0, 출근 5 (출근 정체가 서서히 쌓임)
#   0.25 → 주간 0, 출근 0 (출근은 주간보다 차가 약 40% 많고 느린 정체가 유지됨)
DEMAND_SCALE = 1.0

# 시뮬레이션 길이: 앞 WARMUP초 동안 도로를 채우고, 이후 1시간을 측정 구간으로 쓴다.
WARMUP = 600
MEASURE = 3600
END = WARMUP + MEASURE


def sumo_home() -> Path:
    """사용할 SUMO 설치 폴더를 찾고 tools/ 를 import 경로에 추가한다.

    pip의 eclipse-sumo 패키지를 우선 쓴다. OSM 좌표 변환에 필요한 PROJ가
    항상 포함되어 있기 때문이다. (직접 빌드한 SUMO는 PROJ 없이 빌드되면
    netconvert가 OSM을 읽지 못한다.) 없으면 SUMO_HOME 환경변수를 쓴다.
    """
    try:
        import sumo
        home = sumo.SUMO_HOME
    except ImportError:
        home = os.environ.get("SUMO_HOME")
    if not home or not Path(home).is_dir():
        raise RuntimeError(
            "SUMO를 찾지 못했습니다. "
            "pip install -r yeouido/requirements.txt 를 먼저 실행하세요.")
    tools = str(Path(home) / "tools")
    if tools not in sys.path:
        sys.path.append(tools)
    return Path(home)


def sumo_bin(name: str) -> str:
    """sumo_home()의 bin/ 안에 있는 실행 파일 경로 (netconvert, sumo, sumo-gui 등)."""
    exe = sumo_home() / "bin" / (name + (".exe" if os.name == "nt" else ""))
    if not exe.exists():
        raise RuntimeError(f"{exe} 가 없습니다")
    return str(exe)
