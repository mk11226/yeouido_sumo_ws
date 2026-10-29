"""4단계: TOPIS 측정 지점 → SUMO 도로(edge) 연결, AV 출발/도착 게이트(A~F) 지정.

    python -m yeouido.map_counts

측정 지점 하나에는 방향이 2개(유입/유출) 있다. 각 방향마다
  1) 지점 좌표 반경 RADIUS m 안의 도로 중
  2) TOPIS 설명의 도로명([국회대로] 등)과 이름이 같은 도로를 우선으로,
  3) 진행 방향이 맞는 도로를 고른다.
     유입 = 서울시청 쪽으로 향하는 방향, 유출 = 그 반대 (TOPIS 정의)
  4) 그중 지점과 가장 가까운 도로.
자동 선택이 틀린 곳은 config.py 대신 아래 OVERRIDE에 edge ID를 직접 적는다.

게이트: 측정 지점 두 도로 중 범위 중심으로 향하는 쪽이 "in"(진입),
반대쪽이 "out"(진출). AV는 한 게이트의 in에서 출발해 다른 게이트의 out으로 나간다.
"""
import json
import math
import re

from yeouido.config import (COUNT_POINTS, AREA_CENTER, NET, OUT,
                            SEOUL_CITY_HALL, sumo_home)
from yeouido.prepare_counts import COUNTS

MAPPING = OUT / "count_edges.json"
RADIUS = 120.0
OVERRIDE = {
    # "C-07/유입": "edge_id",
}


def _unit(dx, dy):
    n = math.hypot(dx, dy) or 1.0
    return dx / n, dy / n


def _edge_dir_at(edge, x, y):
    """도로 모양에서 (x, y)와 가장 가까운 구간의 진행 방향 단위벡터."""
    shape = edge.getShape()
    best = None
    for (x1, y1), (x2, y2) in zip(shape, shape[1:]):
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy or 1.0
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
        d = math.hypot(x1 + t * dx - x, y1 + t * dy - y)
        if best is None or d < best[0]:
            best = (d, _unit(dx, dy))
    return best[1]


def pick_edge(net, key, info):
    x, y = net.convertLonLat2XY(info["lon"], info["lat"])
    road = re.match(r"\[(.+?)\]", info["desc"] or "")
    road = road.group(1) if road else ""
    toward_hall = _unit(*(a - b for a, b in zip(net.convertLonLat2XY(*SEOUL_CITY_HALL), (x, y))))
    sign = 1 if info["direction"] == "유입" else -1

    cands = []
    for edge, dist in net.getNeighboringEdges(x, y, RADIUS):
        if edge.getFunction() == "internal" or not edge.allows("passenger"):
            continue
        ux, uy = _edge_dir_at(edge, x, y)
        heading = sign * (ux * toward_hall[0] + uy * toward_hall[1])
        if heading < 0.2:   # 반대 방향 또는 거의 수직인 도로 제외
            continue
        cands.append((edge, dist, road and road in (edge.getName() or "")))
    if not cands:
        raise RuntimeError(f"{key}: 반경 {RADIUS}m 안에 방향이 맞는 도로가 없습니다")
    named = [c for c in cands if c[2]]
    edge, dist, _ = min(named or cands, key=lambda c: c[1])
    return edge, dist, bool(named)


def main():
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET))
    counts = json.loads(COUNTS.read_text())

    mapping = {}
    for key, info in counts.items():
        if key in OVERRIDE:
            edge, dist, named = net.getEdge(OVERRIDE[key]), 0.0, True
        else:
            edge, dist, named = pick_edge(net, key, info)
        mapping[key] = dict(edge=edge.getID(), street=edge.getName(), dist_m=round(dist, 1),
                            lanes=edge.getLaneNumber(), name_match=named)
        print(f"   {key:10s} {info['desc']:34s} → {edge.getID():22s} "
              f"{edge.getName() or '-':10s} {edge.getLaneNumber()}차로 {dist:5.1f}m"
              + ("" if named else "  ※도로명 불일치, GUI로 확인 필요"))

    cx, cy = net.convertLonLat2XY(*AREA_CENTER)
    gates = {}
    for pid, p in COUNT_POINTS.items():
        if not p["gate"]:
            continue
        pair = []
        for d in ("유입", "유출"):
            e = net.getEdge(mapping[f"{pid}/{d}"]["edge"])
            (x1, y1), (x2, y2) = e.getShape()[0], e.getShape()[-1]
            # 도로 끝점이 시작점보다 범위 중심에 가까우면 안으로 들어오는 도로
            closer = math.hypot(x2 - cx, y2 - cy) - math.hypot(x1 - cx, y1 - cy)
            pair.append((closer, e.getID()))
        pair.sort()
        gates[p["gate"]] = dict(point=pid, name=p["name"], **{"in": pair[0][1], "out": pair[1][1]})
    MAPPING.write_text(json.dumps(dict(counts=mapping, gates=gates), ensure_ascii=False, indent=1))
    print(f"[map] → {MAPPING}")
    for g, v in gates.items():
        print(f"   게이트 {g} {v['name']:12s} in={v['in']}  out={v['out']}")


if __name__ == "__main__":
    main()
