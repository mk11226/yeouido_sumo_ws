"""SUMO 도로망(env/yeouido/yeouido.net.xml)을 위도·경도 GeoJSON으로 바꾼다 (검수 지도용).

    python -m inspection.export_net

결과: inspection/out/net.geojson
  - lane     : 차로 중심선 (누르면 도로 ID·차로 수·제한속도 등), 차로 끝 회전 방향·화살표 위치
  - area     : 차로 면 (실제 폭의 다각형, 구간 화면의 SUMO 그림용)
  - mark     : 차로 경계선 (바깥 = 실선, 차로 사이 = 점선) → 항공사진의 차선과 비교
  - conn     : 교차로 안 연결 경로 (어느 차로에서 어느 차로로, 직진·좌회전·우회전)
  - junction : 교차로 영역 (신호 교차로 여부 포함)
  - nl_link  : 국가표준노드링크 원본 링크 (도로망을 만든 원본, region.json) → SUMO와 겹쳐
               변환 중 생긴 차이와 원본 자체가 실제와 다른 곳을 구분
  - nl_node  : 국가표준노드링크 원본 노드
SUMO 좌표(x, y)도 같이 넣어 sumo-gui에서 같은 위치를 찾을 수 있게 한다.
"""
import argparse
import json
import math
from pathlib import Path

import sumolib

from yeouido.config import NET
# edge type = "nl.<도로 등급>.<도로 유형>" (yeouido/build_net.py)
from yeouido.nodelink import NODE_TYPE, REGION, ROAD_RANK, ROAD_TYPE

OUT = Path(__file__).resolve().parent / "out"
GEOJSON = OUT / "net.geojson"

# SUMO 연결 방향 (connection dir)
DIRECTION = {"s": "직진", "r": "우회전", "R": "약간 우회전", "l": "좌회전", "L": "약간 좌회전",
             "t": "U턴", "T": "U턴"}


def offset(shape, d):
    """선을 왼쪽으로 d m 평행 이동 (d < 0 이면 오른쪽). 꼭짓점마다 앞뒤 방향의 평균 법선을 쓴다."""
    if len(shape) < 2:
        return list(shape)
    dirs = []
    for (x0, y0), (x1, y1) in zip(shape, shape[1:]):
        n = math.hypot(x1 - x0, y1 - y0) or 1.0
        dirs.append(((x1 - x0) / n, (y1 - y0) / n))
    out = []
    for i, (x, y) in enumerate(shape):
        a = dirs[max(i - 1, 0)]
        b = dirs[min(i, len(dirs) - 1)]
        tx, ty = a[0] + b[0], a[1] + b[1]
        n = math.hypot(tx, ty)
        if n < 1e-6:            # 되돌아가는 꼭짓점: 앞 방향만 사용
            tx, ty, n = b[0], b[1], 1.0
        tx, ty = tx / n, ty / n
        # 꺾인 곳에서 차로 폭이 줄지 않도록 보정 (너무 뾰족하면 2배로 제한)
        scale = min(1.0 / max(abs(tx * b[0] + ty * b[1]), 1e-6), 2.0)
        out.append((x - ty * d * scale, y + tx * d * scale))
    return out


def nodelink_features(sumo_edges):
    """region.json 의 링크·노드. 링크마다 SUMO에 들어간 도로 ID를 붙인다
    (경계에서 잘린 링크는 SUMO에서 "<링크>_0", "<링크>_1" … 로 나뉜다)."""
    if not REGION.exists():
        print(f"[export_net] {REGION} 이 없어 노드링크 레이어는 건너뜁니다")
        return []
    region = json.loads(REGION.read_text())
    pieces = {}
    for eid in sumo_edges:
        pieces.setdefault(eid.split("_")[0], []).append(eid)
    feats, used_nodes = [], set()
    for l in region["links"]:
        sumo = sorted(pieces.get(l["id"], []))
        if sumo:
            used_nodes.update((l["from_node"], l["to_node"]))
        feats.append(dict(type="Feature",
                          geometry=dict(type="LineString",
                                        coordinates=[[round(lo, 7), round(la, 7)]
                                                     for lo, la in l["shape"]]),
                          properties=dict(kind="nl_link", id=l["id"], name=l["name"] or "",
                                          lanes=l["lanes"], speed_kmh=l["max_speed"],
                                          rank=ROAD_RANK.get(l["rank"], l["rank"]),
                                          level=ROAD_TYPE.get(l["type"], l["type"]),
                                          length=l["length"], from_node=l["from_node"],
                                          to_node=l["to_node"], sumo=sumo)))
    for nid, n in region["nodes"].items():
        feats.append(dict(type="Feature",
                          geometry=dict(type="Point",
                                        coordinates=[round(n["lon"], 7), round(n["lat"], 7)]),
                          properties=dict(kind="nl_node", id=nid, name=n["name"] or "",
                                          node_type=NODE_TYPE.get(n["type"], n["type"]),
                                          in_sumo=nid in used_nodes)))
    return feats


def via_shape(net, conn):
    """교차로 안 연결 경로 전체 모양. 좌회전처럼 교차로 중간에 대기 지점이 있으면 내부 차로가
    두 조각으로 나뉘고, 앞 조각의 연결(via)이 뒤 조각을 가리킨다."""
    shape, via = [], conn.getViaLaneID()
    while via:
        lane = net.getLane(via)
        shape += lane.getShape()[1 if shape else 0:]
        nxt = lane.getOutgoing()
        via = nxt[0].getViaLaneID() if nxt else None
    return shape


def point_before_end(shape, d):
    """선의 끝에서 d m 앞의 점과 그 위치의 진행 방향 (북쪽 0°, 시계 방향)."""
    for (x0, y0), (x1, y1) in zip(reversed(shape[:-1]), reversed(shape[1:])):
        seg = math.hypot(x1 - x0, y1 - y0)
        heading = math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360
        if seg >= d:
            t = 1 - d / seg if seg else 0
            return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t), heading
        d -= seg
    (x0, y0), (x1, y1) = shape[0], shape[1]
    return (x0, y0), math.degrees(math.atan2(x1 - x0, y1 - y0)) % 360


def export(net_file=NET, out=GEOJSON):
    net = sumolib.net.readNet(str(net_file), withInternal=True)
    edges = [e for e in net.getEdges() if e.getFunction() != "internal"]

    def lonlat(shape):
        return [[round(v, 7) for v in net.convertXY2LonLat(x, y)] for x, y in shape]

    def mid(shape):
        x, y = shape[len(shape) // 2]
        return [round(x, 2), round(y, 2)]

    feats = []
    for e in edges:
        _, rank, level = ((e.getType() or "") + "..").split(".")[:3]
        lanes = e.getLanes()
        common = dict(edge=e.getID(), name=e.getName() or "", lanes=len(lanes),
                      rank=ROAD_RANK.get(rank, rank), level=ROAD_TYPE.get(level, level),
                      from_node=e.getFromNode().getID(), to_node=e.getToNode().getID())
        for ln in lanes:
            shape = ln.getShape()
            # 차로 끝 회전 화살표 (sumo-gui 처럼 정지선 앞 6 m, 노면 표시와 비교)
            moves = sorted({DIRECTION.get(c.getDirection(), c.getDirection())
                            for c in ln.getOutgoing()})
            arrow_xy, arrow_heading = point_before_end(shape, min(6.0, ln.getLength() / 2))
            feats.append(dict(type="Feature",
                              geometry=dict(type="LineString", coordinates=lonlat(shape)),
                              properties=dict(common, kind="lane", id=ln.getID(),
                                              index=ln.getIndex(),
                                              speed_kmh=round(ln.getSpeed() * 3.6),
                                              width=round(ln.getWidth(), 2),
                                              length=round(ln.getLength(), 1),
                                              moves=moves, arrow=lonlat([arrow_xy])[0],
                                              arrow_heading=round(arrow_heading, 1),
                                              xy=mid(shape))))
            # 차로 면 (실제 폭)
            half = ln.getWidth() / 2
            ring = offset(shape, half) + list(reversed(offset(shape, -half)))
            feats.append(dict(type="Feature",
                              geometry=dict(type="Polygon", coordinates=[lonlat(ring + ring[:1])]),
                              properties=dict(kind="area", id=ln.getID(), edge=e.getID())))
            # 차로마다 오른쪽 경계선, 맨 왼쪽 차로는 왼쪽 경계선도 (index 0 = 맨 오른쪽 차로)
            sides = [(-half, "solid" if ln.getIndex() == 0 else "dashed")]
            if ln.getIndex() == len(lanes) - 1:
                sides.append((half, "solid"))
            for d, style in sides:
                feats.append(dict(type="Feature",
                                  geometry=dict(type="LineString",
                                                coordinates=lonlat(offset(shape, d))),
                                  properties=dict(kind="mark", style=style, edge=e.getID())))

    # 교차로 안 연결 경로
    for e in edges:
        for conns in e.getOutgoing().values():
            for c in conns:
                shape = via_shape(net, c)
                if len(shape) < 2:
                    continue
                to = c.getTo()
                feats.append(dict(type="Feature",
                                  geometry=dict(type="LineString", coordinates=lonlat(shape)),
                                  properties=dict(kind="conn", id=c.getViaLaneID(),
                                                  junction=c.getJunction().getID(),
                                                  from_lane=c.getFromLane().getID(),
                                                  to_lane=c.getToLane().getID(),
                                                  from_name=e.getName() or "",
                                                  to_name=to.getName() or "",
                                                  direction=DIRECTION.get(c.getDirection(),
                                                                          c.getDirection()),
                                                  tls=c.getTLSID() or "",
                                                  xy=mid(shape))))

    for n in net.getNodes():
        shape = n.getShape()
        if len(shape) < 3:
            continue
        ring = lonlat(list(shape) + [shape[0]])
        feats.append(dict(type="Feature",
                          geometry=dict(type="Polygon", coordinates=[ring]),
                          properties=dict(kind="junction", id=n.getID(), junction_type=n.getType(),
                                          tls=n.getType().startswith("traffic_light"),
                                          incoming=len(n.getIncoming()),
                                          outgoing=len(n.getOutgoing()),
                                          xy=[round(v, 2) for v in n.getCoord()[:2]])))

    feats += nodelink_features({e.getID() for e in edges})

    loc = net._location
    meta = dict(net=Path(net_file).name, proj=loc["projParameter"],
                net_offset=[float(v) for v in loc["netOffset"].split(",")],
                bounds=[[round(v, 6) for v in net.convertXY2LonLat(*xy)]
                        for xy in net.getBBoxXY()])
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(type="FeatureCollection", meta=meta, features=feats),
                              ensure_ascii=False, separators=(",", ":")))
    kinds = {}
    for f in feats:
        kinds[f["properties"]["kind"]] = kinds.get(f["properties"]["kind"], 0) + 1
    print(f"[export_net] 도로 {len(edges)}개 → {kinds}  ({out}, "
          f"{out.stat().st_size / 1e6:.1f} MB)")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--net", type=Path, default=NET, help="SUMO 도로망 (기본: 여의도)")
    args = ap.parse_args(argv)
    export(args.net)


if __name__ == "__main__":
    main()
