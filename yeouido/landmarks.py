"""AV 출발지·목적지 랜드마크(config.LANDMARKS)의 정확한 위치를 구하고 차도에 연결한다.

    python -m yeouido.landmarks

결과: env/yeouido/landmarks.json
  {이름: {lon, lat, x, y, edge, street, dist_m, osm}}
  - lon/lat : OSM 객체 중심 (건물·공원은 면적 중심, 역은 역 노드 위치)
  - edge    : 그 위치에서 가장 가까운 차도. 지하차도·고가차도 위(타고 내릴 수 없음)와
              막다른 길(다른 곳으로 나갈 수 없음)은 제외하고, 도로망 어디서든 오갈 수 있는 도로만 고른다.
  - pos     : edge 위에서 랜드마크와 가장 가까운 지점 (edge 시작점부터 m). AV가 여기서 출발·도착한다.
  AV 경로는 출발 랜드마크의 edge → 도착 랜드마크의 edge 로 잡는다.
"""
import json
import xml.etree.ElementTree as ET

from yeouido.config import LANDMARKS, NET, OUT, sumo_home
from yeouido.download import context_path

RESULT = OUT / "landmarks.json"
MAX_SNAP = 400.0


def _area_centroid(pts):
    """다각형 면적 중심 (꼭짓점 평균보다 정확). 닫히지 않은 선이면 꼭짓점 평균."""
    if len(pts) < 4 or pts[0] != pts[-1]:
        return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
    a = cx = cy = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        cross = x0 * y1 - x1 * y0
        a += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    a *= 0.5
    return cx / (6 * a), cy / (6 * a)


def locate():
    root = ET.parse(context_path()).getroot()
    nodes = {n.get("id"): (float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")}
    ways = {w.get("id"): [nodes[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in nodes]
            for w in root.iter("way")}
    out = {}
    for name, (kind, osm_id) in LANDMARKS.items():
        key = str(osm_id)
        if kind == "node":
            if key not in nodes:
                raise RuntimeError(f"{name}: OSM node {osm_id} 가 주변 환경 자료에 없습니다")
            out[name] = nodes[key]
        else:
            if not ways.get(key):
                raise RuntimeError(f"{name}: OSM way {osm_id} 가 주변 환경 자료에 없습니다")
            out[name] = _area_centroid(ways[key])
    return out


def two_way_edges(net):
    """도로망의 다른 곳에서 올 수 있고 다른 곳으로 갈 수도 있는 도로 (가장 큰 강연결 요소)."""
    edges = [e for e in net.getEdges() if e.allows("passenger")]
    start = max(edges, key=lambda e: len(e.getOutgoing()) + len(e.getIncoming()))

    def reach(nexts):
        seen, stack = {start}, [start]
        while stack:
            for n in nexts(stack.pop()):
                if n not in seen:
                    seen.add(n)
                    stack.append(n)
        return seen

    return reach(lambda e: e.getOutgoing()) & reach(lambda e: e.getIncoming())


def main():
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET))
    usable = two_way_edges(net)
    result = {}
    for name, (lon, lat) in locate().items():
        x, y = net.convertLonLat2XY(lon, lat)
        best = None
        for edge, dist in net.getNeighboringEdges(x, y, MAX_SNAP):
            etype = edge.getType() or ""
            if (edge not in usable or etype.endswith(".001") or etype.endswith(".002")):
                continue
            if best is None or dist < best[1]:
                best = (edge, dist)
        if best is None:
            raise RuntimeError(f"{name}: 반경 {MAX_SNAP}m 안에 차도가 없습니다")
        kind, osm_id = LANDMARKS[name]
        pos, _ = best[0].getLanes()[0].getClosestLanePosAndDist((x, y))
        result[name] = dict(lon=round(lon, 7), lat=round(lat, 7), x=round(x, 1), y=round(y, 1),
                            edge=best[0].getID(), pos=round(pos, 1), street=best[0].getName(),
                            dist_m=round(best[1], 1), osm=f"{kind}/{osm_id}")
        print(f"   {name:8s} ({lon:.5f}, {lat:.5f}) → {best[0].getName() or '-'} "
              f"[{best[0].getID()}] {best[1]:.0f} m")
    RESULT.write_text(json.dumps(result, ensure_ascii=False, indent=1))
    print(f"[landmarks] {len(result)}곳 → {RESULT}")


if __name__ == "__main__":
    main()
