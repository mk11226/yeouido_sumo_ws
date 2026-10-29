"""도로 밖 주변 환경(건물·공원·한강·토지 이용)을 SUMO 폴리곤으로 만들고,
이름 있는 장소(역·공원·건물·관공서 등)를 가장 가까운 도로에 연결한다.

    python -m yeouido.build_context

결과 (env/yeouido/)
  yeouido.poly.xml : sumo-gui 배경용 폴리곤. 주행·교통 시뮬레이션에는 영향 없음
  places.json      : {장소 이름: 종류, 좌표, 가장 가까운 도로(edge), 거리}
                     → "여의도공원에서 영등포역으로" 같은 AV 출발지/목적지 지정에 쓴다.
"""
import json
import math
import subprocess
import xml.etree.ElementTree as ET

from yeouido.config import NET, OUT, sumo_bin, sumo_home
from yeouido.download import context_path

POLY = OUT / "yeouido.poly.xml"
PLACES = OUT / "places.json"
LAND = "242,239,233"     # 배경(땅) 색. view.py 의 sumo-gui 배경색과 같게 맞춘다
WATER = "170,211,223"
MAX_SNAP = 300.0   # 장소에서 이 거리(m) 안에 차도가 없으면 목적지로 쓰지 않는다

# 목적지 후보로 쓸 장소 종류 (OSM 태그 → 표시 이름). 식당·카페처럼 너무 많은 것은 뺀다.
PLACE_TAGS = [
    (("railway", "station"), "역"),
    (("public_transport", "station"), "역"),
    (("leisure", "park"), "공원"),
    (("shop", "mall"), "쇼핑몰"),
    (("amenity", "university"), "대학교"),
    (("amenity", "college"), "대학교"),
    (("amenity", "school"), "학교"),
    (("amenity", "hospital"), "병원"),
    (("amenity", "townhall"), "관공서"),
    (("office", "government"), "관공서"),
    (("amenity", "police"), "경찰서"),
    (("amenity", "fire_station"), "소방서"),
    (("amenity", "library"), "도서관"),
    (("tourism", "attraction"), "명소"),
    (("tourism", "museum"), "명소"),
    (("tourism", "hotel"), "호텔"),
    (("building", None), "건물"),   # 이름 있는 건물 (IFC, 63빌딩, 국회의사당 등)
]


def build_polygons():
    cmd = [sumo_bin("polyconvert"), "--net-file", str(NET),
           "--osm-files", str(context_path()),
           "--type-file", str(sumo_home() / "data" / "typemap" / "osmPolyconvert.typ.xml"),
           "--osm.merge-relations", "1",       # 한강처럼 여러 조각으로 된 물/공원을 하나로
           "--prune.in-net", "--prune.in-net.offsets", "300,300,300,300",
           "-o", str(POLY)]
    log = OUT / "polyconvert.log"
    with open(log, "w") as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        raise RuntimeError(f"polyconvert 실패 — 로그 확인: {log}")
    # polyconvert는 식당·카페 같은 점(OSM node)도 POI로 넣는데, 수천 개라 화면을 덮는다.
    # 배경은 면(건물·공원·물)만 남기고, 목적지 장소는 places.json 으로 따로 관리한다.
    tree = ET.parse(POLY)
    root = tree.getroot()
    for poi in root.findall("poi"):
        root.remove(poi)
    # 구멍(inner)이 있는 물 multipolygon(한강 등)은 polyconvert가 못 읽고 구멍만 흰색으로 남긴다.
    # 구멍 조각은 지우고, 물은 add_water_multipolygons 에서 직접 만든다.
    for poly in root.findall("poly"):
        if poly.get("type") == "inner":
            root.remove(poly)
    added = add_water_multipolygons(root)
    tree.write(POLY, encoding="utf-8", xml_declaration=True)
    print(f"[context] 한강 등 물 영역 {added}개를 직접 조립해 추가")
    kinds = {}
    for p in root.iter("poly"):
        kinds[p.get("type")] = kinds.get(p.get("type"), 0) + 1
    print(f"[context] 폴리곤 {sum(kinds.values())}개 → {POLY}")
    print("          " + ", ".join(f"{k} {v}" for k, v in
                                    sorted(kinds.items(), key=lambda kv: -kv[1])[:8]))


def _rings(way_ids, ways):
    """끝점이 이어지는 way 조각들을 이어 붙여 닫힌 고리(node id 목록)들로 만든다."""
    pending = [list(ways[w]) for w in way_ids if w in ways]
    rings = []
    while pending:
        ring = pending.pop(0)
        grown = True
        while ring[0] != ring[-1] and grown:
            grown = False
            for i, seg in enumerate(pending):
                if seg[0] == ring[-1]:
                    ring += seg[1:]
                elif seg[-1] == ring[-1]:
                    ring += seg[::-1][1:]
                elif seg[-1] == ring[0]:
                    ring = seg[:-1] + ring
                elif seg[0] == ring[0]:
                    ring = seg[::-1][:-1] + ring
                else:
                    continue
                pending.pop(i)
                grown = True
                break
        if len(ring) >= 4:
            rings.append(ring)
    return rings


def _clip(pts, box):
    """다각형을 사각형 범위로 자른다 (Sutherland–Hodgman). 한강처럼 수십 km 이어지는
    물 영역을 도로망 주변만 남기기 위해 쓴다."""
    x0, y0, x1, y1 = box
    edges = [(lambda p: p[0] >= x0, lambda a, b: (x0, a[1] + (b[1] - a[1]) * (x0 - a[0]) / (b[0] - a[0]))),
             (lambda p: p[0] <= x1, lambda a, b: (x1, a[1] + (b[1] - a[1]) * (x1 - a[0]) / (b[0] - a[0]))),
             (lambda p: p[1] >= y0, lambda a, b: (a[0] + (b[0] - a[0]) * (y0 - a[1]) / (b[1] - a[1]), y0)),
             (lambda p: p[1] <= y1, lambda a, b: (a[0] + (b[0] - a[0]) * (y1 - a[1]) / (b[1] - a[1]), y1))]
    for inside, cross in edges:
        out = []
        for i, cur in enumerate(pts):
            prev = pts[i - 1]
            if inside(cur):
                if not inside(prev):
                    out.append(cross(prev, cur))
                out.append(cur)
            elif inside(prev):
                out.append(cross(prev, cur))
        pts = out
        if not pts:
            break
    return pts


def add_water_multipolygons(root):
    """물 multipolygon 을 SUMO 폴리곤으로 조립한다.

    SUMO 폴리곤에는 구멍이 없으므로, 바깥 고리(outer)를 물색으로 깔고
    안쪽 고리(inner = 여의도·밤섬·선유도 같은 섬)를 땅색으로 그 위에 덮는다.
    """
    import sumolib
    net = sumolib.net.readNet(str(NET))
    osm = ET.parse(context_path()).getroot()
    lonlat = {n.get("id"): (float(n.get("lon")), float(n.get("lat"))) for n in osm.iter("node")}
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in osm.iter("way")}
    existing = {p.get("id") for p in root.iter("poly")}
    (bx0, by0), (bx1, by1) = net.getBBoxXY()
    box = (bx0 - 300, by0 - 300, bx1 + 300, by1 + 300)
    added = 0
    for rel in osm.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("natural") != "water" or rel.get("id") in existing:
            continue
        for role, color, layer, kind in (("outer", WATER, "-5", "water"),
                                         ("inner", LAND, "-4.5", "island")):
            ids = [m.get("ref") for m in rel.findall("member")
                   if m.get("type") == "way" and m.get("role") == role]
            for k, ring in enumerate(_rings(ids, ways)):
                pts = _clip([net.convertLonLat2XY(*lonlat[n]) for n in ring if n in lonlat], box)
                if len(pts) < 3:
                    continue
                shape = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
                ET.SubElement(root, "poly", {
                    "id": f"{kind}_{rel.get('id')}_{k}", "type": kind, "color": color,
                    "fill": "1", "layer": layer, "shape": shape})
                added += 1
    return added


def _category(tags):
    for (k, v), label in PLACE_TAGS:
        if k in tags and (v is None or tags[k] == v):
            return label
    return None


def collect_places(net):
    root = ET.parse(context_path()).getroot()
    nodes = {n.get("id"): (float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")}
    found = []
    for el in root:
        if el.tag not in ("node", "way"):
            continue
        tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
        name = tags.get("name")
        cat = _category(tags)
        if not name or not cat:
            continue
        if el.tag == "node":
            lon, lat = nodes[el.get("id")]
        else:
            pts = [nodes[nd.get("ref")] for nd in el.findall("nd") if nd.get("ref") in nodes]
            if not pts:
                continue
            lon = sum(p[0] for p in pts) / len(pts)
            lat = sum(p[1] for p in pts) / len(pts)
        found.append((name, cat, lon, lat))

    places = {}
    order = {label: i for i, (_, label) in enumerate(PLACE_TAGS)}
    for name, cat, lon, lat in sorted(found, key=lambda f: order[f[1]]):
        if name in places:   # 같은 이름은 더 중요한 종류(역 > 공원 > ... > 건물) 하나만
            continue
        x, y = net.convertLonLat2XY(lon, lat)
        best = None
        for edge, dist in net.getNeighboringEdges(x, y, MAX_SNAP):
            if edge.getFunction() == "internal" or not edge.allows("passenger"):
                continue
            if best is None or dist < best[1]:
                best = (edge, dist)
        if best is None:
            continue
        places[name] = dict(category=cat, lon=round(lon, 6), lat=round(lat, 6),
                            edge=best[0].getID(), street=best[0].getName(),
                            dist_m=round(best[1], 1))
    return places


def main():
    sumo_home()
    import sumolib
    build_polygons()
    net = sumolib.net.readNet(str(NET))
    places = collect_places(net)
    PLACES.write_text(json.dumps(places, ensure_ascii=False, indent=1))
    by_cat = {}
    for p in places.values():
        by_cat[p["category"]] = by_cat.get(p["category"], 0) + 1
    print(f"[context] 목적지로 쓸 수 있는 장소 {len(places)}곳 → {PLACES}")
    print("          " + ", ".join(f"{k} {v}" for k, v in by_cat.items()))


if __name__ == "__main__":
    main()
