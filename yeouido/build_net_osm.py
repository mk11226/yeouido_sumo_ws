"""(이전 방식, 참고용) OSM 원본 → SUMO 도로망. 현재는 build_net.py(국가표준노드링크)를 쓴다.

    python -m yeouido.build_net_osm

netconvert 옵션은 실제 도로를 SUMO에서 안정적으로 돌리기 위한 설정이다.
  - 범위(BBOX) 밖 도로는 잘라낸다(clip_osm). 다리는 강 중간에서 잘려 입구가 된다.
  - OSM의 차로 수·회전 차로·제한속도를 읽는다. 차로 수가 없으면 같은 도로의
    다른 구간 값을(fill_lanes), 그것도 없으면 서울 도심 기본값(SEOUL_DEFAULTS)을 쓴다.
  - 가까운 교차점을 하나로 합치고(junctions.join), 신호를 정리한다(tls.*).
  - 서로 이어지지 않은 작은 조각 도로는 버리고 가장 큰 연결망 하나만 남긴다.
"""
import subprocess
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

from yeouido.config import (BBOX, EXCLUDED_CLASSES, NET, OUT, SOURCE, south_of_river,
                            sumo_bin, sumo_home)
from yeouido.download import osm_path

CLIPPED = SOURCE / "yeouido.clipped.osm.xml"
NO_JOIN = OUT / "no_join_nodes.txt"
TYPEMAP = OUT / "seoul.typ.xml"

# OSM에 차로 수·제한속도가 없을 때 쓸 서울 도심 기본값 (방향당 차로 수, km/h).
# SUMO 기본값(보조간선 1차로, 주간선 100km/h)은 서울 도로보다 훨씬 좁고 빨라서
# 출퇴근 교통량을 넣으면 도로가 감당하지 못한다. 제한속도는 안전속도 5030 기준.
SEOUL_DEFAULTS = {
    "trunk": (3, 80), "primary": (3, 50), "secondary": (2, 50), "tertiary": (2, 50),
    "trunk_link": (1, 60), "primary_link": (1, 40),
    "secondary_link": (1, 40), "tertiary_link": (1, 40),
}


def write_typemap():
    """SUMO 기본 OSM 타입맵을 복사해 SEOUL_DEFAULTS 값으로 바꿔 쓴다."""
    root = ET.parse(sumo_home() / "data" / "typemap" / "osmNetconvert.typ.xml").getroot()
    for t in root.iter("type"):
        cls = t.get("id", "").replace("highway.", "")
        if t.get("id", "").startswith("highway.") and cls in SEOUL_DEFAULTS:
            lanes, kmh = SEOUL_DEFAULTS[cls]
            t.set("numLanes", str(lanes))
            t.set("speed", f"{kmh / 3.6:.2f}")
    ET.ElementTree(root).write(TYPEMAP, encoding="utf-8", xml_declaration=True)


def fill_lanes(root):
    """차로 수가 빠진 도로에, 같은 이름·같은 일방통행 여부인 다른 구간의 값을 채운다.

    예: 여의동로 일부 구간에만 lanes=4가 있으면 나머지 여의동로 구간에도 4를 쓴다.
    같은 도로의 다른 구간에도 값이 없으면 SEOUL_DEFAULTS 로 넘어간다.
    """
    seen = defaultdict(Counter)
    ways = []
    for w in root.iter("way"):
        tags = {t.get("k"): t for t in w.findall("tag")}
        if "name" not in tags or "highway" not in tags:
            continue
        key = (tags["name"].get("v"), tags.get("oneway") is not None
               and tags["oneway"].get("v") == "yes")
        ways.append((w, tags, key))
        if "lanes" in tags:
            seen[key][tags["lanes"].get("v")] += 1
    filled = 0
    for w, tags, key in ways:
        if "lanes" not in tags and seen[key]:
            ET.SubElement(w, "tag", {"k": "lanes", "v": seen[key].most_common(1)[0][0]})
            filled += 1
    return filled


def write_no_join(root):
    """지하차도·고가(tunnel/bridge, layer≠0) 도로의 점 목록을 쓴다.

    netconvert의 junctions.join 은 가까운 교차점을 하나로 합치는데, 지하차도와 그 위
    지상 도로의 점까지 합쳐 실제로는 만나지 않는 두 도로를 평면 교차로로 만들었다
    (여의교 남단 노들로·대방지하차도). 이 점들은 합치기에서 제외한다.
    다리(한강 교량)는 강 위라 겹치는 지상 도로가 없으므로 layer 가 있을 때만 제외한다.
    """
    ids = set()
    for w in root.iter("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        layer = tags.get("layer", "0")
        if tags.get("tunnel") in ("yes", "building_passage") or layer not in ("0", ""):
            ids.update(nd.get("ref") for nd in w.findall("nd"))
    NO_JOIN.write_text(",".join(sorted(ids)))


def clip_osm():
    """범위(BBOX, 한강 남쪽) 밖 점을 지우고, 범위를 벗어나는 도로는 경계에서 잘라 조각으로 나눈다.

    netconvert의 범위 옵션은 범위에 "걸친" 도로를 통째로 남기므로
    (예: 신월여의지하도로가 서쪽으로 수 km 이어짐) 여기서 직접 자른다.
    """
    b = BBOX

    def is_in(lon, lat):
        return (b["west"] <= lon <= b["east"] and b["south"] <= lat <= b["north"]
                and south_of_river(lon, lat))

    root = ET.parse(osm_path()).getroot()
    pos = {n.get("id"): (float(n.get("lon")), float(n.get("lat"))) for n in root.findall("node")}
    inside = {i for i, (lon, lat) in pos.items() if is_in(lon, lat)}
    used = set()
    new_id = [9_000_000_000_000]

    def boundary_node(a, c):
        """a(안)–c(밖) 사이에서 경계와 만나는 점을 이분법으로 찾아 새 점으로 추가한다.
        다리처럼 점이 듬성듬성한 긴 구간도 경계까지 남게 된다."""
        (x0, y0), (x1, y1) = pos[a], pos[c]
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = (lo + hi) / 2
            if is_in(x0 + (x1 - x0) * mid, y0 + (y1 - y0) * mid):
                lo = mid
            else:
                hi = mid
        new_id[0] += 1
        nid = str(new_id[0])
        pos[nid] = (x0 + (x1 - x0) * lo, y0 + (y1 - y0) * lo)
        ET.SubElement(root, "node", {"id": nid, "version": "1",
                                     "lon": f"{pos[nid][0]:.7f}", "lat": f"{pos[nid][1]:.7f}"})
        return nid

    for w in root.findall("way"):
        refs = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in pos]
        tags = w.findall("tag")
        root.remove(w)
        if any(t.get("k") == "highway" and t.get("v") in EXCLUDED_CLASSES for t in tags):
            continue
        # 범위 안 점이 연속된 구간마다 별도 도로로 만들고, 경계를 넘는 곳에는 경계점을 붙인다
        pieces, cur = [], []
        for i, r in enumerate(refs):
            if r in inside:
                if not cur and i > 0:
                    cur.append(boundary_node(r, refs[i - 1]))
                cur.append(r)
            elif cur:
                cur.append(boundary_node(refs[i - 1], r))
                pieces.append(cur)
                cur = []
        pieces.append(cur)
        for k, piece in enumerate(pc for pc in pieces if len(pc) >= 2):
            nw = ET.SubElement(root, "way", {"id": str(int(w.get("id")) * 100 + k),
                                             "version": "1"})
            for r in piece:
                ET.SubElement(nw, "nd", {"ref": r})
                used.add(r)
            for t in tags:
                nw.append(t)
    for n in root.findall("node"):
        if n.get("id") not in used and n.get("id") not in inside:
            root.remove(n)
    for rel in root.findall("relation"):
        root.remove(rel)
    # OSM 파일은 점(node)이 도로(way)보다 앞에 있어야 netconvert가 읽는다
    order = {"node": 1, "way": 2}
    root[:] = sorted(root, key=lambda el: order.get(el.tag, 0))
    filled = fill_lanes(root)
    write_no_join(root)
    ET.ElementTree(root).write(CLIPPED, encoding="utf-8", xml_declaration=True)
    return filled


def netconvert_cmd():
    return [
        sumo_bin("netconvert"),
        "--osm-files", str(CLIPPED),
        "--type-files", str(TYPEMAP),
        "-o", str(NET),
        # 차량 도로만, 서로 이어진 가장 큰 도로망 하나만 남기기
        "--keep-edges.by-vclass", "passenger",
        "--keep-edges.components", "1",
        "--remove-edges.isolated",
        # OSM 속성 읽기
        "--osm.turn-lanes", "--osm.lane-access", "--osm.annotate-defaults",
        "--output.street-names",
        # 교차로·램프 정리
        "--geometry.remove", "--junctions.join", "--junctions.join-dist", "20",
        "--junctions.join-exclude", NO_JOIN.read_text(), "--junctions.corner-detail", "5",
        "--ramps.guess", "--roundabouts.guess", "--no-turnarounds",
        # 신호: OSM에 있는 신호를 쓰고, 가까운 신호는 하나로 묶는다
        # 한 교차로가 여러 신호로 쪼개진 곳(여의대방로×여의동로 등)은 60 m 안이면 하나로 묶어
        # 같은 신호 계획으로 돌린다
        "--tls.guess-signals", "--tls.join", "--tls.join-dist", "60", *TLS_OPTIONS,
    ]


# 신호 공통 설정. 실제 신호 시간 계획은 공개되어 있지 않아 감응식으로 추정한다.
# all-red 2초: 노란불 끝에 교차로 안에 남은 차가 빠져나간 뒤 다음 방향에 녹색을 줘서
# 교차로 안 충돌을 막는다.
TLS_OPTIONS = ["--tls.default-type", "actuated", "--tls.cycle.time", "90",
               "--tls.yellow.time", "3", "--tls.allred.time", "2"]


MAJOR = ("trunk", "primary", "secondary")


def missing_signal_nodes(net):
    """OSM에 신호 표시가 빠진 간선도로 교차로를 찾는다.

    들어오는 도로가 3개 이상이고 그중 2개 이상이 간선(trunk/primary/secondary)인데
    신호가 없는 교차로. 이런 곳은 출퇴근 교통량에서 양보 교착(teleport)이 생긴다.
    다리·램프의 단순 합류(들어오는 도로 2개)는 신호 없이 둔다.
    """
    nodes = []
    for n in net.getNodes():
        if n.getType() == "traffic_light":
            continue
        inc = [e for e in n.getIncoming() if e.getFunction() != "internal"]
        major = [e for e in inc if any(k in (e.getType() or "") for k in MAJOR)]
        if len(inc) >= 3 and len(major) >= 2:
            nodes.append(n.getID())
    return nodes


def merge_nodes(net):
    """신호 없는 단순 합류 지점(여러 도로 → 도로 하나)을 찾는다.

    SUMO 기본 우선순위 합류에서는 정체 중 양쪽 차가 같은 차로로 조금씩 밀고 들어가다
    접촉 사고가 났다. 이런 곳은 zipper(지퍼식 교대 합류) 교차로로 바꾼다.
    """
    nodes = []
    for n in net.getNodes():
        if n.getType() not in ("priority", "right_before_left"):
            continue
        inc = [e for e in n.getIncoming() if e.getFunction() != "internal"]
        out = [e for e in n.getOutgoing() if e.getFunction() != "internal"]
        if len(inc) >= 2 and len(out) == 1:
            nodes.append(n.getID())
    return nodes


def _run(cmd, log):
    with open(log, "a") as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        raise RuntimeError(f"netconvert 실패 — 로그 확인: {log}")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    filled = clip_osm()
    write_typemap()
    print(f"[net] 차로 수가 빠진 도로 {filled}개를 같은 도로의 다른 구간 값으로 채움")
    log = OUT / "netconvert.log"
    log.write_text("")
    _run(netconvert_cmd(), log)

    import sumolib
    # 2차: 같은 도로망을 다시 읽어(ID 유지) 교차로를 보정한다
    #   - 신호가 빠진 간선 교차로 → 신호 추가
    #   - 단순 합류 지점 → zipper 합류
    first = sumolib.net.readNet(str(NET))
    added, merges = missing_signal_nodes(first), merge_nodes(first)
    patch = OUT / "junction_fixes.nod.xml"
    patch.write_text("<nodes>\n" + "".join(f'    <node id="{n}" type="zipper"/>\n' for n in merges)
                     + "</nodes>\n")
    cmd = [sumo_bin("netconvert"), "-s", str(NET), "-n", str(patch), "-o", str(NET), *TLS_OPTIONS]
    if added:
        cmd += ["--tls.set", ",".join(added)]
    _run(cmd, log)
    net = sumolib.net.readNet(str(NET))
    edges = [e for e in net.getEdges() if e.getFunction() != "internal"]
    km = sum(e.getLength() for e in edges) / 1000
    print(f"[net] {NET}")
    print(f"      edge {len(edges)}개, 총 {km:.1f} km, 교차로 {len(net.getNodes())}개, "
          f"신호 {len(net.getTrafficLights())}개 (그중 추정 추가 {len(added)}개), "
          f"zipper 합류 {len(merges)}곳 (경고는 {log})")


if __name__ == "__main__":
    main()
