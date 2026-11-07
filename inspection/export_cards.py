"""검수 카드 데이터: 교차로 카드와 도로 구간 카드.

    python -m inspection.export_cards

결과: inspection/out/cards.json  (inspection/cards.html 이 읽는다)

위치를 겹쳐 보는 대신 구조를 비교하기 위한 자료다.
교차로 카드 (cards)
  - 진입로마다 차로 수, 차로별로 갈 수 있는 방향(좌·직·우·U턴)과 도착 도로
  - 나가는 도로
  - 자동 점검 힌트: 노드링크 회전 제한과 다른 연결, 갈 곳 없는 차로, netconvert 경고
  - 검수 순서: AV 랜드마크 경로 위 → 신호 교차로 → 나머지
도로 구간 카드 (roads) — 교차로와 교차로 사이 한 방향 도로 (SUMO edge)
  - 차로 수·폭·제한속도·버스 전용, 차로 끝 방향, 시작·끝 교차로, 반대 방향 도로
  - 노드링크 원본 링크 값(차로 수·제한속도)과 비교
  - 자동 점검 힌트: 원본과 다른 값, 갈 곳 없는 차로, 다음 구간과 차로 수 차이, netconvert 경고
  - 검수 순서: AV 랜드마크 경로 위 → 나머지
화면 보정 이동량 (둘 다): 노드링크 원본은 실제보다 수~수십 m 밀려 있어 항공사진과 어긋난다.
  좁은 범위에서는 거의 한 방향으로 밀린 형태라, 주변 SUMO 도로선을 항공사진과 잘 맞는
  OSM 도로선에 가장 잘 겹치게 하는 평행 이동량을 카드마다 구한다 (도로망 파일은 바꾸지 않음).
"""
import argparse
import itertools
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import xml.etree.ElementTree as ET

import numpy as np
import sumolib

from inspection.export_net import DIRECTION, OUT, via_shape
from yeouido.config import NET, OUT as YEOUIDO_OUT, SOURCE
from yeouido.landmarks import RESULT as LANDMARKS
from yeouido.nodelink import REGION, ROAD_RANK, ROAD_TYPE, TURN_TYPE

CARDS = OUT / "cards.json"
NETCONVERT_LOG = YEOUIDO_OUT / "netconvert.log"
OSM = SOURCE / "yeouido.osm.xml"
CLIP_M = 60          # 지도에 그릴 진입·진출 차로 길이 (교차로에서부터)
ALIGN_R = 100        # 보정 이동량을 구할 때 쓰는 교차로 주변 반경 (m)
ALIGN_MAX = 35       # 이보다 크게 옮겨야 맞는 결과는 믿지 않는다 (m)
FORBIDDEN = {"003", "101", "102", "103"}   # 회전금지, 좌회전·직진·우회전 금지

# netconvert 경고 → 쉬운 설명
WARNINGS = [
    (r"Intersecting left turns", "좌회전 경로끼리 교차로 안에서 겹침"),
    (r"does not control any links", "신호가 제어할 연결이 없어 신호가 만들어지지 않음"),
    (r"Could not build program", "신호 프로그램을 만들지 못함"),
    (r"Unreachable edge", "신호 안에서 도달할 수 없는 도로가 있음"),
    (r"Minor green .* exceeds", "비보호 좌회전 속도가 높음 (좌회전 차로 누락 의심)"),
    (r"Speed of straight connection", "직진 연결이 급하게 꺾여 속도가 줄어듦"),
    (r"Shape for junction .* distance ([\d.]+)", "교차로 모양이 원래 위치에서 {0} m 떨어짐"),
    (r"sharp turn", "도로 끝이 급하게 꺾임"),
    (r"not connected to outgoing edges", "나가는 도로와 연결되지 않은 도로가 있음"),
]


def heading(p, q):
    """p → q 방향 (북쪽 0°, 시계 방향)."""
    return round(math.degrees(math.atan2(q[0] - p[0], q[1] - p[1])) % 360, 1)


def clip(shape, length, from_end):
    """선의 한쪽 끝에서 length m 만큼만 남긴다."""
    pts = list(reversed(shape)) if from_end else list(shape)
    out, left = [pts[0]], length
    for a, b in zip(pts, pts[1:]):
        d = math.dist(a, b)
        if d >= left:
            t = left / d if d else 0
            out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            break
        out.append(b)
        left -= d
    return list(reversed(out)) if from_end else out


def landmark_routes(net):
    """랜드마크 두 곳 사이 최단 경로가 지나는 교차로·도로 → 지나는 경로 수."""
    nodes, edges = defaultdict(int), defaultdict(int)
    if not LANDMARKS.exists():
        return nodes, edges
    marks = json.loads(LANDMARKS.read_text())
    for a, b in itertools.permutations(marks, 2):
        route, _ = net.getShortestPath(net.getEdge(marks[a]["edge"]), net.getEdge(marks[b]["edge"]),
                                       vClass="passenger")
        route = [e for e in route or [] if e.getFunction() != "internal"]
        for e in route:
            edges[e.getID()] += 1
        for e in route[:-1]:
            nodes[e.getToNode().getID()] += 1
    return nodes, edges


def netconvert_hints(net):
    """netconvert 경고를 관련 교차로별·도로별로 모은다 (경고에 나온 교차로·도로·차로·신호 ID로 찾음)."""
    hints, edge_hints = defaultdict(list), defaultdict(list)
    if not NETCONVERT_LOG.exists():
        return hints, edge_hints
    tls_nodes = defaultdict(set)
    for n in net.getNodes():
        for c in n.getConnections():
            if c.getTLSID():
                tls_nodes[c.getTLSID()].add(n.getID())
    seen = set()
    for line in NETCONVERT_LOG.read_text().splitlines():
        if not line.startswith("Warning:") or "total messages" in line or line in seen:
            continue
        seen.add(line)
        text = None
        for pattern, desc in WARNINGS:
            m = re.search(pattern, line)
            if m:
                text = desc.format(*m.groups())
                break
        nodes, edges = set(), set()
        for ident in re.findall(r"'([^']+)'", line):
            if net.hasNode(ident):
                nodes.add(ident)
            nodes |= tls_nodes.get(ident, set())
            ident = ident.split("->")[0]          # 연결 "차로->차로" 는 출발 차로로
            for cand in (ident, ident.rsplit("_", 1)[0]):   # 도로 ID, 또는 차로 ID의 도로
                if net.hasEdge(cand) and net.getEdge(cand).getFunction() != "internal":
                    e = net.getEdge(cand)
                    edges.add(cand)
                    # 도로 끝(end)이면 도착 교차로, 시작(start)이면 출발 교차로
                    nodes.add((e.getFromNode() if "start of edge" in line else e.getToNode()).getID())
                    break
        hint = dict(level="warn", text=text or "netconvert 경고", detail=line.removeprefix("Warning: "))
        for nid in nodes:
            hints[nid].append(hint)
        for eid in edges:
            edge_hints[eid].append(hint)
    return hints, edge_hints


def turn_hints(net):
    """노드링크 회전 제한(TURNINFO)과 SUMO 연결을 비교한다."""
    hints, rules = defaultdict(list), defaultdict(list)
    if not REGION.exists():
        return hints, rules
    from yeouido import corrections
    try:   # 수정 목록에서 지운 회전 제한은 "지움" 으로 표시하고 오류 힌트를 내지 않는다
        removed = {(corrections.link_id(c["from"]), corrections.link_id(c["to"]))
                   for c in corrections.load()["remove_turns"]}
    except ValueError:
        removed = set()
    pieces = defaultdict(list)
    for e in net.getEdges():
        pieces[e.getID().split("_")[0]].append(e)
    for t in json.loads(REGION.read_text())["turns"]:
        pairs = [(a, b) for a in pieces.get(t["from_link"], []) for b in pieces.get(t["to_link"], [])
                 if a.getToNode() == b.getFromNode()]
        if not pairs:
            continue
        a, b = pairs[0]
        node = a.getToNode().getID()
        connected = bool(a.getConnections(b))
        name = TURN_TYPE.get(t["type"], t["type"])
        what = f"{a.getName() or a.getID()} → {b.getName() or b.getID()}"
        gone = (t["from_link"], t["to_link"]) in removed
        rules[(node, a.getID())].append(dict(to=b.getID(), to_name=b.getName() or "", rule=name,
                                             connected=connected, all_day=t.get("all_day", True), removed=gone))
        if gone:
            continue
        if t["type"] in FORBIDDEN and connected:
            hints[node].append(dict(level="bad", text=f"노드링크는 {name}인데 SUMO에 연결이 있음",
                                    detail=what))
        elif t["type"] == "011" and not connected:
            # build_net.py 가 --no-turnarounds 로 U턴을 모두 뺐다 → 오류가 아니라 참고
            hints[node].append(dict(level="info", text="노드링크 U턴 허용 지점 (도로망은 U턴을 모두 뺌)",
                                    detail=what))
    return hints, rules


def osm_segments(net):
    """항공사진과 비교할 OSM 지상 도로선 (지하·2층 이상 고가 제외) → SUMO 좌표 선분 (a, b)."""
    if not OSM.exists():
        return None
    root = ET.parse(OSM).getroot()
    nodes = {n.get("id"): net.convertLonLat2XY(float(n.get("lon")), float(n.get("lat")))
             for n in root.iter("node")}
    a, b = [], []
    for w in root.iter("way"):
        tag = {t.get("k"): t.get("v") for t in w.iter("tag")}
        if "highway" not in tag or tag.get("tunnel") == "yes":
            continue
        try:
            if int(tag.get("layer", 0)) < 0 or int(tag.get("layer", 0)) >= 2:
                continue
        except ValueError:
            continue
        pts = [nodes[r.get("ref")] for r in w.iter("nd") if r.get("ref") in nodes]
        a += pts[:-1]
        b += pts[1:]
    return np.array(a), np.array(b)


def sample_edges(net, center, edges, r=ALIGN_R):
    """반경 r 안의 SUMO 지상·교량 도로 중심선을 4 m 간격 점으로."""
    pts = []
    for e in edges:
        level = ((e.getType() or "") + "..").split(".")[2]
        if level not in ("000", "003"):     # 지상, 교량만 (지하·고가는 사진에서 다르게 보임)
            continue
        shape = e.getShape()
        for (x0, y0), (x1, y1) in zip(shape, shape[1:]):
            n = max(int(math.dist((x0, y0), (x1, y1)) / 4), 1)
            for k in range(n):
                p = (x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
                if math.dist(p, center) <= r:
                    pts.append(p)
    return np.array(pts)


def nearest_on_segments(p, a, b):
    """점마다 가장 가까운 선분 위의 점."""
    ab = b - a
    t = ((p[:, None, :] - a[None]) * ab[None]).sum(-1) / np.maximum((ab ** 2).sum(-1), 1e-9)[None]
    q = a[None] + np.clip(t, 0, 1)[..., None] * ab[None]
    d = np.linalg.norm(q - p[:, None, :], axis=-1)
    i = d.argmin(1)
    return q[np.arange(len(p)), i], d[np.arange(len(p)), i]


def align(center, pts, segs, r=ALIGN_R):
    """pts 를 OSM 선분에 맞추는 평행 이동 (ICP, 이동만). 실패하면 None."""
    if segs is None or len(pts) < 15:
        return None
    a, b = segs
    near = (np.minimum(np.linalg.norm(a - center, axis=1), np.linalg.norm(b - center, axis=1))
            < r + ALIGN_MAX + 30)
    a, b = a[near], b[near]
    if len(a) < 3:
        return None
    t = np.zeros(2)
    for it in range(30):
        q, d = nearest_on_segments(pts + t, a, b)
        limit = max(30 - it * 2, 8)                  # 처음엔 넓게, 점점 가까운 짝만
        keep = d < min(limit, max(2.5 * np.median(d), 3))
        if keep.sum() < 10:
            return None
        step = np.median(q[keep] - (pts + t)[keep], axis=0)
        t += step
        if np.linalg.norm(step) < 0.05 and it > 5:
            break
    q, d = nearest_on_segments(pts + t, a, b)
    used = d < 6
    if np.linalg.norm(t) > ALIGN_MAX or used.mean() < 0.35 or used.sum() < 20:
        return None
    return dict(dx=round(float(t[0]), 2), dy=round(float(t[1]), 2),
                fit=round(float(used.mean()), 2), resid=round(float(np.median(d)), 2))


def export(net_file=NET, out=CARDS):
    net = sumolib.net.readNet(str(net_file), withInternal=True)

    def lonlat(shape):
        return [[round(v, 6) for v in net.convertXY2LonLat(x, y)] for x, y in shape]

    on_route, edge_routes = landmark_routes(net)
    warn, edge_warn = netconvert_hints(net)
    turn_warn, rules = turn_hints(net)
    segs = osm_segments(net)
    plain = [e for e in net.getEdges() if e.getFunction() != "internal"]

    cards = []
    for n in net.getNodes():
        incoming = [e for e in n.getIncoming() if e.getFunction() != "internal"]
        outgoing = [e for e in n.getOutgoing() if e.getFunction() != "internal"]
        if n.getType() in ("dead_end", "internal") or (len(incoming) < 2 and len(outgoing) < 2):
            continue
        hints = warn.get(n.getID(), []) + turn_warn.get(n.getID(), [])
        geo = dict(conns=[], lanes=[])
        approaches = []
        for e in incoming:
            lanes = []
            for ln in e.getLanes():
                moves = []
                for c in ln.getOutgoing():
                    if c.getTo().getFunction() == "internal":
                        continue
                    moves.append(dict(dir=DIRECTION.get(c.getDirection(), c.getDirection()),
                                      code=c.getDirection(), to=c.getTo().getID(),
                                      to_name=c.getTo().getName() or "",
                                      to_lane=c.getToLane().getIndex()))
                    shape = via_shape(net, c)
                    if len(shape) > 1:
                        geo["conns"].append(dict(edge=e.getID(), dir=moves[-1]["dir"],
                                                 shape=lonlat(shape)))
                allowed = ln.getPermissions()
                lanes.append(dict(index=ln.getIndex(), width=round(ln.getWidth(), 2),
                                  speed_kmh=round(ln.getSpeed() * 3.6), moves=moves,
                                  bus_only="passenger" not in allowed and "bus" in allowed))
                if not moves:
                    hints.append(dict(level="bad", text="갈 곳이 없는 차로",
                                      detail=f"{e.getName() or e.getID()} {ln.getIndex()}번 차로"))
                geo["lanes"].append(dict(edge=e.getID(), dir="in",
                                         shape=lonlat(clip(ln.getShape(), CLIP_M, True))))
            shape = e.getLanes()[0].getShape()
            tail = clip(shape, 25, True)
            approaches.append(dict(edge=e.getID(), name=e.getName() or "", lanes=lanes,
                                   heading=heading(tail[0], tail[-1]),
                                   rules=rules.get((n.getID(), e.getID()), [])))
        exits = []
        for e in outgoing:
            for ln in e.getLanes():
                geo["lanes"].append(dict(edge=e.getID(), dir="out",
                                         shape=lonlat(clip(ln.getShape(), CLIP_M, False))))
            head = clip(e.getLanes()[0].getShape(), 25, False)
            exits.append(dict(edge=e.getID(), name=e.getName() or "", lanes=e.getLaneNumber(),
                              heading=heading(head[0], head[-1])))

        tls = n.getType().startswith("traffic_light")
        routes = on_route.get(n.getID(), 0)
        center = n.getCoord()[:2]
        near = net.getNeighboringEdges(*center, r=ALIGN_R)
        offset = align(np.array(center), sample_edges(net, center, {e for e, _ in near if e in plain}
                                                       | set(incoming) | set(outgoing)), segs)
        cards.append(dict(
            id=n.getID(), type=n.getType(), tls=tls,
            group="route" if routes else "tls" if tls else "other", routes=routes,
            names=sorted({e.getName() for e in incoming + outgoing if e.getName()}),
            lonlat=[round(v, 7) for v in net.convertXY2LonLat(*n.getCoord()[:2])],
            xy=[round(v, 2) for v in n.getCoord()[:2]],
            shape=lonlat(list(n.getShape()) + [n.getShape()[0]]) if len(n.getShape()) > 2 else [],
            approaches=approaches, exits=exits, hints=hints, geo=geo, offset=offset))

    order = {"route": 0, "tls": 1, "other": 2}
    issues = lambda c: sum(h["level"] != "info" for h in c["hints"])  # noqa: E731
    cards.sort(key=lambda c: (order[c["group"]], -c["routes"], -issues(c), c["id"]))
    roads = road_cards(net, plain, {c["id"] for c in cards}, edge_routes, edge_warn, segs)
    roads.sort(key=lambda c: (order[c["group"]], -c["routes"], -issues(c), c["id"]))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(net=NET.name, cards=cards, roads=roads), ensure_ascii=False,
                              separators=(",", ":")))
    for kind, items in (("교차로", cards), ("도로 구간", roads)):
        groups = defaultdict(int)
        for c in items:
            groups[c["group"]] += 1
        tls = f", 신호 {groups['tls']}" if groups["tls"] else ""
        print(f"[export_cards] {kind} {len(items)}개 (경로 위 {groups['route']}{tls}, "
              f"나머지 {groups['other']}), 자동 점검 걸림 {sum(issues(c) > 0 for c in items)}개, "
              f"자동 위치 보정 {sum(bool(c['offset']) for c in items)}개")
    print(f"[export_cards] → {out} ({out.stat().st_size / 1e6:.1f} MB)")
    return out


def node_names(node, skip=None):
    """교차로에 붙은 도로 이름들 (skip 이름은 빼고)."""
    return sorted({e.getName() for e in node.getIncoming() + node.getOutgoing()
                   if e.getFunction() != "internal" and e.getName() and e.getName() != skip})


def road_cards(net, edges, junction_cards, edge_routes, edge_warn, segs):
    """도로 구간 카드: 교차로와 교차로 사이 한 방향 도로 (SUMO edge) 하나씩."""
    links = {}
    if REGION.exists():
        links = {l["id"]: l for l in json.loads(REGION.read_text())["links"]}
    plain = set(edges)
    roads = []
    for e in edges:
        hints = list(edge_warn.get(e.getID(), []))
        to_node, from_node = e.getToNode(), e.getFromNode()
        lanes = []
        for ln in e.getLanes():
            allowed = ln.getPermissions()
            moves = sorted({DIRECTION.get(c.getDirection(), c.getDirection()) for c in ln.getOutgoing()})
            lanes.append(dict(index=ln.getIndex(), width=round(ln.getWidth(), 2),
                              speed_kmh=round(ln.getSpeed() * 3.6), moves=moves,
                              bus_only="passenger" not in allowed and "bus" in allowed))
            # 도로망 경계에서 빠져나가는 도로는 끝에 연결이 없는 게 정상
            if not moves and to_node.getType() != "dead_end":
                hints.append(dict(level="bad", text="끝에서 갈 곳이 없는 차로",
                                  detail=f"{ln.getIndex()}번 차로 (0 = 맨 오른쪽)"))
        speed = max(l["speed_kmh"] for l in lanes)

        link = links.get(e.getID().split("_")[0])
        nl = None
        if link:
            nl = dict(id=link["id"], lanes=link["lanes"], speed_kmh=link["max_speed"],
                      name=link["name"] or "")
            if link["lanes"] and link["lanes"] != len(lanes):
                hints.append(dict(level="warn", text="노드링크 원본과 차로 수가 다름",
                                  detail=f"노드링크 {link['lanes']}차로 · SUMO {len(lanes)}차로"))
            if link["max_speed"] and abs(link["max_speed"] - speed) >= 1:
                hints.append(dict(level="warn", text="노드링크 원본과 제한속도가 다름",
                                  detail=f"노드링크 {link['max_speed']} km/h · SUMO {speed} km/h"))
        # 직진으로 이어지는 다음 구간과 차로 수가 2개 이상 차이 나면 (갑자기 줄거나 늘어남)
        for nxt, conns in e.getOutgoing().items():
            if nxt.getFunction() != "internal" and any(c.getDirection() == "s" for c in conns) \
                    and abs(nxt.getLaneNumber() - len(lanes)) >= 2:
                hints.append(dict(level="info", text="직진으로 이어지는 다음 구간과 차로 수 차이가 큼",
                                  detail=f"{len(lanes)}차로 → {nxt.getName() or nxt.getID()} "
                                         f"{nxt.getLaneNumber()}차로"))
        if e.getLength() < 5:
            hints.append(dict(level="info", text=f"아주 짧은 구간 ({e.getLength():.1f} m)", detail=""))

        shape = e.getShape()
        mid = sumolib.geomhelper.positionAtShapeOffset(shape, e.getLength() / 2)
        r = min(max(e.getLength() / 2 + 40, ALIGN_R), 300)   # 긴 도로는 넓게 맞춘다
        near = {x for x, _ in net.getNeighboringEdges(*mid, r=r) if x in plain} | {e}
        offset = align(np.array(mid), sample_edges(net, mid, near, r), segs, r)
        opposite = [x.getID() for x in to_node.getOutgoing()
                    if x.getToNode() == from_node and x.getFunction() != "internal"]
        _, rank, level = ((e.getType() or "") + "..").split(".")[:3]
        routes = edge_routes.get(e.getID(), 0)
        end = lambda n: dict(id=n.getID(), card=n.getID() in junction_cards,   # noqa: E731
                             names=node_names(n, e.getName()), type=n.getType())
        roads.append(dict(
            id=e.getID(), name=e.getName() or "", group="route" if routes else "other", routes=routes,
            rank=ROAD_RANK.get(rank, rank), level=ROAD_TYPE.get(level, level),
            length=round(e.getLength(), 1), speed_kmh=speed, lanes=lanes,
            heading=heading(shape[0], shape[-1]), start=end(from_node), end=end(to_node),
            opposite=opposite[0] if opposite else None, nl=nl, hints=hints,
            lonlat=[round(v, 7) for v in net.convertXY2LonLat(*mid)], xy=[round(v, 2) for v in mid],
            offset=offset))
    return roads


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--net", type=Path, default=NET, help="SUMO 도로망 (기본: 여의도)")
    args = ap.parse_args(argv)
    export(args.net)


if __name__ == "__main__":
    main()
