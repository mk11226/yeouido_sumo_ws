"""2단계: 국가표준노드링크 → SUMO 도로망(yeouido.net.xml).

    python -m yeouido.build_net        (먼저 python -m yeouido.nodelink)

신호 위치는 표준노드링크에 없으므로 OSM 신호 표시와 간선 교차로 기준으로 정한다.
검수로 확인한 수정(도로 삭제·추가, 차로 수, 회전 포켓, 차로별 방향)은 yeouido/corrections.json 에 적고 마지막에 덧붙인다.

    python -m yeouido.build_net --no-corrections          # 수정 목록 없이 (원본 그대로)
    python -m yeouido.build_net --out 다른/경로/net.xml    # 다른 곳에 만들어 비교 (env 파일은 그대로)
"""
import argparse
import itertools
import json
import math
import re
import subprocess
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from yeouido import corrections
from yeouido.config import BBOX, NET, OUT, south_of_river, sumo_bin
from yeouido.download import osm_path
from yeouido.nodelink import REGION

PLAIN = OUT / "plain"
# 도시고속도로(올림픽대로·서부간선도로 등)와 고속국도는 뺀다. 측정값이 없어 교통량을
# 정할 수 없고, 거기서 내려오는 램프는 범위 입구(차가 들어오고 나가는 곳)가 된다.
EXCLUDED_RANKS = {"101", "102"}
# 안양천 서쪽 목동의 목동동로·목동서로는 뺀다 (안양천로·목동교·양평교 등 안양천 쪽은 남긴다).
# 노드링크 링크 ID 앞 4자리가 관리 구 코드(1140 = 양천구)이고, 이 범위에서는 안양천이 양천구와
# 영등포구의 경계다. 이 두 길로만 이어지던 조각(골목 등)은 연결된 큰 도로망만 남기는 단계에서 같이 빠진다.
EXCLUDED_WEST = ("1140", {"목동동로", "목동서로"})   # (구 코드, 도로명)
# 1차로 골목길("○○로8길", "○○가길", 이름 없는 도로)은 뺀다. 전체 링크의 약 1/3이고,
# 큰길에 붙은 골목 교차로가 교차로 모양을 복잡하게 만들어 교차로 안 충돌의 원인이 됐다.
# 2차로 이상 골목길과 연결로(램프)는 남긴다.
ALLEY = re.compile(r"길$")


def is_minor_alley(link):
    return ((ALLEY.search(link["name"]) or link["name"] in ("", "-"))
            and link["lanes"] <= 1 and not link["connect"])


# 이름이 같은 교차로 노드를 잇는 링크가 이보다 짧으면 합친다 (m). 교차로 내부 링크는 보통
# 12~14 m 이다. 더 크게 잡으면 마포대교 남단 교차로와 35 m 떨어진 "마포대교앞" 합류점
# (여의대로 우회전 전용 차로가 들어오는 곳)까지 하나로 합쳐져 교차로가 엉켰다.
JOIN_DIST = 30.0
JOIN_SHORT = 20.0      # 이름과 상관없이 교차로 노드를 잇는 링크가 이보다 짧으면 합친다 (m)
SIGNAL_SNAP = 35.0     # OSM 신호 표시를 교차로에 붙이는 최대 거리 (m)
MIN_PIECE = 5.0        # 범위 경계에서 잘리고 남은 조각이 이보다 짧으면 버린다 (m)
# 갈라지는 고가·지하차도와 옆 도로 벌리기 (separate_levels)
LEVEL_GAP = 0.5        # 두 도로 차로 가장자리 사이에 둘 간격 (m)
LEVEL_TAPER = 0.15     # 끝점(교차로)에서 1 m 멀어질 때마다 더 옮길 수 있는 양 (m)
LEVEL_MAX_SHIFT = 3.0  # 한 도로를 옮기는 최대 거리 (m)
LEVEL_STEP = 10.0      # 옮기는 양을 정하는 점 간격 (m)
LEVEL_ITER = 6         # 양쪽에 도로가 붙은 곳은 한 번에 안 맞아서 몇 번 반복한다
LEVEL_ZONE = 20.0      # 끝점에서 이 거리 안은 원래 붙어 있는 분기·합류 구간 — 여기서만 겹치면 옮기지 않는다 (m)
# 표준노드링크 회전 제한 중 승용차가 그 방향으로 못 가는 것
FORBIDDEN_TURNS = {"003", "002", "101", "102", "103"}   # 회전금지, 버스만, 좌·직·우 금지

# 신호 공통 설정. 실제 신호 시간 계획은 공개되어 있지 않아 감응식으로 추정한다.
# - layout incoming: 진입 방향별로 차례로 녹색(직진+좌회전 동시). 한국 도심 교차로의 일반적인
#   방향별 순차 신호이며, 맞은편 직진과 비보호 좌회전이 섞이지 않아 교차로 안 교착·충돌이 없다.
#   (SUMO 기본 opposites 방식에서는 마포대교 남단 등에서 좌회전 차가 교차로 안에 갇혔다)
# - all-red 2초: 노란불 끝에 교차로 안에 남은 차가 빠져나간 뒤 다음 방향에 녹색을 준다.
TLS_OPTIONS = ["--tls.default-type", "actuated", "--tls.layout", "opposites", "--tls.left-green.time", "8",
               "--tls.cycle.time", "90", "--tls.yellow.time", "3", "--tls.allred.time", "2"]


def _dist(a, b):
    """위경도 두 점 사이 거리 (m, 짧은 거리용 근사)."""
    dx = (a[0] - b[0]) * 111320 * math.cos(math.radians(a[1]))
    dy = (a[1] - b[1]) * 110540
    return math.hypot(dx, dy)


def _inside(lon, lat):
    b = BBOX
    return (b["west"] <= lon <= b["east"] and b["south"] <= lat <= b["north"]
            and south_of_river(lon, lat))


def _cross(p, q):
    """p(안)–q(밖) 사이 경계점을 이분법으로 찾는다."""
    lo, hi = 0.0, 1.0
    for _ in range(30):
        mid = (lo + hi) / 2
        if _inside(p[0] + (q[0] - p[0]) * mid, p[1] + (q[1] - p[1]) * mid):
            lo = mid
        else:
            hi = mid
    return [p[0] + (q[0] - p[0]) * lo, p[1] + (q[1] - p[1]) * lo]


def clip(links, nodes):
    """범위 밖으로 나가는 링크를 경계점에서 자른다. 잘린 끝에는 새 노드(경계 노드)를 만든다."""
    out, new_nodes = [], {}
    for l in links:
        if l["rank"] in EXCLUDED_RANKS or (l["id"].startswith(EXCLUDED_WEST[0]) and l["name"] in EXCLUDED_WEST[1]):
            continue
        pts = l["shape"]
        flags = [_inside(*p) for p in pts]
        if all(flags):
            out.append(l)
            continue
        pieces, cur, start_cut = [], [], False
        for i, (p, f) in enumerate(zip(pts, flags)):
            if f:
                if not cur:
                    start_cut = i > 0
                    if start_cut:
                        cur.append(_cross(p, pts[i - 1]))
                cur.append(p)
            elif cur:
                cur.append(_cross(pts[i - 1], p))
                pieces.append((cur, start_cut, True))
                cur = []
        if cur:
            pieces.append((cur, start_cut, False))
        for k, (shape, cut_start, cut_end) in enumerate(pieces):
            length = sum(_dist(a, b) for a, b in zip(shape, shape[1:]))
            if length < MIN_PIECE:
                continue
            piece = dict(l, id=l["id"] if len(pieces) == 1 else f"{l['id']}_{k}", shape=shape)
            if cut_start:
                piece["from_node"] = f"B{l['id']}_{k}s"
                new_nodes[piece["from_node"]] = dict(type="boundary", name="", lon=shape[0][0],
                                                     lat=shape[0][1])
            if cut_end:
                piece["to_node"] = f"B{l['id']}_{k}e"
                new_nodes[piece["to_node"]] = dict(type="boundary", name="", lon=shape[-1][0],
                                                   lat=shape[-1][1])
            out.append(piece)
    used = {n for l in out for n in (l["from_node"], l["to_node"])}
    all_nodes = {**{k: v for k, v in nodes.items() if k in used}, **new_nodes}
    return out, all_nodes


def largest_component(links, nodes, group_of=None):
    """서로 이어진 가장 큰 도로망만 남긴다. (골목길을 빼면 공원 내부 도로 같은 조각이 끊어져
    남는데, netconvert가 이를 먼저 지우면 교차로 합치기 목록이 깨지므로 여기서 미리 정리)
    group_of 를 주면 합쳐질 교차로 노드들을 하나로 보고 연결 여부를 판단한다."""
    g = group_of or {}
    adj = defaultdict(set)
    for l in links:
        a, b = g.get(l["from_node"], l["from_node"]), g.get(l["to_node"], l["to_node"])
        adj[a].add(b)
        adj[b].add(a)
    best, seen = set(), set()
    for start in adj:
        if start in seen:
            continue
        comp, stack = set(), [start]
        while stack:
            n = stack.pop()
            if n in comp:
                continue
            comp.add(n)
            stack.extend(adj[n] - comp)
        seen |= comp
        if len(comp) > len(best):
            best = comp
    links = [l for l in links if g.get(l["from_node"], l["from_node"]) in best]
    used = {n for l in links for n in (l["from_node"], l["to_node"])}
    return links, {k: v for k, v in nodes.items() if k in used}


def _base_name(name):
    """교차로 이름 표기 차이를 없앤다.
    예) "서강대교(남측)", "서강대교남단", "서강대교남측교차로" → "서강대교남" """
    n = re.sub(r"[\s()]", "", name)
    n = re.sub(r"(교차로|사거리|삼거리|오거리)$", "", n)
    for side in "남북동서":
        n = re.sub(f"{side}(측|단)$", side, n)
    return n


def _levels(links):
    """노드가 고가·지하(고가차도 001, 지하차도 002) 링크에만 붙어 있으면 그 높이를 돌려준다.
    지상 링크가 하나라도 붙어 있으면(진출입구 포함) 지상(000)으로 본다."""
    kinds = defaultdict(set)
    for l in links:
        lvl = l["type"] if l["type"] in ("001", "002") else "000"
        kinds[l["from_node"]].add(lvl)
        kinds[l["to_node"]].add(lvl)
    return {n: ("000" if "000" in k else min(k)) for n, k in kinds.items()}


def join_clusters(nodes, links):
    """같은 교차로에 속한 노드들을 하나로 묶는다 (union-find).

    표준노드링크는 한 교차로를 진입·진출 지점마다 노드로 나누고, 그 사이를 짧은 링크로
    잇는다 (중앙분리대가 있는 도로끼리 만나면 네모 모양 노드 4개). 그래서
      - 교차로 노드 두 개가 링크로 직접 이어져 있고
      - 그 링크가 JOIN_SHORT m 이하이거나, 이름(표기 차이 정리 후)이 같고 JOIN_DIST m 이하이면
    같은 교차로로 본다. 이름만 같고 연결되지 않은 노드(예: 신도림고가차도 옆 도림천로 분기점)나
    주소로 이름이 제각각인 귀퉁이 노드(예: 선유로×문래북로)도 이 기준으로 바르게 처리된다.
    높이가 다른 노드(고가 위 / 지하 / 지상)는 묶지 않는다 → 고가·지하차도가 평면 교차로가 되지 않는다.

    돌려주는 값: (묶음 목록, 연결된 큰 도로망의 링크, 그 노드)
    """
    parent = {n: n for n in nodes}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n

    level = _levels(links)
    junction_like = {n for n, v in nodes.items() if v["type"] in ("101", "104", "106")}
    for l in links:
        a, b = l["from_node"], l["to_node"]
        if a not in junction_like or b not in junction_like or l["type"] in ("001", "002"):
            continue
        if level.get(a) != level.get(b):
            continue
        same = (nodes[a]["name"] and _base_name(nodes[a]["name"]) == _base_name(nodes[b]["name"]))
        if l["length"] <= JOIN_SHORT or (same and l["length"] <= JOIN_DIST):
            parent[find(a)] = find(b)

    group_of = {n: find(n) for n in nodes}
    # 1차로 골목길은 여기서 뺀다. 단, 교차로 내부를 잇는 링크는 이름이 "○○길"이어도 남긴다.
    # (영등포로터리는 고리 모양 노드 4개 중 한 변이 "영등포로53길"이라 먼저 지우면 교차로가
    #  둘로 쪼개져 서로 막는 교착이 생겼다)
    links = [l for l in links
             if not is_minor_alley(l) or group_of[l["from_node"]] == group_of[l["to_node"]]]
    # 교차로 내부 링크는 그대로 두면 netconvert가 합치면서 지우고, 그 링크를 근거로 교차로를
    # 지나는 차로 연결을 만든다. (직접 지우면 연결 근거가 사라져 교차로 절반이 막혔다)
    # 합쳐질 교차로를 한 점으로 보고 연결된 큰 도로망만 남긴다.
    links, nodes = largest_component(links, nodes, group_of)
    groups = defaultdict(list)
    for nid in nodes:
        groups[group_of[nid]].append(nid)
    return [g for g in groups.values() if len(g) > 1], links, nodes


def osm_signals():
    root = ET.parse(osm_path()).getroot()
    return [(float(n.get("lon")), float(n.get("lat"))) for n in root.iter("node")
            if any(t.get("k") == "highway" and t.get("v") == "traffic_signals"
                   for t in n.findall("tag"))]


def signal_nodes(links, nodes, clusters):
    """신호를 둘 교차로(노드 묶음)를 정한다.

    - OSM에 신호 표시가 SIGNAL_SNAP m 안에 있는 교차로, 또는
    - 들어오는 링크가 3개 이상이고, 도로 이름이 2개 이상이며, 2차로 이상 도로가 있는 교차로
      (서울 간선도로 교차로는 거의 모두 신호 교차로다)
    """
    member = {n: tuple(sorted(g)) for g in clusters for n in g}
    incoming = defaultdict(list)
    for l in links:
        incoming[member.get(l["to_node"], (l["to_node"],))].append(l)
    signals = osm_signals()
    chosen = set()
    for group, inc in incoming.items():
        if any(nodes[n]["type"] == "boundary" for n in group):
            continue
        pts = [(nodes[n]["lon"], nodes[n]["lat"]) for n in group]
        near_signal = any(_dist(p, s) <= SIGNAL_SNAP for p in pts for s in signals)
        sources = {l["from_node"] for l in inc}
        names = {l["name"] for l in inc if l["name"]}
        major = len(sources) >= 3 and len(names) >= 2 and max(l["lanes"] for l in inc) >= 2
        if near_signal or major:
            chosen.add(group)
    return chosen


def _level(link):
    return link["type"] if link["type"] in ("001", "002") else "000"


def _densify(shape, step):
    """위경도 선에 step m 보다 긴 변이 없게 중간 점을 넣는다 (끝점·꺾인 점은 그대로)."""
    out = [list(shape[0])]
    for a, b in zip(shape, shape[1:]):
        n = max(1, math.ceil(_dist(a, b) / step))
        out += [[a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n] for k in range(1, n + 1)]
    return out


def _nearest(p, poly):
    """점 p 에서 선 poly 까지 가장 가까운 점과 거리 (평면 m 좌표)."""
    best = (math.inf, None)
    for (ax, ay), (bx, by) in zip(poly, poly[1:]):
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L))
        c = (ax + t * dx, ay + t * dy)
        d = math.hypot(p[0] - c[0], p[1] - c[1])
        if d < best[0]:
            best = (d, c)
    return best


def separate_levels(links, clusters):
    """같은 교차로에서 갈라지거나 합쳐지는 고가·지하차도와 옆 도로가 겹쳐 그려지지 않게 벌린다.

    표준노드링크는 도로를 중심선 하나로 주는데, 지하차도와 바로 옆 지상 도로(또는 고가와 그 옆
    도로)는 중심선 간격이 차로 폭보다 좁게 들어가 있어 SUMO에서 차로가 겹쳐 그려졌다. 서로 다른
    도로라 차끼리 간섭은 없지만 화면에서는 한 덩어리처럼 보인다. 두 도로가 차로 폭 + LEVEL_GAP 만큼
    떨어지게 각자 반씩 서로 반대쪽으로 옮긴다.
      - 양 끝점(교차로)은 그대로 두고, 끝에서 멀어질수록 LEVEL_TAPER 기울기로 옮기는 양을 늘린다
        → 갈라지는 곳의 연결 모양은 바뀌지 않는다.
      - 한 도로를 LEVEL_MAX_SHIFT m 보다 많이 옮기지 않는다 (그보다 크게 겹치면 원본 배치 문제).
      - 높이가 같은 도로끼리는 건드리지 않는다 (같은 높이에서 나란한 도로는 검수 대상).
    돌려주는 값: (새 링크 목록, 옮긴 도로 수, 가장 많이 옮긴 거리 m)"""
    group = {n: i for i, g in enumerate(clusters) for n in g}
    key = lambda n: group.get(n, n)                                   # noqa: E731
    ends = defaultdict(list)
    for i, l in enumerate(links):
        ends["from", key(l["from_node"])].append(i)
        ends["to", key(l["to_node"])].append(i)
    pairs = set()
    for idx in ends.values():
        for a, b in itertools.combinations(idx, 2):
            A, B = links[a], links[b]
            if _level(A) == _level(B):
                continue
            if key(A["from_node"]) == key(B["to_node"]) and key(A["to_node"]) == key(B["from_node"]):
                continue                                              # 같은 도로의 반대 방향
            pairs.add((min(a, b), max(a, b)))
    if not pairs:
        return links, 0, 0.0

    kx, ky = 111320 * math.cos(math.radians((BBOX["south"] + BBOX["north"]) / 2)), 110540
    to_m = lambda s: [(x * kx, y * ky) for x, y in s]                  # noqa: E731
    involved = {i for p in pairs for i in p}
    base = {i: _densify(links[i]["shape"], LEVEL_STEP) for i in involved}
    width = {i: max(links[i]["lanes"], 1) * corrections.LANE_WIDTH for i in involved}
    along, dist_along, along_len = {}, {}, {}
    for i, s in base.items():
        m = to_m(s)
        acc = [0.0]
        for a, b in zip(m, m[1:]):
            acc.append(acc[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        dist_along[i], along_len[i] = acc, acc[-1]
        along[i] = [min(LEVEL_MAX_SHIFT, LEVEL_TAPER * min(s_, acc[-1] - s_)) for s_ in acc]
    # 분기·합류 구간 밖에서도 겹치는 쌍만 옮긴다. 갈라지는 곳 근처는 원래 붙어 있으므로, 거기까지
    # 밀면 갈라지는 각도가 바뀌어 netconvert가 교차로를 더 크게 잡았다.
    cur = {i: to_m(base[i]) for i in involved}

    def overlaps(p, q):
        need = (width[p] + width[q]) / 2 + LEVEL_GAP
        L = along_len[p]
        return any(_nearest(pt, cur[q])[0] < need for pt, s_ in zip(cur[p], dist_along[p])
                   if LEVEL_ZONE <= s_ <= L - LEVEL_ZONE)
    pairs = {(a, b) for a, b in pairs if overlaps(a, b) or overlaps(b, a)}
    involved = {i for p in pairs for i in p}
    total = {i: [0.0] * len(base[i]) for i in involved}
    for _ in range(LEVEL_ITER):
        cur = {i: to_m(corrections.offset_right(base[i], total[i])) for i in involved}
        step = {i: [0.0] * len(base[i]) for i in involved}
        for a, b in pairs:
            need = (width[a] + width[b]) / 2 + LEVEL_GAP
            for p, q in ((a, b), (b, a)):
                pts = cur[p]
                for k, pt in enumerate(pts):
                    d, c = _nearest(pt, cur[q])
                    if d >= need or d < 1e-6:
                        continue
                    t0, t1 = pts[max(k - 1, 0)], pts[min(k + 1, len(pts) - 1)]
                    tx, ty = t1[0] - t0[0], t1[1] - t0[1]
                    right = (c[0] - pt[0]) * ty - (c[1] - pt[1]) * tx > 0      # 상대 도로가 오른쪽
                    step[p][k] += (need - d) / 2 * (-1 if right else 1)
        moved = False
        for i in involved:
            for k, s_ in enumerate(step[i]):
                if s_:
                    lim = along[i][k]
                    new = max(-lim, min(lim, total[i][k] + s_))
                    moved |= abs(new - total[i][k]) > 0.01
                    total[i][k] = new
        if not moved:
            break
    if not pairs:
        return links, 0, 0.0
    out, n_moved, biggest = list(links), 0, 0.0
    for i in involved:
        far = max(map(abs, total[i]))
        if far > 0.05:
            out[i] = dict(links[i], shape=corrections.offset_right(base[i], total[i]))
            n_moved += 1
            biggest = max(biggest, far)
    return out, n_moved, biggest


def write_plain(links, nodes, clusters, tls_groups, turns, plain=PLAIN, splits=None, extra_con=()):
    """splits: {도로 ID: dict(pos, lanes, id_after)} — 도로 끝을 잘라 차로를 늘린다 (corrections).
    포켓 구간은 바깥쪽으로만 넓힌다 (중앙선 쪽 가장자리 그대로).
    extra_con: 포켓·차로 방향 수정 도로의 차로별 연결 (corrections.connections).
    차로 수를 고친 도로(lanes0 있음)도 우선순위는 원래 차로 수로 정한다 (교차로 양보 관계 유지)."""
    splits = splits or {}
    plain.mkdir(parents=True, exist_ok=True)
    tls_nodes = {n for g in tls_groups for n in g}
    nod = ["<nodes>"]
    for nid, n in nodes.items():
        kind = "traffic_light" if nid in tls_nodes else "priority"
        nod.append(f'    <node id="{nid}" x="{n["lon"]:.7f}" y="{n["lat"]:.7f}" type="{kind}"/>')
    by_id = {l["id"]: l for l in links}
    for eid, sp in splits.items():   # 포켓이 시작하는 점 (자른 곳)
        l = by_id[eid]
        _, _, p = corrections._cut(l["shape"], max(corrections._length(l["shape"]) + sp["pos"], 1.0))
        sp["node"] = f"{eid}{corrections.POCKET_SUFFIX}.start"
        nod.append(f'    <node id="{sp["node"]}" x="{p[0]:.7f}" y="{p[1]:.7f}" type="priority"/>')
    for g in clusters:
        nod.append(f'    <join nodes="{" ".join(g)}"/>')
    nod.append("</nodes>")
    (plain / "nl.nod.xml").write_text("\n".join(nod) + "\n")

    prio = {"103": 12, "104": 10, "105": 9, "106": 9, "107": 7, "108": 6}
    edg = ["<edges>"]

    def edge_xml(l, eid, frm, to, lanes, shape):
        pts = " ".join(f"{x:.7f},{y:.7f}" for x, y in shape)
        speed = max(l["max_speed"], 20) / 3.6
        name = l["name"].replace("&", "&amp;").replace('"', "&quot;")
        return (f'    <edge id="{eid}" from="{frm}" to="{to}" '
                f'numLanes="{max(lanes, 1)}" speed="{speed:.2f}" '
                f'priority="{prio.get(l["rank"], 6) + min(l.get("lanes0", l["lanes"]), 4)}" '
                + (f'name="{name}" ' if name else "") + f'type="nl.{l["rank"]}.{l["type"]}" shape="{pts}" '
                f'spreadType="center"/>')

    for l in links:
        s = splits.get(l["id"])
        if not s:
            edg.append(edge_xml(l, l["id"], l["from_node"], l["to_node"], l["lanes"], l["shape"]))
            continue
        # 포켓: 끝에서 -pos m 지점에서 도로를 둘로 나눠 뒤쪽(포켓 구간) 차로를 늘린다.
        # 차로 수 수정과 같이 중앙선 쪽 가장자리는 그대로 두고 바깥쪽으로만 넓힌다
        # (포켓 구간 도로선을 늘어난 폭의 절반만큼 오른쪽으로 → 반대 방향 도로를 파고들지 않음).
        # 어느 차로가 새 차로인지는 extra_con 의 연결로 정한다.
        length = corrections._length(l["shape"])
        before, after, p = corrections._cut(l["shape"], max(length + s["pos"], 1.0))
        mid = s["node"]
        shift = (s["lanes"] - max(l["lanes"], 1)) * corrections.LANE_WIDTH / 2
        after = corrections.offset_right(after, shift)
        edg.append(edge_xml(l, l["id"], l["from_node"], mid, l["lanes"], before))
        edg.append(edge_xml(l, s["id_after"], mid, l["to_node"], s["lanes"], after))
    edg.append("</edges>")
    (plain / "nl.edg.xml").write_text("\n".join(edg) + "\n")

    ids = {l["id"] for l in links}
    con = ["<connections>"]
    n_del = 0
    for t in turns:
        if t["type"] in FORBIDDEN_TURNS and t["from_link"] in ids and t["to_link"] in ids:
            con.append(f'    <delete from="{t["from_link"]}" to="{t["to_link"]}"/>')
            n_del += 1
    con += list(extra_con)
    con.append("</connections>")
    (plain / "nl.con.xml").write_text("\n".join(con) + "\n")
    return n_del


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
        # 지퍼 합류는 비슷한 두 도로가 하나로 모이면서 차로가 실제로 줄어들 때만 쓴다.
        # - 셋 이상이 모이거나 차로가 충분하면 지퍼 규칙 때문에 오히려 양보 교착이 생겼다.
        # - 본선에 우회전 전용 차로 같은 보조 도로가 붙는 곳(도로 우선순위가 다름)은 본선 우선이
        #   실제와 맞다. 지퍼로 하면 마포대교 남단처럼 본선이 막혀 교차로까지 정체가 찼다.
        lanes_in = sum(e.getLaneNumber() for e in inc)
        if (len(inc) == 2 and len(out) == 1 and lanes_in > out[0].getLaneNumber()
                and inc[0].getPriority() == inc[1].getPriority()):
            nodes.append(n.getID())
    return nodes


CLEAR_SPEED = 7.0              # 교차로 안을 빠져나가는 속도 가정 (m/s, 약 25 km/h)
ALLRED_MIN, ALLRED_MAX = 2, 8  # 전체 적색 시간 범위 (초)


def set_clearance_times(path):
    """신호 교차로마다 전체 적색 시간을 교차로 크기에 맞춘다.

    노란불 3초 + 전체 적색 2초로는 여의대로(왕복 10차로)처럼 넓은 교차로를 다 건너지 못한
    차가 교차로 안에 남고, 다음 방향 녹색 흐름이 끊길 때까지 수십 초씩 갇혔다 (마포대교 남단).
    교차로 안 가장 긴 경로를 CLEAR_SPEED 로 지나는 시간 - 노란불 시간 을 전체 적색으로 준다.
    """
    tree = ET.parse(path)
    root = tree.getroot()
    lane_len = {l.get("id"): float(l.get("length")) for e in root.iter("edge")
                if e.get("function") == "internal" for l in e.findall("lane")}
    longest = defaultdict(float)
    for c in root.iter("connection"):
        if c.get("tl") and c.get("via") in lane_len:
            longest[c.get("tl")] = max(longest[c.get("tl")], lane_len[c.get("via")])
    changed = []
    for tl in root.iter("tlLogic"):
        need = longest.get(tl.get("id"), 0) / CLEAR_SPEED
        phases = tl.findall("phase")
        for i, ph in enumerate(phases):
            state = ph.get("state")
            prev = phases[i - 1].get("state")
            if set(state) == {"r"} and "y" in prev:
                yellow = float(phases[i - 1].get("duration"))
                allred = min(max(round(need - yellow), ALLRED_MIN), ALLRED_MAX)
                if allred != float(ph.get("duration")):
                    ph.set("duration", str(allred))
                    changed.append(allred)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return changed


def _run(cmd, log):
    with open(log, "a") as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        raise RuntimeError(f"netconvert 실패 — 로그 확인: {log}")


def _convert(plain, net_out, log):
    """plain 파일(노드·도로·연결) → SUMO 도로망 (1차, 신호·교차로 생성)."""
    _run([sumo_bin("netconvert"),
          "--node-files", str(plain / "nl.nod.xml"), "--edge-files", str(plain / "nl.edg.xml"),
          "--connection-files", str(plain / "nl.con.xml"),
          "--proj.utm",                         # 위경도 입력 → UTM 평면 좌표
          "-o", str(net_out),
          # 연결된 큰 도로망만 남기는 일은 join_clusters 에서 합친 교차로 기준으로 이미 했다.
          # (netconvert의 components 옵션은 교차로를 합치기 전에 판단해 필요한 노드를 지운다)
          "--output.street-names", "--no-turnarounds",
          "--junctions.corner-detail", "5",
          "--tls.join", *TLS_OPTIONS], log)


def landmark_edges():
    """랜드마크가 붙은 도로 (자르면 랜드마크 위치가 바뀌므로 수정하지 않는다)."""
    path = OUT / "landmarks.json"
    return {v["edge"] for v in json.loads(path.read_text()).values()} if path.exists() else set()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=NET, help="만들 도로망 파일 (기본: env/yeouido/yeouido.net.xml)")
    ap.add_argument("--no-corrections", action="store_true", help="yeouido/corrections.json 을 반영하지 않음")
    args = ap.parse_args(argv)
    net_out = args.out.resolve()
    # 기본 위치가 아니면 plain 파일·로그도 그 옆에 따로 둔다 (env/yeouido 는 건드리지 않음)
    default = net_out == NET.resolve()
    plain = PLAIN if default else net_out.parent / f"{net_out.stem}_plain"
    log = OUT / "netconvert.log" if default else net_out.with_name(f"{net_out.stem}.netconvert.log")
    net_out.parent.mkdir(parents=True, exist_ok=True)

    OUT.mkdir(parents=True, exist_ok=True)
    region = json.loads(REGION.read_text())
    links, nodes = clip(region["links"], region["nodes"])
    links, nodes = largest_component(links, nodes)
    clusters, links, nodes = join_clusters(nodes, links)
    data = {} if args.no_corrections else corrections.load()
    # 교차로 합치기: 노드링크가 나눠 놓은 교차로를 하나로 (사이 링크는 교차로 내부가 된다)
    clusters, joined, skipped = corrections.join_junctions(clusters, nodes, data.get("join_junctions", []),
                                                             _levels(links))
    # 합친 교차로에서 원래 떨어져 있던 노드 사이 회전(connect): netconvert 는 교차로를 합칠 때 안쪽 링크로
    # 이어져 있던 길만 회전으로 만들고 연결 파일의 연결도 지우므로, 두 노드 사이에 안쪽 링크를 넣어 준다
    # (교차로 안이라 netconvert 가 합치면서 없애고, 진입로 → 진출로 회전만 남는다)
    inner, more = corrections.join_connections(data.get("join_junctions", []), links, joined)
    links += inner
    skipped += more
    # 도로 삭제: 교차로 묶음은 그대로 두고 빼며, 그 때문에 떨어져 나간 조각도 정리한다
    links, removed, more = corrections.remove_links(links, data.get("remove_links", []), clusters,
                                                       landmark_edges())
    skipped += more
    orphans = []
    if removed:
        group_of = {n: g[0] for g in clusters for n in g}
        before = [l["id"] for l in links]
        links, nodes = largest_component(links, nodes, group_of)
        kept = {l["id"] for l in links}
        orphans = [i for i in before if i not in kept]
        clusters = [g for g in ([n for n in g if n in nodes] for g in clusters) if len(g) > 1]
    tls_groups = signal_nodes(links, nodes, clusters)
    import sumolib

    links, more = corrections.lane_counts(links, data.get("lane_counts", []))
    skipped += more
    # 새 도로: 차로 수 수정 뒤에 붙인다 (자른 도로 조각이 바뀐 차로 수·위치를 이어받게)
    links, nodes, added, cut, more = corrections.add_links(links, nodes, data.get("add_links", []),
                                                           landmark_edges())
    skipped += more
    # 회전 제한 지우기 (원래 링크 ID 기준이라 도로를 자르기 전 목록에서 뺀다)
    turns, n_turns, more = corrections.remove_turns(region["turns"], data.get("remove_turns", []))
    skipped += more
    turns = [dict(t, to_link=cut.get(t["to_link"], t["to_link"])) for t in turns]
    # 합친 교차로 안으로 들어간 링크의 회전 제한 → 진입로 → 진출로 회전 제한으로 옮김
    carried = corrections.carry_turns(turns, links, joined)
    turns += carried
    # connect 로 넣은 안쪽 링크는 적은 회전만 쓰게 한다 (다른 진입로가 타고 가는 회전은 막음)
    allowed = {(str(a), str(b)) for c in data.get("join_junctions", []) for a, b in c.get("connect", [])}
    turns += corrections.inner_only_turns(links, joined, inner, allowed)
    # 차로 수 수정으로 폭·위치가 바뀐 뒤에 벌린다 (포켓 계획은 옮긴 모양 기준)
    links, n_sep, max_sep = separate_levels(links, clusters)
    print(f"[net] 갈라지는 고가·지하차도와 옆 도로 벌리기: {n_sep}개 도로 (최대 {max_sep:.1f} m)")
    plans, more = corrections.turn_pockets(links, data.get("turn_pockets", []), landmark_edges())
    skipped += more
    moves = data.get("lane_moves", [])
    splits, extra_con = {}, []
    # 고친 도로 (차로 수, 회전 제한 지우기, 새 도로): 차로 연결을 정리 규칙으로 다시 정한다 (corrections.connections)
    edited = ({l["id"] for l in links if "lanes0" in l} | {c["from"] for c in data.get("remove_turns", [])}
              | set(added) | set(cut.values()) | {c.get("to") for c in data.get("add_links", [])}) - {None}
    if plans or moves or edited:
        # 차로별 연결을 정하려면 원래 차로마다 어디로 가는지 알아야 해서, 먼저 차로 수 수정만 넣고 만든다
        write_plain(links, nodes, clusters, tls_groups, turns, plain)
        net0 = plain / "no_corrections.net.xml"
        log0 = plain / "no_corrections.netconvert.log"
        log0.write_text("")
        _convert(plain, net0, log0)
        extra_con, more = corrections.connections(sumolib.net.readNet(str(net0)), plans, moves, edited)
        skipped += more
        by_id = {l["id"]: l for l in links}
        splits = {eid: corrections.split_spec(by_id[eid], p) for eid, p in plans.items()}
    n_del = write_plain(links, nodes, clusters, tls_groups, turns, plain, splits, extra_con)
    print(f"[net] 표준노드링크 링크 {len(links)}개, 노드 {len(nodes)}개, "
          f"교차로 합치기 {len(clusters)}곳, 신호 교차로 {len(tls_groups)}곳, 회전 금지 {n_del}개")
    if not args.no_corrections:
        n_moves = len(moves) - sum(s["type"] == "lane_moves" for s in skipped)
        print(f"[net] 수정 목록 (yeouido/corrections.json): 교차로 합침 {len(joined)}곳"
              + (f" (안으로 들어간 링크의 회전 제한 {len(carried)}개 옮김)" if carried else "") + f", 회전 제한 지움 {n_turns}개, 도로 삭제 {len(removed)}개"
              + (f" (끊겨서 같이 빠진 도로 {len(orphans)}개: {', '.join(orphans)})" if orphans else "")
              + f", 새 도로 {len(added)}개" + (f" (자른 도로 {', '.join(cut)})" if cut else "")
              + f", 차로 수 "
              f"{sum('lanes0' in l for l in links)}곳, 좌회전 포켓 "
              f"{sum(1 for p in plans.values() if p['left'])}곳, 우회전 포켓 "
              f"{sum(1 for p in plans.values() if p['right'])}곳, 차로 방향 {n_moves}곳 반영"
              + "".join(f"\n      건너뜀 {corrections.label(s)}: {s['why']}" for s in skipped))
    # 검수 도구가 읽는 건너뛴 항목 목록
    (plain / "skipped.json").write_text(json.dumps(skipped, ensure_ascii=False, indent=1) + "\n")
    (plain / "removed.json").write_text(json.dumps(dict(removed=removed, orphans=orphans)) + "\n")

    log.write_text("")
    _convert(plain, net_out, log)
    if splits:
        # 포켓 길이는 정지선부터 잰 길이여야 한다. 자르는 위치는 교차로 중심 기준이라 교차로 영역만큼
        # 짧아지므로, 만든 뒤 재서 모자란 만큼 더 앞에서 자르고 다시 만든다 (보통 한 번이면 맞음).
        by_id = {l["id"]: l for l in links}
        for _ in range(2):
            got = sumolib.net.readNet(str(net_out))
            fix = {}
            for eid, s in splits.items():
                short = plans[eid]["length"] - got.getEdge(s["id_after"]).getLength()
                limit = -(corrections._length(by_id[eid]["shape"]) - corrections.MIN_REST)
                if abs(short) > 1.0 and s["pos"] - short >= limit:
                    fix[eid] = s["pos"] - short
            if not fix:
                break
            for eid, pos in fix.items():
                splits[eid]["pos"] = pos
            write_plain(links, nodes, clusters, tls_groups, turns, plain, splits, extra_con)
            log.write_text("")
            _convert(plain, net_out, log)
    # 2차: 단순 합류 지점 → zipper 합류 (같은 도로망을 다시 읽어 ID 유지)
    merges = merge_nodes(sumolib.net.readNet(str(net_out)))
    patch = plain / "zipper.nod.xml"
    patch.write_text("<nodes>\n" + "".join(f'    <node id="{n}" type="zipper"/>\n' for n in merges)
                     + "</nodes>\n")
    _run([sumo_bin("netconvert"), "-s", str(net_out), "-n", str(patch), "-o", str(net_out),
          *TLS_OPTIONS], log)

    changed = set_clearance_times(net_out)
    print(f"[net] 넓은 교차로 전체 적색 시간 조정: {len(changed)}개 단계 "
          f"(최대 {max(changed, default=ALLRED_MIN)}초)")
    net = sumolib.net.readNet(str(net_out))
    edges = [e for e in net.getEdges() if e.getFunction() != "internal"]
    km = sum(e.getLength() for e in edges) / 1000
    print(f"[net] {net_out}")
    print(f"      edge {len(edges)}개, 총 {km:.1f} km, 교차로 {len(net.getNodes())}개, "
          f"신호 {len(net.getTrafficLights())}개, zipper 합류 {len(merges)}곳 (경고는 {log})")


if __name__ == "__main__":
    import sys
    main(sys.argv[1:])
