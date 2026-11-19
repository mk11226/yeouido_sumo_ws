"""도로망 수정 목록 (yeouido/corrections.json) — 검수로 확인한 것만 원본 위에 덧붙인다.

국가표준노드링크 원본은 그대로 두고, 검수 도구(inspection/)에서 항공사진으로 확인한 차이만
이 목록에 적는다. build_net 이 plain 파일을 쓸 때 반영하며, 목록이 비어 있으면 도로망은
수정 전과 같다 (python -m yeouido.build_net --no-corrections 와 같음).

지금 지원하는 수정 (반영 순서대로)
  keep_links  : 1차로 골목길 규칙(build_net.is_minor_alley)으로 빠지는 원본 링크 중 실제로 쓰이는 도로를 남긴다.
                원본 노드링크 링크 ID 그대로 적는다 (예: 한쪽 방향만 2차로라 남고 반대 방향 1차로가 빠진 길).
  join_junctions: 노드링크가 둘 이상으로 나눈 교차로를 하나로 합친다. nodes 에 적은 노드가 속한
                교차로 묶음(자동으로 합친 묶음)을 통째로 하나로 묶는다. 묶음 사이를 잇던 짧은 링크는
                교차로 내부가 되어 도로로 남지 않고, netconvert 가 그 링크를 지나던 차로 연결을
                합친 교차로를 바로 지나는 회전으로 만든다 (예: A 진입 → 내부 링크 → B 에서 좌회전
                → A 진입로에서 바로 좌회전). 신호도 합친 교차로 하나로 다시 정한다.
                높이가 다른 노드(고가·지하)는 합치지 않는다.
                  connect : [[진입로, 진출로], …] — 원래 서로 떨어진 노드라 사이 링크가 없어 netconvert 가
                            만들지 않는 회전을 직접 잇는다 (도로 단위, 차로는 lane_moves 로 정함).
  remove_turns: 노드링크 회전 제한(TURNINFO) 중 실제와 다른 것을 지운다 (from 도로 → to 도로).
                지우면 그 회전은 처음부터 제한이 없던 것처럼 netconvert 가 보통 교차로처럼 잇는다.
  remove_links: 실제로 없는 도로(한 방향 링크 하나)를 뺀다. 교차로 합치기가 끝난 뒤 빼고,
                그 때문에 끊겨 떨어진 조각은 기존 규칙(연결된 가장 큰 도로망만 남김)대로 같이 빠진다.
                교차로 안을 잇는 짧은 링크, 랜드마크가 붙은 도로는 빼지 않는다.
  add_links   : 노드링크에 없는 도로(한 방향)를 새로 만든다. 차로 수 수정 뒤, 포켓 앞에 넣는다.
                from 도로의 끝 교차로에서 시작해 to 도로의 시작 교차로로 이어진다.
                  from_at : from 도로 끝에서 이 길이(m)만큼 앞에서 갈라져 나온다 (없으면 끝 교차로에서)
                  to_at   : to 도로 시작에서 이 길이(m)만큼 뒤에 붙는다 (없으면 시작 교차로로)
                  replace : true 면 from 도로의 갈라지는 점 뒤쪽을 없앤다 (from_at 과 같이 씀).
                            길이 갈라지는 게 아니라, 원래 도로가 새 도로 쪽으로 꺾여 이어져야 할 때.
                            from 도로는 원래 ID 그대로 갈라지는 점에서 끝난다.
                  via     : 지나는 점 [[경도, 위도], …] (검수 화면에서 오른쪽 클릭 → 좌표 복사)
                  lanes 차로 수 (기본 1), speed 제한속도 km/h (기본 from 도로와 같음), name 도로명 (기본 to 도로명)
                갈라지는 곳에서 도로를 자르면 앞쪽 조각이 <도로 ID>_a 가 되고, 교차로 쪽 조각이 원래 ID를 지킨다
                (그래서 그 도로의 포켓·차로 방향 수정은 그대로 맞다).
  lane_counts : 도로 전체의 차로 수. 노드링크 값 대신 이 값으로 만든다 (1~8).
                중앙선 쪽(왼쪽) 가장자리는 그대로 두고 바깥쪽(오른쪽)으로만 넓히거나 좁힌다
                (도로선을 바뀐 폭의 절반만큼 오른쪽으로 옮김 → 반대 방향 도로를 파고들지 않음).
                도로 우선순위(교차로 양보 관계)는 원래 차로 수 기준 그대로 둔다.
                side="left"이면 오른쪽 가장자리를 유지하고 왼쪽으로 확장한다 (기본 right).
  turn_pockets: 교차로 직전에 회전 전용 차로(포켓)가 생기는 진입로.
                노드링크는 링크마다 차로 수가 하나뿐이라, 정지선 앞에서 늘어나는 차로가 빠지고
                맨 왼쪽(오른쪽) 차로가 직진·좌회전(우회전) 공용이 된다.
                도로 끝 length m 를 잘라(<도로 ID>_pocket) 차로를 늘리고, 차로별 연결을 직접 정한다.
                포켓 구간도 차로 수 수정처럼 중앙선 쪽 가장자리는 그대로, 바깥쪽으로만 넓힌다.
                  side "left" : 새 맨 왼쪽 차로 = 좌회전(U턴 포함) 전용, 원래 차로에서 좌회전을 뺀다
                  side "right": 새 맨 오른쪽 차로 = 우회전 전용, 원래 차로에서 우회전을 뺀다
                (원래 차로가 그 회전만 하던 차로면 그대로 둔다 — 회전 차로가 하나 더 생김)
                좌회전 전용 차로가 생기면 신호(opposites)에 보호 좌회전 단계가 생긴다.
  lane_moves  : 정지선 차로 하나가 교차로에서 갈 수 있는 방향 (s 직진, l 좌회전, r 우회전).
                lane 은 정지선 차로 번호 (0 = 맨 오른쪽, 포켓이 있으면 포켓 차로까지 센 번호).
                없던 방향은 그 방향으로 가는 옆 차로의 연결을 빌려 온다. 도로 단위로 갈 수 있는
                곳은 늘지 않는다 (없는 방향은 추가 못 함). 그 방향으로 가던 마지막 차로에서 빼면 그 회전은 없어진다.
                to (선택): 그 차로가 갈 도로 ID 목록. 같은 방향(예: 좌회전)으로 갈 수 있는 도로가 둘 이상일 때
                그중 일부로만 가게 한다. 목록에 없는 도로로 가던 연결은 뺀다.

    {"keep_links":     [{"edge": "1160017603", "why": "...", "checked": "..."}],
     "join_junctions": [{"nodes": ["1180038801", "1180038706"], "connect": [["1180088700", "1180087100"]],
                         "why": "...", "checked": "..."}],
     "remove_turns": [{"from": "1180037605", "to": "1180035701", "why": "...", "checked": "..."}],
     "remove_links": [{"edge": "1140068200", "why": "...", "checked": "..."}],
     "add_links":    [{"id": "new_목동교_진출", "from": "1140068000", "from_at": 150, "to": "1140066300",
                       "lanes": 1, "via": [[126.8838, 37.5314]], "why": "...", "checked": "..."}],
     "lane_counts":  [{"edge": "1180038001", "lanes": 3, "why": "...", "checked": "..."}],
     "turn_pockets": [{"edge": "1180031502", "side": "left", "length": 40, "why": "...", "checked": "..."}],
     "lane_moves":   [{"edge": "1180031502", "lane": 3, "moves": ["s", "l"], "why": "...", "checked": "..."},
                      {"edge": "1180075600", "lane": 5, "moves": ["l"], "to": ["1180016204"], "why": "...", "checked": "..."}]}
    why     : 근거 (예: "항공사진: 정지선 앞 좌회전 포켓, 좌회전 화살표")
    checked : 검수 카드·검수자·날짜

차로 연결 규칙 (수정한 모든 곳: 차로 수, 포켓, 차로 방향, 회전 제한 지우기, 새 도로 — 그 도로와,
그 도로가 시작하는 교차로로 들어오는 진입로)
  1. 같은 도로로 가는 차로끼리 엇갈리지 않는다 (오른쪽 차로 → 오른쪽 도착 차로).
  2. 도착 차로가 넉넉하면 한 차로에 둘이 몰리지 않게 하나씩 나눠 준다.
     단, 신호 없는 합류(다른 도로도 직진해 들어옴)는 그 도로 차로와 겹치지 않게 순서만 맞춘다.
  3. 차로 방향을 켜면 그 방향으로 가는 가장 가까운 옆 차로가 가는 도로로만 잇는다.

건너뛴 항목은 dict(type, edge, side|lane, why) 로 알린다.
"""
import json
import math
from pathlib import Path

CORRECTIONS = Path(__file__).resolve().parent / "corrections.json"
KINDS = ("keep_links", "join_junctions", "remove_turns", "remove_links", "add_links", "lane_counts", "turn_pockets", "lane_moves")
POCKET_SUFFIX = "_pocket"
MIN_REST = 10.0     # 포켓을 자르고 앞쪽에 남아야 하는 최소 길이 (m)
MAX_LANES = 8
LANE_WIDTH = 3.2    # netconvert 기본 차로 폭 (m) — 차로 수를 바꿀 때 도로선을 옮기는 양
SIDES = {"left": {"l", "L", "t", "T"}, "right": {"r", "R"}}   # 포켓 차로가 맡는 회전 (SUMO dir)
GROUP = {"s": "s", "l": "l", "L": "l", "t": "l", "T": "l", "r": "r", "R": "r"}   # SUMO dir → 직·좌·우
MOVE_NAME = {"s": "직진", "l": "좌회전", "r": "우회전"}


def load():
    data = json.loads(CORRECTIONS.read_text()) if CORRECTIONS.exists() else {}
    return {k: data.get(k, []) for k in KINDS}


def save(data):
    CORRECTIONS.write_text(json.dumps({k: data.get(k, []) for k in KINDS}, ensure_ascii=False, indent=2) + "\n")


def label(s):
    """건너뛴 항목 → 사람이 읽는 이름."""
    if s["type"] == "join_junctions":
        return f"교차로 합치기 {s['edge']}"
    if s["type"] == "keep_links":
        return f"{s['edge']} 원본 도로 살리기"
    if s["type"] == "turn_pockets":
        return f"{s['edge']} {'좌회전' if s['side'] == 'left' else '우회전'} 포켓"
    if s["type"] == "lane_moves":
        return f"{s['edge']} 차로 {s['lane']} 방향"
    if s["type"] == "remove_links":
        return f"{s['edge']} 도로 삭제"
    if s["type"] == "remove_turns":
        return f"{s['from']} → {s['to']} 회전 제한 지우기"
    if s["type"] == "add_links":
        return f"새 도로 {s['edge']}"
    return f"{s['edge']} 차로 수"


def _cut(shape, at):
    """위경도 선을 시작에서 at m 지점에서 둘로 자른다 → (앞 조각, 뒤 조각, 자른 점)."""
    done = 0.0
    for i, (a, b) in enumerate(zip(shape, shape[1:])):
        seg = _length([a, b])
        if seg > 0 and done + seg >= at:
            t = (at - done) / seg
            p = [round(a[0] + (b[0] - a[0]) * t, 7), round(a[1] + (b[1] - a[1]) * t, 7)]
            return shape[:i + 1] + [p], [p] + shape[i + 1:], p
        done += seg
    return shape, [shape[-1]], shape[-1]


def add_links(links, nodes, entries, reserved=()):
    """새 도로 붙이기 → (링크 목록, 노드, 만든 도로 ID, 자른 도로 {원래 ID: 앞 조각 ID}, 건너뛴 항목).
    nodes 는 고친 사본을 돌려준다. 자른 도로로 들어오던 회전 제한은 build_net 이 앞 조각 ID로 바꾼다."""
    links, nodes = list(links), dict(nodes)
    made, cut, skipped = [], {}, []

    def split(eid, at, which, drop_rest=False):
        """eid 도로를 at m 지점에서 자르고 새 점 ID 를 돌려준다 (앞 조각 = <ID>_a).
        drop_rest 면 뒤 조각을 버리고 앞 조각이 원래 ID 를 지킨다."""
        i = next(k for k, l in enumerate(links) if l["id"] == eid)
        l = links[i]
        a, b, p = _cut(l["shape"], at)
        nid = f"N{c['id']}_{which}"
        nodes[nid] = dict(type="000", name="", lon=p[0], lat=p[1])
        if drop_rest:
            links[i] = dict(l, to_node=nid, shape=a, length=round(_length(a), 1))
            return nid, p
        first = dict(l, id=f"{eid}_a", to_node=nid, shape=a, length=round(_length(a), 1))
        links[i:i + 1] = [first, dict(l, from_node=nid, shape=b, length=round(_length(b), 1))]
        cut[eid] = first["id"]
        return nid, p

    for c in entries:
        by_id = {l["id"]: l for l in links}
        new_id, fe, te = str(c.get("id", "")), c.get("from"), c.get("to")
        lanes, fa, ta = c.get("lanes", 1), c.get("from_at"), c.get("to_at")
        why = None
        if not new_id or new_id in by_id or any(ch in new_id for ch in ' "&<>:'):
            why = "id 가 비었거나 이미 있거나, 쓸 수 없는 글자(빈칸 \" & < > :)가 있음"
        elif fe not in by_id or te not in by_id:
            why = f"from / to 도로가 도로망에 없음 ({fe if fe not in by_id else te})"
        elif not isinstance(lanes, int) or not 1 <= lanes <= MAX_LANES:
            why = f"차로 수는 1~{MAX_LANES}"
        elif (fa and fe in reserved) or (ta and te in reserved):
            why = "랜드마크가 붙은 도로는 자를 수 없음"
        elif fa and not MIN_REST <= fa <= _length(by_id[fe]["shape"]) - MIN_REST:
            why = f"from_at 은 {MIN_REST:.0f} m ~ (from 도로 길이 - {MIN_REST:.0f} m)"
        elif ta and not MIN_REST <= ta <= _length(by_id[te]["shape"]) - MIN_REST:
            why = f"to_at 은 {MIN_REST:.0f} m ~ (to 도로 길이 - {MIN_REST:.0f} m)"
        elif c.get("replace") and not fa:
            why = "replace 는 from_at 과 같이 써야 함 (갈라지는 점 뒤쪽을 없앰)"
        elif not all(isinstance(v, (list, tuple)) and len(v) == 2 for v in c.get("via", [])):
            why = "via 는 [[경도, 위도], …]"
        if why:
            skipped.append(dict(type="add_links", edge=new_id or "(id 없음)", why=why))
            continue
        f, t = by_id[fe], by_id[te]
        if fa:
            start, p0 = split(fe, _length(f["shape"]) - fa, "s", bool(c.get("replace")))
        else:
            start, p0 = f["to_node"], f["shape"][-1]
        if ta:
            end, p1 = split(te, ta, "e")
        else:
            end, p1 = t["from_node"], t["shape"][0]
        shape = [list(p0)] + [[float(x), float(y)] for x, y in c.get("via", [])] + [list(p1)]
        links.append(dict(id=new_id, from_node=start, to_node=end, lanes=lanes, rank=f["rank"], type="000",
                          name=c.get("name", t["name"]), connect=True, max_speed=c.get("speed", f["max_speed"]),
                          length=round(_length(shape), 1), shape=shape))
        made.append(new_id)
    return links, nodes, made, cut, skipped


def join_junctions(clusters, nodes, entries, level=None):
    """교차로 합치기 → (새 묶음 목록, 합친 묶음 목록, 건너뛴 항목).
    clusters: build_net.join_clusters 의 묶음 목록 (노드 2개 이상). 묶음에 없는 노드는 혼자인 교차로.
    level   : {노드: 높이} — 높이가 다른 노드는 합치지 않는다."""
    level = level or {}
    groups = [list(g) for g in clusters]
    skipped, joined = [], []
    for c in entries:
        want = [str(x) for x in c.get("nodes", [])]
        name = "+".join(want) or "(nodes 없음)"
        why = None
        if len(want) < 2:
            why = "nodes 에 노드를 2개 이상 적어야 함"
        elif any(x not in nodes for x in want):
            why = f"도로망에 없는 노드 ({', '.join(x for x in want if x not in nodes)})"
        if not why:
            hit = [g for g in groups if any(x in g for x in want)]
            members = {x for g in hit for x in g} | set(want)
            if len({level.get(x) for x in members}) > 1:
                why = "높이가 다른 노드(고가·지하)가 섞여 있음"
            elif len(hit) == 1 and set(want) <= set(hit[0]):
                why = "이미 같은 교차로"
        if why:
            skipped.append(dict(type="join_junctions", edge=name, why=why))
            continue
        groups = [g for g in groups if g not in hit] + [sorted(members)]
        joined = [g for g in joined if not set(g) & members] + [sorted(members)]
    return groups, joined, skipped


def join_connections(entries, links, joined):
    """교차로 합치기의 connect → (넣을 교차로 안쪽 링크 목록, 건너뛴 항목).
    진입로 끝 노드 → 진출로 시작 노드로 잇는 링크 (두 노드가 같으면 필요 없음).
    차로 수는 진입로·진출로 중 적은 쪽 — 실제 회전 차로 수는 lane_moves 로 정한다."""
    by_id = {l["id"]: l for l in links}
    group = {n: i for i, g in enumerate(joined) for n in g}
    out, skipped, seen = [], [], set()
    for c in entries:
        for pair in c.get("connect", []):
            f, t = (str(x) for x in pair) if len(pair) == 2 else (None, None)
            why = None
            if f not in by_id or t not in by_id:
                why = "connect 의 도로가 도로망에 없음"
            elif group.get(by_id[f]["to_node"]) is None or group.get(by_id[f]["to_node"]) != group.get(by_id[t]["from_node"]):
                why = "진입로가 합친 교차로로 들어오고 진출로가 그 교차로에서 나가야 함"
            if why:
                skipped.append(dict(type="join_junctions", edge=f"{f}→{t}", why=why))
                continue
            a, b = by_id[f], by_id[t]
            key = (a["to_node"], b["from_node"])
            if key[0] == key[1] or key in seen:
                continue
            seen.add(key)
            shape = [a["shape"][-1], b["shape"][0]]
            out.append(dict(id=f"J{f}_{t}", from_node=key[0], to_node=key[1], lanes=min(a["lanes"], b["lanes"]),
                            rank=a["rank"], type="000", name="", connect=True, max_speed=a["max_speed"],
                            length=round(_length(shape), 1), shape=shape))
    return out, skipped


def inner_only_turns(links, joined, inner, allowed):
    """connect 로 넣은 안쪽 링크를 다른 진입로가 타고 생기는 회전 → 막을 회전 제한 목록.
    안쪽 링크 없이는 못 가고 안쪽 링크로만 갈 수 있는 (진입로, 진출로) 중 connect 에 적지 않은 것."""
    inner_ids = {l["id"] for l in inner}
    out = []
    for g in joined:
        g = set(g)
        inside = [l for l in links if l["from_node"] in g and l["to_node"] in g]
        if not any(l["id"] in inner_ids for l in inside):
            continue
        entries = [l for l in links if l["to_node"] in g and l["from_node"] not in g]
        exits = [l for l in links if l["from_node"] in g and l["to_node"] not in g]

        def reach(start, use_inner):
            seen, todo = {start}, [start]
            while todo:
                n = todo.pop()
                for l in inside:
                    if l["from_node"] == n and (use_inner or l["id"] not in inner_ids) and l["to_node"] not in seen:
                        seen.add(l["to_node"])
                        todo.append(l["to_node"])
            return seen

        for e in entries:
            before, after = reach(e["to_node"], False), reach(e["to_node"], True)
            out += [dict(node=x["from_node"], from_link=e["id"], to_link=x["id"], type="003", all_day=True,
                         inner_only=True)
                    for x in exits if x["from_node"] in after and x["from_node"] not in before
                    and (e["id"], x["id"]) not in allowed]
    return out


def carry_turns(turns, links, joined):
    """합친 교차로 안으로 들어간 링크에 걸린 회전 제한 → 교차로를 바로 지나는 회전 제한으로 옮긴다.
    netconvert 는 교차로 내부 링크를 지나던 길을 진입로 → 진출로 회전으로 만드는데, 내부 링크에 걸린
    제한은 그 링크가 없어지면서 같이 사라진다 (예: A 에서 내부 링크로 좌회전 금지 → 합친 뒤 그 너머 진출로로
    가는 회전이 새로 생김). 그 내부 링크를 지나야만 갈 수 있는 진입로 → 진출로를 대신 막는다.
    반환: 덧붙인 회전 제한 목록 (원래 제한과 같은 type)."""
    out = []
    for g in joined:
        g = set(g)
        inner = [l for l in links if l["from_node"] in g and l["to_node"] in g]
        entries = [l for l in links if l["to_node"] in g and l["from_node"] not in g]
        exits = [l for l in links if l["from_node"] in g and l["to_node"] not in g]
        ids = {l["id"] for l in inner}

        def reach(start, banned=None):
            seen, todo = {start}, [start]
            while todo:
                n = todo.pop()
                for l in inner:
                    if l["from_node"] == n and l["id"] != banned and l["to_node"] not in seen:
                        seen.add(l["to_node"])
                        todo.append(l["to_node"])
            return seen

        by_id = {l["id"]: l for l in links}
        for t in turns:
            f, to = by_id.get(t["from_link"]), by_id.get(t["to_link"])
            if not f or not to:
                continue
            if to["id"] in ids and f["id"] not in ids:      # 진입로 → 내부 링크 금지
                ahead = reach(to["to_node"])
                other = reach(f["to_node"], banned=to["id"])
                out += [dict(t, to_link=x["id"], carried=to["id"]) for x in exits
                        if x["from_node"] in ahead and x["from_node"] not in other]
            elif f["id"] in ids and to["id"] not in ids:    # 내부 링크 → 진출로 금지
                out += [dict(t, from_link=e["id"], carried=f["id"]) for e in entries
                        if f["from_node"] in reach(e["to_node"])
                        and to["from_node"] not in reach(e["to_node"], banned=f["id"])]
    return out


def link_id(eid):
    """SUMO 도로 ID → 노드링크 링크 ID (잘린 조각 _0, 자른 앞 조각 _a, 포켓 _pocket 을 뗌)."""
    return str(eid).split("_")[0]


def remove_turns(turns, entries):
    """노드링크 회전 제한 지우기 → (남은 회전 제한, 지운 개수, 건너뛴 항목)."""
    want, skipped = {}, []
    have = {(t["from_link"], t["to_link"]) for t in turns}
    for c in entries:
        key = (link_id(c.get("from")), link_id(c.get("to")))
        if key not in have:
            skipped.append(dict(type="remove_turns", edge=c.get("from"), **{"from": c.get("from"), "to": c.get("to")},
                                why="노드링크 회전 제한 목록에 없음"))
        else:
            want[key] = c
    return [t for t in turns if (t["from_link"], t["to_link"]) not in want], len(want), skipped


def remove_links(links, entries, clusters, reserved=()):
    """도로 빼기 → (남은 링크, 뺀 링크 ID, 건너뛴 항목). 끊긴 조각 정리는 build_net 이 한다.
    edge 는 링크 ID. 범위 경계에서 잘린 조각(<ID>_0, <ID>_1 …)은 조각 ID로도, 원래 ID로 한꺼번에도 뺄 수 있다."""
    group = {n: i for i, g in enumerate(clusters) for n in g}
    pieces = {}
    for l in links:
        pieces.setdefault(l["id"], []).append(l)
        base, _, k = l["id"].rpartition("_")
        if base and k.isdigit():
            pieces.setdefault(base, []).append(l)
    drop, skipped = set(), []
    for c in entries:
        eid = c["edge"]
        hit = pieces.get(eid, [])
        why = None
        if not hit:
            why = "도로망에 없는 도로"
        elif any(l["id"] in drop for l in hit):
            why = "목록에 두 번 있음"
        elif any(group.get(l["from_node"], l["from_node"]) == group.get(l["to_node"], l["to_node"]) for l in hit):
            why = "교차로 안을 잇는 링크라 뺄 수 없음"
        elif any(l["id"] in reserved for l in hit):
            why = "랜드마크가 붙은 도로"
        if why:
            skipped.append(dict(type="remove_links", edge=eid, why=why))
        else:
            drop |= {l["id"] for l in hit}
    return [l for l in links if l["id"] not in drop], sorted(drop), skipped


def _length(shape):
    """위경도 선 길이 (m, 짧은 거리용 근사)."""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(shape, shape[1:]):
        dx = (x1 - x0) * 111320 * math.cos(math.radians(y0))
        dy = (y1 - y0) * 110540
        total += math.hypot(dx, dy)
    return total


def offset_right(shape, d):
    """위경도 선을 진행 방향 오른쪽으로 d m 평행 이동 (d < 0 이면 왼쪽). 꺾인 점은 두 변의 법선 평균.
    d 에 점마다 다른 값을 목록으로 주면 그 점만큼씩 옮긴다 (build_net.separate_levels)."""
    ds = list(d) if isinstance(d, (list, tuple)) else [d] * len(shape)
    kx, ky = 111320 * math.cos(math.radians(shape[0][1])), 110540
    pts = [(x * kx, y * ky) for x, y in shape]
    normals = []
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        n = math.hypot(x1 - x0, y1 - y0) or 1.0
        normals.append(((y1 - y0) / n, -(x1 - x0) / n))   # 오른쪽 법선
    out = []
    for i, (x, y) in enumerate(pts):
        a, b = normals[max(i - 1, 0)], normals[min(i, len(normals) - 1)]
        nx, ny = a[0] + b[0], a[1] + b[1]
        n = math.hypot(nx, ny)
        nx, ny = (nx / n, ny / n) if n > 1e-9 else a
        scale = ds[i] / max(nx * a[0] + ny * a[1], 0.5)        # 꺾인 곳은 폭이 유지되게 조금 더 (최대 2배)
        out.append([(x + nx * scale) / kx, (y + ny * scale) / ky])
    return out


def lane_counts(links, entries):
    """도로 차로 수 바꾸기 → (새 링크 목록, 건너뛴 항목). 원래 값은 lanes0 에 남긴다.
    기본은 왼쪽 경계를 유지해 오른쪽으로 확장한다. side=left는 오른쪽 경계를 유지한다."""
    want, skipped = {}, []
    sides = {}
    ids = {l["id"] for l in links}
    for c in entries:
        eid, n = c["edge"], c.get("lanes")
        why = None
        if eid not in ids:
            why = "도로망에 없는 도로"
        elif not isinstance(n, int) or not 1 <= n <= MAX_LANES:
            why = f"차로 수는 1~{MAX_LANES}"
        elif eid in want:
            why = "목록에 두 번 있음"
        elif c.get('side', 'right') not in ('left', 'right'):
            why = "차로 확장 방향은 left 또는 right"
        if why:
            skipped.append(dict(type="lane_counts", edge=eid, why=why))
        else:
            want[eid] = n
            sides[eid] = c.get('side', 'right')
    out = []
    for l in links:
        if l["id"] in want:
            shift = (want[l["id"]] - max(l["lanes"], 1)) * LANE_WIDTH / 2
            if sides[l['id']] == 'left':
                shift = -shift
            l = dict(l, lanes=want[l["id"]], lanes0=l["lanes"], shape=offset_right(l["shape"], shift))
        out.append(l)
    return out, skipped


def turn_pockets(links, entries, reserved=()):
    """build_net 의 최종 링크 중 수정 목록에 있는 도로 → 도로별 포켓 계획.

    reserved: 위치가 바뀌면 안 되는 도로 (랜드마크가 붙은 도로 등) — 건너뛴다.
    반환: ({도로 ID: dict(length, left, right)}, 건너뛴 항목)
    """
    by_id = {l["id"]: l for l in links}
    plans, skipped = {}, []
    for p in entries:
        eid, side = p["edge"], p.get("side", "left")
        length, extra = float(p.get("length", 40)), int(p.get("lanes", 1))
        l = by_id.get(eid)
        why = None
        if side not in SIDES:
            why = f"알 수 없는 쪽 '{side}'"
        elif l is None:
            why = "도로망에 없는 도로"
        elif eid in reserved:
            why = "랜드마크가 붙은 도로 (도로를 자르면 위치가 바뀜)"
        elif _length(l["shape"]) < length + MIN_REST:
            why = f"도로가 짧음 ({_length(l['shape']):.0f} m < 포켓 {length:.0f} m + {MIN_REST:.0f} m)"
        elif plans.get(eid, {}).get(side):
            why = "목록에 두 번 있음"
        if why:
            skipped.append(dict(type="turn_pockets", edge=eid, side=side, why=why))
            continue
        plan = plans.setdefault(eid, dict(length=0.0, left=0, right=0))
        plan["length"] = max(plan["length"], length)   # 좌·우 포켓이 같이 있으면 긴 쪽에서 자른다
        plan[side] = extra
    return plans, skipped


def split_spec(link, plan):
    """plain 도로(edg.xml)에 넣을 <split> 값."""
    total = max(link["lanes"], 1) + plan["left"] + plan["right"]
    return dict(pos=-plan["length"], lanes=total, id_after=link["id"] + POCKET_SUFFIX)


def _con(f, t, fl, tl):
    return f'    <connection from="{f}" to="{t}" fromLane="{fl}" toLane="{tl}"/>'


def _pockets(net0, plans):
    """포켓: 자른 곳의 연결 줄 + 정지선 차로별 연결표 {포켓 도로: [(차로, 도착 도로, 도착 차로, dir)]}.
    그 회전을 하는 연결이 없으면 그 쪽은 빼고 plans 도 그에 맞게 고친다."""
    lines, tables, skipped = [], {}, []
    for eid in list(plans):
        plan, e = plans[eid], net0.getEdge(eid)
        orig = [[(c.getTo().getID(), c.getToLane().getIndex(), c.getDirection())
                 for c in ln.getOutgoing()] for ln in e.getLanes()]
        for side in ("left", "right"):
            if plan[side] and not any(d in SIDES[side] for moves in orig for _, _, d in moves):
                skipped.append(dict(type="turn_pockets", edge=eid, side=side, why="그 쪽으로 회전하는 연결이 없음"))
                plan[side] = 0
        if not plan["left"] and not plan["right"]:
            del plans[eid]
            continue
        n, R, Lf = len(orig), plan["right"], plan["left"]
        pocket = eid + POCKET_SUFFIX
        # 자른 곳: 원래 차로 i → 포켓 도로 i+R, 새 오른쪽 차로는 맨 오른쪽에서, 새 왼쪽 차로는 맨 왼쪽에서
        lines += [_con(eid, pocket, 0, j) for j in range(R)]
        lines += [_con(eid, pocket, i, i + R) for i in range(n)]
        lines += [_con(eid, pocket, n - 1, R + n + k) for k in range(Lf)]
        # 교차로: 포켓 차로는 그 회전만, 원래 차로는 그 회전을 뺀다 (빼면 갈 곳이 없어지는 차로는 그대로)
        turns = {side: sorted({(t, tl, d) for moves in orig for t, tl, d in moves if d in SIDES[side]})
                 for side in SIDES}
        rows = []
        for j in range(R):
            rows += [(j, t, tl, d) for t, tl, d in turns["right"]]
        for k in range(Lf):
            rows += [(R + n + k, t, tl, d) for t, tl, d in turns["left"]]
        drop = (SIDES["left"] if Lf else set()) | (SIDES["right"] if R else set())
        for i, moves in enumerate(orig):
            keep = [m for m in moves if m[2] not in drop] or moves
            rows += [(i + R, t, tl, d) for t, tl, d in keep]
        tables[pocket] = (n + R + Lf, rows)
    return lines, tables, skipped


def _set_moves(rows, lane, want, to_lanes):
    """정지선 차로 lane 의 방향을 want(직·좌·우 집합)로 → (새 연결표, 안 되는 이유)."""
    have = {GROUP.get(d) for fl, _, _, d in rows if fl == lane}
    new = [r for r in rows if r[0] != lane or GROUP.get(r[3]) in want]
    for g in sorted(want - have):
        donors = [r for r in rows if GROUP.get(r[3]) == g]
        if not donors:
            return None, f"{MOVE_NAME[g]}으로 갈 길이 없음 (도로가 없거나 노드링크 회전 금지)"
        # 그 방향으로 가는 가장 가까운 옆 차로가 가는 도로만 따라간다
        # (같은 "직진" 이라도 반대편 끝 차로가 가는 빠지는 길까지 붙이지 않는다)
        # 도착 차로는 옮긴 만큼 옆으로, 다른 차로가 이미 들어가는 차로는 되도록 피한다
        near = min(abs(r[0] - lane) for r in donors)
        for to in sorted({r[1] for r in donors if abs(r[0] - lane) == near}):
            fl, _, tl, d = min((r for r in donors if r[1] == to), key=lambda r: abs(r[0] - lane))
            ideal = tl + lane - fl
            used = {r[2] for r in new if r[1] == to}
            best = min(range(to_lanes[to]), key=lambda k: (k in used, abs(k - ideal)))
            new.append((lane, to, best, d))
    # 그 방향으로 가던 마지막 차로에서 빼면 그 회전은 없어진다 (실제로 못 가는 회전 = 회전 금지와 같음)
    return sorted(set(new)), None   # 정리(_uncross)는 connections 에서 한꺼번에


def _uncross(rows, to_lanes, shared=()):
    """같은 도착 도로로 가는 연결을 정리한다: 엇갈리지 않게(오른쪽 차로 → 오른쪽 도착 차로),
    도착 차로가 넉넉하면 한 차로에 둘이 몰리지 않게 이어진 도착 차로에 하나씩 나눠 준다.
    shared: 다른 진입로도 직진해 들어오는 도착 도로 (합류) — 나눠 주면 그 진입로 차로와 겹치므로 순서만 맞춘다."""
    by_to = {}
    for r in sorted(set(rows), key=lambda r: (r[0], r[2])):
        by_to.setdefault(r[1], []).append(r)
    out = []
    for to, rs in by_to.items():
        lanes = [r[0] for r in rs]
        if to not in shared and len(set(lanes)) == len(lanes) <= to_lanes[to]:
            # 정지선 차로마다 연결 하나: 지금 쓰는 맨 오른쪽 도착 차로부터 차례로 (넘치면 왼쪽 끝에 맞춤)
            start = min(min(r[2] for r in rs), to_lanes[to] - len(rs))
            out += [(fl, to, start + k, d) for k, (fl, _, _, d) in enumerate(rs)]
        else:   # 한 차로가 여러 도착 차로로 가거나 도착 차로가 모자라면 순서만 맞춘다
            out += [(fl, to, tl, d) for (fl, _, _, d), tl in zip(rs, sorted(r[2] for r in rs))]
    return sorted(out)


def connections(net0, plans, moves, edited=()):
    """포켓·차로 방향 수정을 차로별 연결 줄(nl.con.xml)로.

    net0  : 차로 수 수정만 반영해 만든 도로망 (sumolib) — 원래 차로마다 어디로 가는지 읽는다.
    moves : lane_moves 항목. 포켓이 있는 도로는 포켓 차로 기준 번호.
    edited: 그 밖에 고친 도로 (차로 수, 회전 제한 지우기, 새 도로). 그 도로와, 그 도로가 시작하는
            교차로로 들어오는 진입로도 아래 규칙으로 연결을 다시 정리한다.
    수정한 곳의 차로 연결은 항상 (_uncross): 엇갈리지 않게, 도착 차로가 넉넉하면 하나씩 나눠서.
    반환: (연결 줄 목록, 건너뛴 항목) — 연결을 정한 도로는 그 도로의 연결 전부를 적는다.
    """
    lines, tables, skipped = _pockets(net0, plans)

    def table(eid):
        e = net0.getEdge(eid)
        return (e.getLaneNumber(), [(ln.getIndex(), c.getTo().getID(), c.getToLane().getIndex(), c.getDirection())
                                    for ln in e.getLanes() for c in ln.getOutgoing()])
    seen, orig = set(), {}
    for m in moves:
        eid, lane, want = m["edge"], m.get("lane"), set(m.get("moves") or [])
        skip = dict(type="lane_moves", edge=eid, lane=lane)
        approach = eid + POCKET_SUFFIX if eid in plans else eid
        if approach not in tables:
            if not net0.hasEdge(eid):
                skipped.append(dict(skip, why="도로망에 없는 도로"))
                continue
            tables[approach] = table(eid)
        n, rows = tables[approach]
        first = orig.setdefault(approach, rows)   # 이 진입로를 고치기 전 연결 (다른 차로 수정으로 빠진 도로도 빌려 옴)
        why = None
        if not isinstance(lane, int) or not 0 <= lane < n:
            why = f"차로 {lane}번이 없음 (정지선 {n}차로)"
        elif not want or not want <= set(MOVE_NAME):
            why = "방향은 s·l·r 중 하나 이상"
        elif (eid, lane) in seen:
            why = "목록에 두 번 있음"
        elif "to" in m and not (isinstance(m["to"], list) and m["to"]):
            why = "to 는 도로 ID 목록"
        else:
            to_lanes = {t: net0.getEdge(t).getLaneNumber() for t in {r[1] for r in rows + first}}
            new, why = _set_moves(rows, lane, want, to_lanes)
            if not why and m.get("to"):
                only = {str(t) for t in m["to"]}
                # to 의 도로로 이 차로가 아직 안 가면, 그 도로로 가는 가장 가까운 옆 차로의 연결을 빌려 온다
                # (예: 2개 차로 우회전 — 옆 차로만 가던 우회전 도로를 이 차로도 가게)
                for t in sorted({r[1] for r in rows + first if r[1] in only or link_id(r[1]) in only}):
                    if any(r[0] == lane and r[1] == t for r in new):
                        continue
                    donors = [r for r in first if r[1] == t and GROUP.get(r[3]) in want]
                    if not donors:
                        continue
                    fl, _, tl, d = min(donors, key=lambda r: abs(r[0] - lane))
                    used = {r[2] for r in new if r[1] == t}
                    ideal = tl + lane - fl
                    best = min(range(to_lanes[t]), key=lambda k: (k in used, abs(k - ideal)))
                    new.append((lane, t, best, d))
                new = [r for r in new if r[0] != lane or r[1] in only or link_id(r[1]) in only]
                if not any(r[0] == lane for r in new):
                    why = "to 의 도로로 가는 연결이 없음 (그 방향으로 갈 수 있는 도로인지 확인)"
        if why:
            skipped.append(dict(skip, why=why))
            continue
        seen.add((eid, lane))
        tables[approach] = (n, new)
    forced = set(tables)          # 포켓·차로 방향을 고친 진입로: 연결을 늘 직접 적는다
    for eid in edited:
        if not net0.hasEdge(eid):
            continue
        e = net0.getEdge(eid)
        for a in {eid} | {x.getID() for x in e.getFromNode().getIncoming() if x.getFunction() != "internal"}:
            if (a + POCKET_SUFFIX if a in plans else a) not in tables:
                tables[a] = table(a)
    for approach, (n, rows) in tables.items():
        root = approach[:-len(POCKET_SUFFIX)] if approach.endswith(POCKET_SUFFIX) else approach
        targets = {r[1] for r in rows}
        to_lanes = {t: net0.getEdge(t).getLaneNumber() for t in targets}
        # 합류: 신호 없는 곳에서 다른 진입로도 그 도로로 "직진" 해 들어오는 경우 (같은 때 같은 차로로 들어와 겹침).
        # 신호 교차로는 방향마다 녹색 시간이 달라서 나눠 줘도 된다 (예: 2개 차로 좌회전 → 도착 차로 2개)
        signal = net0.getEdge(root).getToNode().getType() == "traffic_light"
        shared = set() if signal else {
            t for t in targets
            if any(c.getDirection() == "s" for x, cs in net0.getEdge(t).getIncoming().items()
                   if x.getID() != root for c in cs)}
        fixed = _uncross(rows, to_lanes, shared)
        # 나머지 고친 도로는 정리해서 바뀔 때만 적는다 (그대로면 netconvert 에 맡김 → 합류 차로 나눔 유지)
        if approach in forced or fixed != sorted(set(rows)):
            lines += [_con(approach, t, fl, tl) for fl, t, tl, _ in fixed]
    return lines, skipped
