"""도로망 수정 목록 (yeouido/corrections.json) — 검수로 확인한 것만 원본 위에 덧붙인다.

국가표준노드링크 원본은 그대로 두고, 검수 도구(inspection/)에서 항공사진으로 확인한 차이만
이 목록에 적는다. build_net 이 plain 파일을 쓸 때 반영하며, 목록이 비어 있으면 도로망은
수정 전과 같다 (python -m yeouido.build_net --no-corrections 와 같음).

지금 지원하는 수정 (반영 순서대로)
  lane_counts : 도로 전체의 차로 수. 노드링크 값 대신 이 값으로 만든다 (1~8).
                중앙선 쪽(왼쪽) 가장자리는 그대로 두고 바깥쪽(오른쪽)으로만 넓히거나 좁힌다
                (도로선을 바뀐 폭의 절반만큼 오른쪽으로 옮김 → 반대 방향 도로를 파고들지 않음).
                도로 우선순위(교차로 양보 관계)는 원래 차로 수 기준 그대로 둔다.
  turn_pockets: 교차로 직전에 회전 전용 차로(포켓)가 생기는 진입로.
                노드링크는 링크마다 차로 수가 하나뿐이라, 정지선 앞에서 늘어나는 차로가 빠지고
                맨 왼쪽(오른쪽) 차로가 직진·좌회전(우회전) 공용이 된다.
                도로 끝 length m 를 잘라(<도로 ID>_pocket) 차로를 늘리고, 차로별 연결을 직접 정한다.
                  side "left" : 새 맨 왼쪽 차로 = 좌회전(U턴 포함) 전용, 원래 차로에서 좌회전을 뺀다
                  side "right": 새 맨 오른쪽 차로 = 우회전 전용, 원래 차로에서 우회전을 뺀다
                (원래 차로가 그 회전만 하던 차로면 그대로 둔다 — 회전 차로가 하나 더 생김)
                좌회전 전용 차로가 생기면 신호(opposites)에 보호 좌회전 단계가 생긴다.
  lane_moves  : 정지선 차로 하나가 교차로에서 갈 수 있는 방향 (s 직진, l 좌회전, r 우회전).
                lane 은 정지선 차로 번호 (0 = 맨 오른쪽, 포켓이 있으면 포켓 차로까지 센 번호).
                없던 방향은 그 방향으로 가는 옆 차로의 연결을 빌려 온다. 도로 단위로 갈 수 있는
                곳은 바뀌지 않는다 (없는 방향은 추가 못 하고, 마지막 남은 차로에서는 빼지 못한다).

    {"lane_counts":  [{"edge": "1180038001", "lanes": 3, "why": "...", "checked": "..."}],
     "turn_pockets": [{"edge": "1180031502", "side": "left", "length": 40, "why": "...", "checked": "..."}],
     "lane_moves":   [{"edge": "1180031502", "lane": 3, "moves": ["s", "l"], "why": "...", "checked": "..."}]}
    why     : 근거 (예: "항공사진: 정지선 앞 좌회전 포켓, 좌회전 화살표")
    checked : 검수 카드·검수자·날짜

건너뛴 항목은 dict(type, edge, side|lane, why) 로 알린다.
"""
import json
import math
from pathlib import Path

CORRECTIONS = Path(__file__).resolve().parent / "corrections.json"
KINDS = ("lane_counts", "turn_pockets", "lane_moves")
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
    if s["type"] == "turn_pockets":
        return f"{s['edge']} {'좌회전' if s['side'] == 'left' else '우회전'} 포켓"
    if s["type"] == "lane_moves":
        return f"{s['edge']} 차로 {s['lane']} 방향"
    return f"{s['edge']} 차로 수"


def _length(shape):
    """위경도 선 길이 (m, 짧은 거리용 근사)."""
    total = 0.0
    for (x0, y0), (x1, y1) in zip(shape, shape[1:]):
        dx = (x1 - x0) * 111320 * math.cos(math.radians(y0))
        dy = (y1 - y0) * 110540
        total += math.hypot(dx, dy)
    return total


def offset_right(shape, d):
    """위경도 선을 진행 방향 오른쪽으로 d m 평행 이동 (d < 0 이면 왼쪽). 꺾인 점은 두 변의 법선 평균."""
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
        scale = d / max(nx * a[0] + ny * a[1], 0.5)        # 꺾인 곳은 폭이 유지되게 조금 더 (최대 2배)
        out.append([(x + nx * scale) / kx, (y + ny * scale) / ky])
    return out


def lane_counts(links, entries):
    """도로 차로 수 바꾸기 → (새 링크 목록, 건너뛴 항목). 원래 값은 lanes0 에 남긴다.
    중앙선 쪽 가장자리가 그대로 있게 도로선을 (늘어난 폭 / 2) 만큼 오른쪽으로 옮긴다."""
    want, skipped = {}, []
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
        if why:
            skipped.append(dict(type="lane_counts", edge=eid, why=why))
        else:
            want[eid] = n
    out = []
    for l in links:
        if l["id"] in want:
            shift = (want[l["id"]] - max(l["lanes"], 1)) * LANE_WIDTH / 2
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
        # 그 방향 도착 도로마다, 가장 가까운 차로의 연결을 이 차로 쪽으로 옮겨 붙인다
        # (도착 차로는 옮긴 만큼 옆으로, 다른 차로가 이미 들어가는 차로는 되도록 피한다)
        for to in sorted({r[1] for r in donors}):
            fl, _, tl, d = min((r for r in donors if r[1] == to), key=lambda r: abs(r[0] - lane))
            ideal = tl + lane - fl
            used = {r[2] for r in new if r[1] == to}
            best = min(range(to_lanes[to]), key=lambda k: (k in used, abs(k - ideal)))
            new.append((lane, to, best, d))
    lost = {r[1] for r in rows} - {r[1] for r in new}
    if lost:
        return None, "그 방향으로 가는 차로가 하나도 남지 않음"
    return sorted(set(new)), None


def connections(net0, plans, moves):
    """포켓·차로 방향 수정을 차로별 연결 줄(nl.con.xml)로.

    net0 : 차로 수 수정만 반영해 만든 도로망 (sumolib) — 원래 차로마다 어디로 가는지 읽는다.
    moves: lane_moves 항목. 포켓이 있는 도로는 포켓 차로 기준 번호.
    반환: (연결 줄 목록, 건너뛴 항목) — 연결을 정한 도로는 그 도로의 연결 전부를 적는다.
    """
    lines, tables, skipped = _pockets(net0, plans)
    seen = set()
    for m in moves:
        eid, lane, want = m["edge"], m.get("lane"), set(m.get("moves") or [])
        skip = dict(type="lane_moves", edge=eid, lane=lane)
        approach = eid + POCKET_SUFFIX if eid in plans else eid
        if approach not in tables:
            if not net0.hasEdge(eid):
                skipped.append(dict(skip, why="도로망에 없는 도로"))
                continue
            e = net0.getEdge(eid)
            tables[approach] = (e.getLaneNumber(), [(ln.getIndex(), c.getTo().getID(), c.getToLane().getIndex(),
                                                     c.getDirection()) for ln in e.getLanes() for c in ln.getOutgoing()])
        n, rows = tables[approach]
        why = None
        if not isinstance(lane, int) or not 0 <= lane < n:
            why = f"차로 {lane}번이 없음 (정지선 {n}차로)"
        elif not want or not want <= set(MOVE_NAME):
            why = "방향은 s·l·r 중 하나 이상"
        elif (eid, lane) in seen:
            why = "목록에 두 번 있음"
        else:
            to_lanes = {t: net0.getEdge(t).getLaneNumber() for t in {r[1] for r in rows}}
            new, why = _set_moves(rows, lane, want, to_lanes)
        if why:
            skipped.append(dict(skip, why=why))
            continue
        seen.add((eid, lane))
        tables[approach] = (n, new)
    for approach, (_, rows) in tables.items():
        lines += [_con(approach, t, fl, tl) for fl, t, tl, _ in rows]
    return lines, skipped
