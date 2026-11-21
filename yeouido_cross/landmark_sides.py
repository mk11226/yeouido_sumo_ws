"""랜드마크마다 건물 쪽 도로와 '반대 방향' 도로를 함께 찾는다 (양방향 출발·도착).

yeouido/landmarks.py 는 랜드마크를 가장 가까운 도로 한 방향(건물 쪽 차로)에만 붙인다.
그러면 반대 방향에서 오는 차는 그 차로에 들어가려고 블록을 크게 돌아야 한다
(예: 노량진수산시장은 노들로 동쪽 방향에만 붙어 있어 서쪽에서 오면 수 km를 돎).
여기서는 같은 지점 앞을 반대 방향으로 지나는 도로도 찾아서, 경로를 만들 때 둘 중 가까운 쪽을
출발·도착 지점으로 쓸 수 있게 한다. 도로망은 바꾸지 않는다.

반대 방향 도로 고르는 기준
  - 랜드마크에서 MAX_DIST (또는 건물 쪽 도로까지 거리 + 60 m) 안, 승용차 도로 (지하차도·고가도로 제외)
  - 진행 방향이 건물 쪽 도로와 반대 (가장 가까운 점에서 방향 벡터 내적 < -0.7)
  - 출발로 쓰려면 그 도로에서 도로망 다른 곳으로 나갈 수 있어야 하고 (can_depart),
    도착으로 쓰려면 도로망 다른 곳에서 그 도로로 들어올 수 있어야 한다 (can_arrive).
    예: 노량진수산시장 앞 노들로 서쪽 방향은 지도 동쪽 경계 밖에서만 들어오므로 출발만 가능.
  - 조건을 만족하는 것 중 랜드마크에서 가장 가까운 것
"""
import json
import math

from yeouido.landmarks import RESULT as LANDMARK_FILE

MAX_DIST = 120.0        # 반대 방향 도로를 찾는 반경 (m)


def _direction(lane, pos):
    """차로 위 pos 지점의 진행 방향 단위 벡터."""
    import sumolib
    shape = lane.getShape()
    a = sumolib.geomhelper.positionAtShapeOffset(shape, max(pos - 2.0, 0.0))
    b = sumolib.geomhelper.positionAtShapeOffset(shape, min(pos + 2.0, lane.getLength()))
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy) or 1.0
    return dx / n, dy / n


def _reachability(net):
    """(들어올 수 있는 도로, 나갈 수 있는 도로) — 도로망에서 가장 많이 이어진 도로 기준."""
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
    return reach(lambda e: e.getOutgoing()), reach(lambda e: e.getIncoming())


def landmark_sides(net, lms=None):
    """{랜드마크: [dict(edge, pos, side, dist, can_depart, can_arrive), ...]}.
    side 는 "건물 쪽" / "반대쪽". 반대쪽이 없으면 건물 쪽만. 건물 쪽은 늘 출발·도착 둘 다 가능."""
    lms = lms or json.loads(LANDMARK_FILE.read_text())
    enter, leave = _reachability(net)
    out = {}
    for name, lm in lms.items():
        main = net.getEdge(lm["edge"])
        x, y = lm["x"], lm["y"]
        main_dir = _direction(main.getLanes()[0], lm["pos"])
        sides = [dict(edge=main, pos=lm["pos"], side="건물 쪽", dist=lm["dist_m"],
                      can_depart=True, can_arrive=True)]
        best = None
        for edge, dist in net.getNeighboringEdges(x, y, max(MAX_DIST, lm["dist_m"] + 60.0)):
            etype = edge.getType() or ""
            if edge is main or not edge.allows("passenger") or etype.endswith((".001", ".002")):
                continue
            can_arrive, can_depart = edge in enter, edge in leave
            if not (can_arrive or can_depart):
                continue
            lane = edge.getLanes()[0]
            pos, _ = lane.getClosestLanePosAndDist((x, y))
            d = _direction(lane, pos)
            if d[0] * main_dir[0] + d[1] * main_dir[1] > -0.7:
                continue                       # 반대 방향이 아님
            if best is None or dist < best["dist"]:
                best = dict(edge=edge, pos=pos, side="반대쪽", dist=dist,
                            can_depart=can_depart, can_arrive=can_arrive)
        if best:
            sides.append(best)
        out[name] = sides
    return out


def main():
    from yeouido.config import NET, sumo_home
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET))
    for name, sides in landmark_sides(net).items():
        print(f"{name:8s} " + "  |  ".join(
            f"{s['side']} {s['edge'].getID()} {s['edge'].getName() or '-'} ({s['dist']:.0f} m, "
            f"{'출발' if s['can_depart'] else ''}{'·' if s['can_depart'] and s['can_arrive'] else ''}"
            f"{'도착' if s['can_arrive'] else ''})" for s in sides))


if __name__ == "__main__":
    main()
