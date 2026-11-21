"""여의도·영등포 핀포인트 13곳 사이의 모든 AV 경로를 만든다 (13 × 12 = 156개, 쌍마다 최단거리 하나).

    python -m yeouido_cross.build_routes          # 경로 생성 → yeouido_cross/output/
    python -m yeouido_cross.build_routes --gui    # 생성 후 sumo-gui로 경로 보기

핀포인트 13곳
  여의도 섬   랜드마크 1~4 (국회의사당, IFC 서울, 63빌딩, 여의도공원) + 북쪽 다리 게이트 A 서강·B 마포·C 원효대교
  영등포 본토 랜드마크 5~7 (영등포역, 당산역, 노량진수산시장) + 경계 게이트 D 양화대교·E 오목교·F 거리공원
  (샛강 다리 위 게이트 ㄱ 여의교·ㄴ 서울교·ㄷ 여의2교는 섬도 본토도 아니라서 넣지 않는다)
출발·도착 위치는 YeouidoEnv 와 같다: 랜드마크는 그 지점, 게이트는 출발 = 들어오는 도로 시작, 도착 = 나가는 도로 끝.

여의도 섬과 영등포 본토는 샛강을 건너는 연결로 네 개로만 이어진다.
  여의2교 (국회대로) · 서울교 (경인로) · 여의교 (여의대방로) — 양방향
  여의하류 램프 (노들로 당산 쪽 → 여의서로, 도로명 "올림픽대로") — 본토 → 섬 일방통행

id: 섬↔본토 x000~, 여의도 안 i000~, 영등포 안 m000~ (같은 쪽 안 경로도 최단거리가 샛강을 건너면 bridge 에 기록)

결과 (yeouido_cross/output/):
  routes.json     경로 목록 {id, origin, destination, direction, bridge, length_m, free_time_s, straight_m, detour, edges}
                  bridge = 건너는 연결로 (안 건너면 null, 여러 개면 "+"로 연결)
                  detour = 도로 길이 / 직선거리 (가까운데 크게 돌아가는 경로를 거를 때 참고)
  routes.rou.xml  SUMO <route> 정의 (id = routes.json 의 id)
  routes.poly.xml sumo-gui 겹쳐 보기용 선 (연결로별 색, 같은 쪽 안 경로는 주황/청록)
"""
import argparse
import json
import math
import subprocess
from collections import deque
from pathlib import Path

from yeouido.build_context import POLY
from yeouido.config import NET, OUT as YEOUIDO_OUT, sumo_home
from yeouido.landmarks import RESULT as LANDMARK_FILE
from yeouido.map_counts import MAPPING
from yeouido_cross.landmark_sides import landmark_sides

HERE = Path(__file__).resolve().parent
OUT = HERE / "output"

# 샛강 연결로: 이름 → 링크 (있는 방향 모두). 다리는 표준노드링크 도로 종류 .003(교량) 중 샛강 위에 있는 것.
# 이 링크들을 빼면 섬과 본토가 끊어진다 (island_edges 로 확인).
BRIDGES = {
    "여의2교": ["1180031402", "1180031502"],                              # 국회대로
    "서울교": ["1180001501", "1180001601"],                               # 경인로
    "여의교": ["1190006902", "1190006903", "1190007002", "1190007003"],   # 여의대방로
    "여의하류램프": ["1180103300"],                                       # 노들로 → 여의서로 (일방)
}
BRIDGE_COLOR = {"여의2교": "1,0.2,0.2", "서울교": "0.1,0.5,1", "여의교": "0.1,0.75,0.2",
                "여의하류램프": "0.8,0.3,0.9"}
INNER_COLOR = {"여의도 안": "1,0.6,0", "영등포 안": "0,0.7,0.7"}   # 같은 쪽 안 경로 (주황 / 청록)

ISLAND, MAINLAND = "여의도", "영등포"
ORIGIN_MARGIN = 20.0     # 출발 지점은 edge 끝에서 이만큼 남긴다 (YeouidoEnv._insert_ego 와 같음)


def island_edges(net, seed_edge):
    """샛강 다리를 빼고 seed_edge 에서 (방향 무시하고) 닿는 도로 = 여의도 섬."""
    cut = {e for ids in BRIDGES.values() for e in ids}
    seen, queue = {seed_edge}, deque([seed_edge])
    while queue:
        e = queue.popleft()
        for n in list(e.getOutgoing()) + list(e.getIncoming()):
            if n.getID() not in cut and n not in seen:
                seen.add(n)
                queue.append(n)
    return seen


def free_time(edges):
    return sum(e.getLength() / e.getSpeed() for e in edges)


def shortest(net, a, b):
    """a → b 최단거리 경로 (도로 길이 기준, 승용차 통행·회전 제한 반영)."""
    if a == b:
        return [a]
    path, _ = net.getShortestPath(a, b, vClass="passenger")
    return path


def crossed(edges):
    ids = {e.getID() for e in edges}
    return [name for name, b in BRIDGES.items() if ids & set(b)]


def places(net, lms, island):
    """핀포인트 {이름: (쪽, 출발 후보, 도착 후보, 기준점)} — 랜드마크 7 + 게이트 6 = 13곳.

    후보 = [(edge, pos, 라벨)]. 랜드마크는 건물 쪽 도로와 반대 방향 도로 둘 다 후보다 (landmark_sides.py).
    경로를 만들 때 출발 후보 × 도착 후보 중 가장 짧은 조합을 고른다. 게이트는 후보가 하나씩
    (출발 = 들어오는 도로 시작, 도착 = 나가는 도로 끝). 기준점은 직선거리를 잴 때 쓰는 그 장소의 위치.
    """
    cut = {e for ids in BRIDGES.values() for e in ids}
    out = {}
    for n, sides in landmark_sides(net, lms).items():
        main = sides[0]["edge"]
        dep = [(s["edge"], min(max(s["pos"], 0.0), max(s["edge"].getLength() - ORIGIN_MARGIN, 0.0)), s["side"])
               for s in sides if s["can_depart"]]
        arr = [(s["edge"], s["pos"], s["side"]) for s in sides if s["can_arrive"]]
        out[n] = (ISLAND if main in island else MAINLAND, dep, arr, (lms[n]["x"], lms[n]["y"]))
    for g, v in json.loads(MAPPING.read_text())["gates"].items():
        if v["in"] in cut or v["out"] in cut:
            continue                       # 샛강 다리 위 게이트 ㄱ·ㄴ·ㄷ
        e_in, e_out = net.getEdge(v["in"]), net.getEdge(v["out"])
        out[f"게이트 {g}"] = (ISLAND if e_in in island else MAINLAND,
                             [(e_in, 0.0, "게이트")], [(e_out, e_out.getLength(), "게이트")],
                             _point(e_in, 0.0))
    return out


def _point(edge, pos):
    import sumolib
    return sumolib.geomhelper.positionAtShapeOffset(edge.getLanes()[0].getShape(),
                                                    min(max(pos, 0.0), edge.getLength()))


def _best_route(net, dep, arr):
    """출발 후보 × 도착 후보 중 가장 짧은 경로 (edges, 길이, 출발 후보, 도착 후보)."""
    best = None
    for oe, op, ol in dep:
        for de, dp, dl in arr:
            if oe is de and dp <= op:
                continue                   # 같은 도로에서 뒤쪽으로는 못 감
            edges = shortest(net, oe, de)
            if not edges:
                continue
            length = sum(e.getLength() for e in edges) - op - (de.getLength() - dp)
            if best is None or length < best[1]:
                best = (edges, length, (oe, op, ol), (de, dp, dl))
    return best


def build():
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET))
    lms = json.loads(LANDMARK_FILE.read_text())
    island = island_edges(net, net.getEdge(lms["여의도공원"]["edge"]))
    pts = places(net, lms, island)
    for which in (ISLAND, MAINLAND):
        print(f"[cross] {which}:", ", ".join(n for n, p in pts.items() if p[0] == which))

    routes, count = [], {"x": 0, "i": 0, "m": 0}
    for o, (o_side, dep, _, o_ref) in pts.items():
        for d, (d_side, _, arr, d_ref) in pts.items():
            if o == d:
                continue
            found = _best_route(net, dep, arr)
            if not found:
                raise RuntimeError(f"{o} → {d}: 경로 없음")
            edges, length, (oe, op, ol), (de, dp, dl) = found
            if o_side != d_side:
                prefix, direction = "x", f"{o_side}→{d_side}"
            else:
                prefix, direction = ("i" if o_side == ISLAND else "m"), f"{o_side} 안"
            t = free_time(edges) - op / oe.getSpeed() - (de.getLength() - dp) / de.getSpeed()
            straight = math.dist(o_ref, d_ref)
            routes.append(dict(id=f"{prefix}{count[prefix]:03d}", origin=o, destination=d,
                               direction=direction, bridge="+".join(crossed(edges)) or None,
                               length_m=round(length, 1), free_time_s=round(t, 1),
                               straight_m=round(straight, 1),
                               detour=round(length / straight, 2) if straight > 0 else None,
                               origin_side=ol, origin_pos=round(op, 1),
                               dest_side=dl, dest_pos=round(dp, 1),
                               edges=[e.getID() for e in edges]))
            count[prefix] += 1
    routes.sort(key=lambda r: ("xim".index(r["id"][0]), r["id"]))
    return net, routes


def write(net, routes):
    OUT.mkdir(exist_ok=True)
    keys = ["id", "origin", "destination", "direction", "bridge",
            "length_m", "free_time_s", "straight_m", "detour",
            "origin_side", "origin_pos", "dest_side", "dest_pos", "edges"]
    (OUT / "routes.json").write_text(json.dumps([{k: r[k] for k in keys} for r in routes],
                                                ensure_ascii=False, indent=1))

    lines = ["<routes>"]
    for r in routes:
        via = r["direction"] + (f" via {r['bridge']}" if r["bridge"] else "")
        lines.append(f'    <!-- {r["origin"]} → {r["destination"]} {via} -->')
        lines.append(f'    <route id="{r["id"]}" edges="{" ".join(r["edges"])}"/>')
    lines.append("</routes>")
    (OUT / "routes.rou.xml").write_text("\n".join(lines) + "\n")

    # 겹쳐 보기: 연결로별 색 (한쪽 안 경로는 주황/청록)
    lines = ["<additional>"]
    for r in routes:
        shape = []
        for eid in r["edges"]:
            for x, y in net.getEdge(eid).getShape():
                if not shape or (x, y) != shape[-1]:
                    shape.append((x, y))
        pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in shape)
        lines.append(f'    <poly id="{r["id"]}" type="{r["origin"]}→{r["destination"]} ({r["bridge"] or r["direction"]})" '
                     f'color="{_color(r)}" fill="0" lineWidth="4" '
                     f'layer="20" shape="{pts}"/>')
    lines.append("</additional>")
    (OUT / "routes.poly.xml").write_text("\n".join(lines) + "\n")


def _color(r):
    if r["direction"].endswith(" 안"):
        return INNER_COLOR[r["direction"]]
    return BRIDGE_COLOR[r["bridge"].split("+")[0]]


def summary(routes):
    for title, pick in (("섬↔본토", lambda r: r["id"][0] == "x"),
                        (f"{ISLAND} 안", lambda r: r["id"][0] == "i"),
                        (f"{MAINLAND} 안", lambda r: r["id"][0] == "m")):
        sel = [r for r in routes if pick(r)]
        print(f"\n[{title}] {len(sel)}개")
        for r in sel:
            via = f" via {r['bridge']}" if r["bridge"] else ""
            print(f"  {r['id']} {r['origin']:>8s} → {r['destination']:<8s} "
                  f"{r['length_m'] / 1000:4.1f}km {r['free_time_s'] / 60:4.1f}분 "
                  f"(직선 대비 {r['detour']}배){via}")


def show():
    from yeouido.view import GUI_SETTINGS, MARKERS, gui_binary, write_gui_settings, write_markers
    write_markers()
    write_gui_settings()
    subprocess.run([gui_binary("sumo-gui"), "-n", str(NET),
                    "--additional-files", f"{POLY},{MARKERS},{OUT / 'routes.poly.xml'}",
                    "--gui-settings-file", str(GUI_SETTINGS)])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gui", action="store_true", help="생성 후 sumo-gui로 경로 보기")
    args = ap.parse_args(argv)
    net, routes = build()
    write(net, routes)
    summary(routes)
    print(f"\n[cross] 경로 {len(routes)}개 → {OUT.relative_to(YEOUIDO_OUT.parent.parent)}/")
    if args.gui:
        show()


if __name__ == "__main__":
    main()
