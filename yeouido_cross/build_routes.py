"""여의도와 영등포(샛강 남쪽 본토)를 넘나드는 AV 경로를 만든다.

    python -m yeouido_cross.build_routes          # 경로 생성 → yeouido_cross/output/
    python -m yeouido_cross.build_routes --gui    # 생성 후 sumo-gui로 경로 보기

여의도 섬과 영등포 본토는 샛강을 건너는 연결로 네 개로만 이어진다.
  여의2교 (국회대로) · 서울교 (경인로) · 여의교 (여의대방로) — 양방향
  여의하류 램프 (노들로 당산 쪽 → 여의서로, 도로명 "올림픽대로") — 본토 → 섬 일방통행
섬 랜드마크 × 본토 랜드마크 × 방향 × 연결로마다 "그 연결로를 건너는 가장 빠른 경로"를 하나씩 만든다 (id x000~).
같은 출발·도착이라도 어느 다리로 건너느냐에 따라 경로가 달라지므로, 다리 선택(내비게이션 비교)에 쓸 수 있다.
한쪽 안에서만 다니는 경로도 만든다. 샛강을 건너지 않는 가장 빠른 경로 하나씩이고, 쓸 만한 것만 남긴다
(1 km 이상 = 학습 환경 min_trip, 경로가 직선거리의 MAX_DETOUR배 이내).
  섬 안 (id i000~)    섬 랜드마크 4곳 + 섬에 닿는 북쪽 다리 게이트 A 서강·B 마포·C 원효대교
  영등포 안 (id m000~) 본토 랜드마크 3곳 + 본토 경계 게이트 G 양화대교·H 오목교·I 거리공원
  (샛강 다리 위 게이트 D·E·F는 어느 한쪽 안이 아니라서 넣지 않는다)

결과 (yeouido_cross/output/):
  routes.json     경로 목록 {id, origin, destination, direction, bridge, fastest, length_m, free_time_s, detour, edges}
                  (한쪽 안 경로는 direction "여의도 안"/"영등포 안", bridge null)
  routes.rou.xml  SUMO <route> 정의 (id = routes.json 의 id)
  routes.poly.xml sumo-gui 겹쳐 보기용 선 (다리별 색)
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
INNER_COLOR = {"여의도 안": "1,0.6,0", "영등포 안": "0,0.7,0.7"}   # 한쪽 안 경로 (주황 / 청록)

ISLAND, MAINLAND = "여의도", "영등포"
ORIGIN_MARGIN = 20.0     # 출발 지점은 edge 끝에서 이만큼 남긴다 (YeouidoEnv._insert_ego 와 같음)
MIN_TRIP = 1000.0        # 한쪽 안 경로 최소 길이 (env/road_config.py min_trip 과 같음)
MAX_DETOUR = 2.5         # 한쪽 안 경로: 도로 길이가 직선거리의 이 배수를 넘으면 돌아가는 길이라 뺀다


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


def bridge_chains(net, island):
    """연결로마다 {방향: [edge, ...]} — 섬→본토, 본토→섬 각각 이어진 링크 순서대로 (일방이면 한 방향만)."""
    chains = {}
    for name, ids in BRIDGES.items():
        edges = [net.getEdge(i) for i in ids]
        group = set(edges)
        out = {}
        for e in edges:
            if any(p in group for p in e.getIncoming()):
                continue                       # 체인 중간 링크
            chain = [e]
            while True:
                nxt = [n for n in chain[-1].getOutgoing() if n in group]
                if not nxt:
                    break
                chain.append(nxt[0])
            before = [p for p in chain[0].getIncoming() if p not in group]
            from_island = any(p in island for p in before)
            out[f"{ISLAND}→{MAINLAND}" if from_island else f"{MAINLAND}→{ISLAND}"] = chain
        if not out or len(out) < sum(1 for e in edges if not any(p in group for p in e.getIncoming())):
            raise RuntimeError(f"{name}: 링크 방향을 정하지 못했습니다 ({list(out)})")
        chains[name] = out
    return chains


def free_time(edges):
    return sum(e.getLength() / e.getSpeed() for e in edges)


def fastest(net, a, b):
    if a == b:
        return [a]
    path, _ = net.getFastestPath(a, b, vClass="passenger")
    return path


def crossed(edges):
    ids = {e.getID() for e in edges}
    return [name for name, b in BRIDGES.items() if ids & set(b)]


def build():
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET))
    lms = json.loads(LANDMARK_FILE.read_text())

    island = island_edges(net, net.getEdge(lms["여의도공원"]["edge"]))
    side = {name: ISLAND if net.getEdge(lm["edge"]) in island else MAINLAND for name, lm in lms.items()}
    chains = bridge_chains(net, island)
    print("[cross] 여의도:", ", ".join(n for n, s in side.items() if s == ISLAND))
    print("[cross] 영등포:", ", ".join(n for n, s in side.items() if s == MAINLAND))

    routes = []
    for o, o_side in side.items():
        for d, d_side in side.items():
            if o_side == d_side:
                continue
            direction = f"{o_side}→{d_side}"
            o_edge, d_edge = net.getEdge(lms[o]["edge"]), net.getEdge(lms[d]["edge"])
            o_pos = min(max(lms[o]["pos"], 0.0), max(o_edge.getLength() - ORIGIN_MARGIN, 0.0))
            d_pos = lms[d]["pos"]
            pair = []
            for bridge, by_dir in chains.items():
                if direction not in by_dir:
                    continue                   # 일방통행 연결로
                chain = by_dir[direction]
                head, tail = fastest(net, o_edge, chain[0]), fastest(net, chain[-1], d_edge)
                if not head or not tail:
                    print(f"   ! {o} → {d} via {bridge}: 경로 없음")
                    continue
                edges = list(head[:-1]) + chain + list(tail[1:])
                over = crossed(edges)
                if over != [bridge]:
                    # 다른 연결로로 한 번 더 건너야만 하는 경로는 "이 연결로 경로"로 보지 않는다
                    print(f"   - {o} → {d} via {bridge}: {over} 를 모두 건너서 제외")
                    continue
                length = sum(e.getLength() for e in edges) - o_pos - (d_edge.getLength() - d_pos)
                t = free_time(edges) - o_pos / o_edge.getSpeed() \
                    - (d_edge.getLength() - d_pos) / d_edge.getSpeed()
                pair.append(dict(origin=o, destination=d, direction=direction, bridge=bridge,
                                 length_m=round(length, 1), free_time_s=round(t, 1),
                                 edges=[e.getID() for e in edges]))
            best = min(p["free_time_s"] for p in pair)
            for p in sorted(pair, key=lambda p: p["free_time_s"]):
                p["fastest"] = p["free_time_s"] == best
                p["detour"] = round(p["free_time_s"] / best, 2)
                p["id"] = f"x{len(routes):03d}"
                routes.append(p)
    for which, prefix in ((ISLAND, "i"), (MAINLAND, "m")):
        routes += inside_routes(net, lms, island, side, which, prefix)
    return net, routes


def inside_places(net, lms, island, side, which):
    """한쪽(섬/본토) 안 출발·도착 후보 {이름: (출발 edge, 출발 pos, 도착 edge, 도착 pos)}.
    랜드마크는 그 지점, 게이트는 YeouidoEnv 와 같이 출발 = 들어오는 도로 시작, 도착 = 나가는 도로 끝.
    샛강 다리 위 게이트(D·E·F)는 어느 한쪽 안도 아니라서 빠진다."""
    cut = {e for ids in BRIDGES.values() for e in ids}
    out = {}
    for n, lm in lms.items():
        if side[n] == which:
            e = net.getEdge(lm["edge"])
            out[n] = (e, min(max(lm["pos"], 0.0), max(e.getLength() - ORIGIN_MARGIN, 0.0)), e, lm["pos"])
    for g, v in json.loads(MAPPING.read_text())["gates"].items():
        e_in, e_out = net.getEdge(v["in"]), net.getEdge(v["out"])
        if v["in"] in cut or v["out"] in cut:
            continue
        g_side = ISLAND if (e_in in island and e_out in island) else MAINLAND
        if g_side == which:
            out[f"게이트 {g}"] = (e_in, 0.0, e_out, e_out.getLength())
    return out


def _point(edge, pos):
    import sumolib
    return sumolib.geomhelper.positionAtShapeOffset(edge.getLanes()[0].getShape(),
                                                    min(max(pos, 0.0), edge.getLength()))


def inside_routes(net, lms, island, side, which, prefix):
    places = inside_places(net, lms, island, side, which)
    print(f"[cross] {which} 안 후보:", ", ".join(places))
    routes = []
    for o, (oe, op, _, _) in places.items():
        for d, (_, _, de, dp) in places.items():
            if o == d:
                continue
            edges = fastest(net, oe, de)
            if not edges or crossed(edges):
                continue                   # 길이 없거나, 가장 빠른 길이 건너편을 거쳐 감
            length = sum(e.getLength() for e in edges) - op - (de.getLength() - dp)
            straight = math.dist(_point(oe, op), _point(de, dp))
            if length < MIN_TRIP or length > MAX_DETOUR * straight:
                print(f"   - {o} → {d}: {length:.0f} m (직선 {straight:.0f} m) 제외")
                continue
            t = free_time(edges) - op / oe.getSpeed() - (de.getLength() - dp) / de.getSpeed()
            routes.append(dict(id=f"{prefix}{len(routes):03d}", origin=o, destination=d,
                               direction=f"{which} 안", bridge=None, fastest=True, detour=1.0,
                               length_m=round(length, 1), free_time_s=round(t, 1),
                               edges=[e.getID() for e in edges]))
    return routes


def write(net, routes):
    OUT.mkdir(exist_ok=True)
    keys = ["id", "origin", "destination", "direction", "bridge", "fastest",
            "length_m", "free_time_s", "detour", "edges"]
    (OUT / "routes.json").write_text(json.dumps([{k: r[k] for k in keys} for r in routes],
                                                ensure_ascii=False, indent=1))

    lines = ["<routes>"]
    for r in routes:
        via = f"via {r['bridge']}" if r["bridge"] else r["direction"]
        lines.append(f'    <!-- {r["origin"]} → {r["destination"]} {via} -->')
        lines.append(f'    <route id="{r["id"]}" edges="{" ".join(r["edges"])}"/>')
    lines.append("</routes>")
    (OUT / "routes.rou.xml").write_text("\n".join(lines) + "\n")

    # 겹쳐 보기: 가장 빠른 경로는 굵게(위), 나머지는 가늘게
    lines = ["<additional>"]
    for r in routes:
        shape = []
        for eid in r["edges"]:
            for x, y in net.getEdge(eid).getShape():
                if not shape or (x, y) != shape[-1]:
                    shape.append((x, y))
        pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in shape)
        lines.append(f'    <poly id="{r["id"]}" type="{r["origin"]}→{r["destination"]} ({r["bridge"] or r["direction"]})" '
                     f'color="{BRIDGE_COLOR.get(r["bridge"]) or INNER_COLOR[r["direction"]]}" fill="0" lineWidth="{6 if r["fastest"] else 2}" '
                     f'layer="{20 if r["fastest"] else 10}" shape="{pts}"/>')
    lines.append("</additional>")
    (OUT / "routes.poly.xml").write_text("\n".join(lines) + "\n")


def summary(routes):
    print(f"\n{'출발':>8s} → {'도착':<8s} " + "  ".join(f"{b:>13s}" for b in BRIDGES))
    pairs = {}
    for r in routes:
        if r["bridge"] is None:
            continue
        pairs.setdefault((r["origin"], r["destination"]), {})[r["bridge"]] = r
    for (o, d), by_b in pairs.items():
        cells = []
        for b in BRIDGES:
            r = by_b.get(b)
            cells.append(f"{'-':>13s}" if r is None else
                         f"{('*' if r['fastest'] else ' '):>2s}{r['length_m'] / 1000:4.1f}km {r['free_time_s'] / 60:4.1f}분")
        print(f"{o:>8s} → {d:<8s} " + "  ".join(cells))
    print("(* = 가장 빠른 연결로, 시간은 신호·정체 없이 제한속도로 달릴 때)")
    for which in (ISLAND, MAINLAND):
        inner = [r for r in routes if r["direction"] == f"{which} 안"]
        print(f"\n[{which} 안] {len(inner)}개")
        for r in inner:
            print(f"  {r['id']} {r['origin']:>8s} → {r['destination']:<8s} "
                  f"{r['length_m'] / 1000:4.1f}km {r['free_time_s'] / 60:4.1f}분")


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
