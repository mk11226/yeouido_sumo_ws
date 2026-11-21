"""핀포인트 두 곳을 골라 sumo-gui에서 그 경로를 주행한다. 주행 경로의 도로만 바탕색이 바뀐다.

    python -m yeouido_cross.drive_route 1 B                    # 랜드마크 1(국회의사당) → 게이트 B(마포대교)
    python -m yeouido_cross.drive_route 5 3 --level rush       # 출근 시간대 교통
    python -m yeouido_cross.drive_route 영등포역 "게이트 A"     # 이름으로도 지정
    python -m yeouido_cross.drive_route 1 B --overview         # ego를 따라가지 않고 도로망 전체를 보여 줌
    python -m yeouido_cross.drive_route --list                 # 고를 수 있는 핀포인트 목록

핀포인트 13곳 (sumo-gui에 항상 표시)
  랜드마크 1~7  1 국회의사당 · 2 IFC 서울 · 3 63빌딩 · 4 여의도공원 / 5 영등포역 · 6 당산역 · 7 노량진수산시장
  게이트 A~F    A 서강대교 · B 마포대교 · C 원효대교 / D 양화대교 · E 오목교 · F 거리공원

경로는 yeouido_cross/output/routes.json (쌍마다 최단거리 하나, python -m yeouido_cross.build_routes) 을 그대로 쓴다.
주행은 SUMO 운전자 모델(SumoExpert)이 한다.

경로 표시: 경로가 지나는 차로(교차로 안 연결 차로 포함)를 sumo-gui에서 "선택"하고, 차로 색칠 방식을
"by selection"으로 둔다. 선택된 차로는 아스팔트 색만 바뀌고 차선·중앙선은 그 위에 그대로 그려진다.
나머지 도로는 평소 색(검정 아스팔트)이다.

경로가 주행 중에 바뀌면 (경로 재탐색 --reroute, 목적지 변경) 처음 파란 경로는 그대로 두고, 바뀐 경로를
주황색 선으로 겹쳐 그린다 (SUMO 도형 polygon, 차로 가운데에 차로보다 좁게 → 겹치는 구간은 파랑 위에 주황 줄).
다시 바뀌면 주황색만 새 경로로 다시 그린다. RouteWatch 가 매 스텝 ego 경로를 보고 바뀌었을 때만 그린다.
"""
import argparse
import json
import os
import time
import xml.sax.saxutils as su

from env.road_config import ROAD, EGO, TRAFFIC
from env.mdp_config import SIMULATION, OBSERVATION, ACTION
from env import road_builder
from env.yeouido_env import DATA, make_env, place_name, traci
from train import REWARD
from yeouido.build_context import LAND
from yeouido.config import LANDMARK_NO

HERE = os.path.dirname(os.path.abspath(__file__))     # yeouido_cross/
BASE = os.path.dirname(HERE)                          # 저장소 루트
ROUTES = os.path.join(HERE, "output", "routes.json")
GUI_DIR = os.path.join(HERE, "output")
PINS = os.path.join(GUI_DIR, "pins.add.xml")
VIEW = os.path.join(GUI_DIR, "route.view.xml")

ROUTE_COLOR = "70,120,185"      # 경로 차로 바탕색 (검정 아스팔트 대신). 차선이 잘 보이도록 어둡지 않은 중간 톤
ROAD_COLOR = "black"            # 나머지 도로 (sumo-gui 기본 아스팔트 색)
REROUTE_COLOR = (255, 140, 0, 235)   # 주행 중 바뀐 경로 (주황)
REROUTE_WIDTH = 1.4             # 주황 선 폭 (m). 차로(약 3.2 m)보다 좁아 겹치는 구간에서도 양옆 파랑이 보인다
REROUTE_LAYER = 5.0             # 도로 위, 핀(110~)·차량 아래
LANDMARK_FILL = "#f2b632"       # 노란 동그라미
GATE_FILL = "#c9403a"           # 빨간 둥근 네모
PIN_DIR = os.path.join(GUI_DIR, "pins")
# 핀 크기는 지도에 고정 (확대·축소하면 지도와 같이 커지고 작아진다).
# 핀포인트 지도 이미지와 같은 비율: 그 이미지는 약 2.9 m/픽셀이고 동그라미 지름 52 픽셀 ≈ 150 m.
ICON = 173.0                    # 아이콘 그림 한 변 (m). 동그라미는 그림의 88% → 지름 약 150 m
PIN_H = ICON                    # 이름표 그림 높이 (m)
PIN_W = ICON * 8                # 이름표 그림 폭 (m). 가운데가 지점, 이름은 한쪽 절반에
MAX_SCREEN = 0.025              # 확대했을 때 핀(아이콘 한 변)이 화면 폭의 이 비율보다 커지지 않게 줄인다.
                                # 전체 지도에서는 지도 고정 크기 그대로, 가까이 보면 작아져 도로를 가리지 않는다.
LABEL_LEFT = {"여의도공원"}       # 이웃 핀(2 IFC 서울)과 겹치지 않게 이름을 왼쪽에 다는 곳
SCHEME = "route_view"           # sumo-gui 화면 설정 이름


def pins():
    """핀포인트 13곳 {이름: (표기, x, y, 종류)}. 샛강 다리 게이트 ㄱ~ㄷ은 넣지 않는다."""
    import sumolib
    net = sumolib.net.readNet(os.path.join(DATA, "yeouido.net.xml"))
    out = {}
    for name, lm in json.load(open(os.path.join(DATA, "landmarks.json"))).items():
        out[name] = (LANDMARK_NO[name], lm["x"], lm["y"], "landmark")
    for g, v in json.load(open(os.path.join(DATA, "count_edges.json")))["gates"].items():
        if g.isascii():
            x, y = net.getEdge(v["in"]).getShape()[0]
            out[f"게이트 {g}"] = (g, x, y, v["name"])
    return out


def _pin_png(path, tag, label, kind, left=False):
    """핀 그림 두 장: 아이콘(노란 동그라미 / 빨간 둥근 네모, 정사각형)과 이름표.
    이름표 그림은 가운데가 지점이 되도록 반대쪽 절반을 비워 둔다. (sumo-gui 글꼴은 한글이 안 나와서 그림으로 넣는다)"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyBboxPatch
    import matplotlib.patheffects as pe
    from matplotlib import font_manager
    installed = set(font_manager.get_font_names())
    plt.rcParams["font.family"] = next(
        (name for name in ("Nanum Gothic", "NanumGothic", "AppleGothic", "Noto Sans CJK JP")
         if name in installed), "sans-serif")
    # 아이콘
    fig = plt.figure(figsize=(2, 2), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    if kind == "landmark":
        ax.add_patch(Circle((0.5, 0.5), 0.44, fc=LANDMARK_FILL, ec="#1d1d1f", lw=3))
        tc = "#1d1d1f"
    else:
        ax.add_patch(FancyBboxPatch((0.12, 0.12), 0.76, 0.76, boxstyle="round,pad=0,rounding_size=0.2",
                                    fc=GATE_FILL, ec="white", lw=3))
        tc = "white"
    ax.text(0.5, 0.5, tag, fontsize=66, color=tc, ha="center", va="center",
            path_effects=[pe.Stroke(linewidth=1.5, foreground=tc), pe.Normal()])
    fig.savefig(path + "_icon.png", transparent=True)
    plt.close(fig)
    # 이름표
    ratio = PIN_W / PIN_H
    fig = plt.figure(figsize=(ratio * 2, 2), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, ratio); ax.set_ylim(0, 1); ax.axis("off")
    cx = ratio / 2
    # 지도 이미지와 같게: 랜드마크 이름은 굵은 검정, 게이트 이름은 작은 회색. 흰 테두리로 도로 위에서도 읽히게
    big = kind == "landmark"
    color = "#1d1d1f" if big else "#555555"
    ax.text(cx - 0.64 if left else cx + 0.64, 0.5, label, fontsize=66 if big else 50, color=color,
            ha="right" if left else "left", va="center",
            path_effects=[pe.Stroke(linewidth=10, foreground="white")]
                         + ([pe.Stroke(linewidth=2, foreground=color)] if big else []) + [pe.Normal()])
    fig.savefig(path + "_label.png", transparent=True)
    plt.close(fig)


def write_gui_files():
    """핀 13곳(그림 POI)과 경로 색칠용 sumo-gui 설정을 쓴다. 아이콘은 이름표보다 위 층이라 늘 보인다."""
    os.makedirs(PIN_DIR, exist_ok=True)
    lines = ["<additional>"]
    for i, (name, (tag, x, y, kind)) in enumerate(pins().items()):
        base = os.path.join(PIN_DIR, f"pin_{i}")
        _pin_png(base, tag, name if kind == "landmark" else kind, kind, left=name in LABEL_LEFT)
        lines.append(f'    <poi id="pin_{i}" type={su.quoteattr(f"{tag} {name}")} x="{x:.1f}" y="{y:.1f}" '
                     f'layer="{140 + i}" imgFile="{base}_icon.png" width="{ICON}" height="{ICON}"/>')
        lines.append(f'    <poi id="pin_{i}_label" type={su.quoteattr(name)} x="{x:.1f}" y="{y:.1f}" '
                     f'layer="{(110 if name in LABEL_LEFT else 120) + i}" imgFile="{base}_label.png" '
                     f'width="{PIN_W}" height="{PIN_H}"/>')
    lines.append("</additional>")
    with open(PINS, "w") as f:
        f.write("\n".join(lines) + "\n")

    with open(VIEW, "w") as f:
        f.write(
            '<viewsettings>\n'
            f'    <scheme name="{SCHEME}">\n'
            f'        <background backgroundColor="{LAND}"/>\n'
            '        <vehicles vehicleQuality="2" vehicle_minSize="1.00" vehicle_exaggeration="1.00" vehicle_constantSize="0"/>\n'
            '        <edges laneEdgeMode="1">\n'
            '            <colorScheme name="by selection">\n'
            f'                <entry color="{ROAD_COLOR}" name="unselected"/>\n'
            f'                <entry color="{ROUTE_COLOR}" name="selected"/>\n'
            '            </colorScheme>\n'
            '        </edges>\n'
            '        <pois poi_exaggeration="1" poi_constantSize="0" poiType_show="0" poiName_show="0"/>\n'
            '    </scheme>\n'
            '</viewsettings>\n')


def route_points(r):
    """routes.json 경로의 실제 출발·도착 지점 → env.reset 옵션.
    랜드마크는 건물 쪽 도로 대신 반대 방향 도로에서 출발·도착하는 경로도 있다 (landmark_sides.py)."""
    return {"origin_at": [r["edges"][0], r["origin_pos"]],
            "destination_at": [r["edges"][-1], r["dest_pos"]]}


def route_lanes(edges):
    """경로를 따라 실제로 갈 수 있는 차로만.

    도로(edge)마다 다음 도로로 이어지는 연결이 있는 차로만 고른다. 예를 들어 다음 교차로에서 좌회전해야 하고
    좌회전이 1·2차로에서만 되면 그 도로는 1·2차로만 칠한다. 교차로 안에서는 그 차로들에서 나가는
    연결 차로(internal lane)만 칠한다. 마지막 도로는 모든 차로.
    """
    lanes = []
    for i, e in enumerate(edges):
        all_lanes = [f"{e}_{k}" for k in range(traci.edge.getLaneNumber(e))]
        nxt = edges[i + 1] if i + 1 < len(edges) else None
        if nxt is None:
            lanes += all_lanes
            continue
        picked = []
        for lane in all_lanes:
            for link in traci.lane.getLinks(lane, extended=True):
                to_lane, via = link[0], link[4]
                if to_lane.rsplit("_", 1)[0] != nxt:
                    continue
                picked.append(lane)
                while via:                        # 큰 교차로는 연결 차로가 여러 개 이어진다
                    lanes.append(via)
                    nexts = traci.lane.getLinks(via, extended=True)
                    via = nexts[0][4] if nexts and nexts[0][4] else ""
        lanes += picked or all_lanes              # 연결 정보가 없으면 (없어야 정상) 모든 차로
    return list(dict.fromkeys(lanes))


_pin_scale = None


def fit_pins():
    """핀 크기 = min(지도 고정 크기, 화면 폭 × MAX_SCREEN). 화면이 바뀌었을 때만 크기를 고친다."""
    global _pin_scale
    try:
        view = traci.gui.getIDList()[0]
        (x0, _), (x1, _) = traci.gui.getBoundary(view)
    except (traci.TraCIException, IndexError):
        return
    scale = round(min(1.0, (x1 - x0) * MAX_SCREEN / ICON), 3)
    if scale == _pin_scale:
        return
    _pin_scale = scale
    for pid in traci.poi.getIDList():
        if not pid.startswith("pin_"):
            continue
        w, h = (PIN_W, PIN_H) if pid.endswith("_label") else (ICON, ICON)
        traci.poi.setWidth(pid, w * scale)
        traci.poi.setHeight(pid, h * scale)


def highlight(lanes):
    try:
        traci.gui.setSchema(traci.gui.getIDList()[0], SCHEME)
    except (traci.TraCIException, IndexError):
        pass
    for lane in lanes:
        try:
            if not traci.gui.isSelected(lane, "lane"):
                traci.gui.toggleSelection(lane, "lane")
        except traci.TraCIException:
            pass


_overlay_ids = []


def clear_overlay():
    """주황 경로 선을 지운다."""
    for pid in _overlay_ids:
        try:
            traci.polygon.remove(pid)
        except traci.TraCIException:
            pass
    _overlay_ids.clear()


def overlay(lanes):
    """lanes(차로 id 목록)를 주황 선으로 겹쳐 그린다. 이전 주황 선은 지운다 (파란 경로는 그대로)."""
    clear_overlay()
    for k, lane in enumerate(lanes):
        try:
            shape = traci.lane.getShape(lane)
            if len(shape) < 2:
                continue
            pid = f"reroute_{k}"
            traci.polygon.add(pid, shape, REROUTE_COLOR, fill=False, polygonType="reroute",
                              layer=REROUTE_LAYER, lineWidth=REROUTE_WIDTH)
            _overlay_ids.append(pid)
        except traci.TraCIException:
            pass


class RouteWatch:
    """처음 경로(파랑)를 기억해 두고, ego 경로가 바뀌면 남은 경로를 주황으로 겹쳐 그린다."""

    def __init__(self, veh):
        self.veh = veh
        self.route = tuple(traci.vehicle.getRoute(veh))
        self.changes = 0
        clear_overlay()

    def update(self):
        """매 스텝 부른다. 경로가 바뀌어 새로 그렸으면 True."""
        try:
            route = tuple(traci.vehicle.getRoute(self.veh))
            idx = traci.vehicle.getRouteIndex(self.veh)
        except traci.TraCIException:
            return False                      # ego 가 없음 (도착·충돌로 제거됨)
        if route == self.route:
            return False
        self.route = route
        self.changes += 1
        overlay(route_lanes(list(route[max(idx, 0):])))
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("origin", nargs="?", help="출발지 (1~7, A~F, 또는 이름)")
    ap.add_argument("destination", nargs="?", help="도착지")
    ap.add_argument("--level", choices=["night", "day", "rush"], default="day", help="교통 시간대 (기본 day)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--delay", type=float, default=50.0, help="sumo-gui 재생 지연 ms (작을수록 빠름)")
    ap.add_argument("--overview", action="store_true", help="ego를 따라가지 않고 전체 도로망 보기")
    ap.add_argument("--auto-close", action="store_true",
                    help="주행이 끝나면 바로 창을 닫음 (기본: 사용자가 창을 닫을 때까지 끝 화면 유지)")
    ap.add_argument("--list", action="store_true", help="핀포인트 목록 출력")
    ap.add_argument("--screenshot", metavar="PNG", help="출발 직후 sumo-gui 화면을 이 파일로 저장")
    ap.add_argument("--reroute", action="store_true",
                    help="경로 재탐색 켜기 (막히면 더 빠른 길로, 바뀐 경로는 주황색으로 겹쳐 표시)")
    args = ap.parse_args()

    routes = {(r["origin"], r["destination"]): r for r in json.load(open(ROUTES))}
    names = list(dict.fromkeys(o for o, _ in routes))
    if args.list or not (args.origin and args.destination):
        no = {**{n: t for n, t in LANDMARK_NO.items()}, **{n: n.split()[-1] for n in names if n.startswith("게이트")}}
        print("핀포인트:", "  ".join(f"{no[n]}={n}" for n in sorted(names, key=lambda n: (n.startswith("게이트"), no[n]))))
        if not args.list:
            ap.error("출발지와 도착지를 주세요. 예: python -m yeouido_cross.drive_route 1 B")
        return

    places = {n: None for n in names}             # 13곳만 허용 (ㄱ~ㄷ 제외)
    origin, destination = place_name(places, args.origin), place_name(places, args.destination)
    if origin == destination:
        ap.error("출발지와 도착지가 같습니다.")
    r = routes[(origin, destination)]

    write_gui_files()
    road = {**ROAD, "gui_files": ["yeouido.poly.xml", PINS], "gui_settings": VIEW}
    if args.reroute:
        road["reroute"] = {**ROAD.get("reroute", {}), "enabled": True}
    cfg = road_builder.build(road, EGO, TRAFFIC, os.path.join(BASE, "env", "sumo"))
    env = make_env(cfg_path=cfg, road=road, ego=EGO, mdp_sim=SIMULATION, mdp_obs=OBSERVATION,
                   action=ACTION, reward=REWARD, traffic=TRAFFIC, gui=True, gui_autostart=True)
    env.gui_delay, env.gui_track_ego = args.delay, not args.overview
    from algorithms.sumo_expert import SumoExpert
    expert = SumoExpert(env)
    try:
        obs, _ = env.reset(seed=args.seed, options={"level": args.level, "origin": origin,
                                                   "destination": destination, **route_points(r)})
        # routes.json 의 최단거리 경로로 바꾼다 (env는 기본으로 최단시간 경로를 탄다)
        here = traci.vehicle.getRoadID(env.ego_id)
        edges = r["edges"][r["edges"].index(here):] if here in r["edges"] else None
        if edges:
            traci.vehicle.setRoute(env.ego_id, edges)
            env.route_length = r["length_m"]
            env._remaining = env._prev_remaining = env.remaining_distance()
        else:
            print(f"[drive] 출발 도로({here})가 경로 파일과 달라 SUMO 기본 경로로 주행합니다.")
        highlight(route_lanes(list(traci.vehicle.getRoute(env.ego_id))))
        watch = RouteWatch(env.ego_id)
        fit_pins()
        via = f" via {r['bridge']}" if r["bridge"] else ""
        print(f"[drive] {args.level}: {origin} → {destination}{via}  "
              f"{r['length_m'] / 1000:.1f} km (경로 {r['id']}), 주변 일반 차량 {traci.vehicle.getIDCount() - 1}대",
              flush=True)

        done, t, info, last = False, 0, {}, None
        while not done:
            obs, _, term, trunc, info = env.step(expert.predict(obs))
            done, t = term or trunc, t + 1
            if not done and watch.update():
                print(f"[drive] {t} 스텝: 경로 바뀜 ({watch.changes}번째) → 주황색으로 표시, "
                      f"남은 거리 {env._remaining or 0:.0f} m", flush=True)
            if env.ego_id in traci.vehicle.getIDList():
                last = (traci.vehicle.getRoadID(env.ego_id), traci.vehicle.getPosition(env.ego_id), env._remaining)
            crashes = [c for c in traci.simulation.getCollisions() if env.ego_id in (c.collider, c.victim)]
            if t % 4 == 0:
                fit_pins()                        # 마우스로 확대·축소해도 2초(시뮬레이션) 안에 맞춰진다
            if t % 100 == 0:
                left = env._remaining
                print(f"[drive] {t} 스텝, 남은 거리 {left:.0f} m" if left is not None else f"[drive] {t} 스텝",
                      flush=True)
            if args.screenshot and t == 20:
                traci.gui.screenshot(traci.gui.getIDList()[0], os.path.abspath(args.screenshot))
        result = "도착" if info.get("arrived") else "충돌" if info.get("collided") else "시간 초과/중단"
        where = ""
        if last:
            road, (x, y), left = last
            name = traci.edge.getStreetName(road) if not road.startswith(":") else "교차로 안"
            where = f", 마지막 위치 {name} ({road}, x={x:.0f} y={y:.0f})" + \
                    (f", 남은 거리 {left:.0f} m" if left is not None and result != "도착" else "")
        print(f"[drive] {result} — {t} 스텝{where}", flush=True)
        for c in crashes:
            other = c.victim if c.collider == env.ego_id else c.collider
            print(f"[drive]   충돌 상대 {other} ({c.type}), 차로 {c.lane}, 위치 {c.pos:.0f} m", flush=True)
        if not args.auto_close:
            # 끝 화면 유지: 시뮬레이션은 멈춘 채 창은 그대로 (확대·이동 가능). 창을 닫으면 끝난다.
            print("[drive] 끝 화면을 유지합니다. sumo-gui 창을 닫으면 종료됩니다.", flush=True)
            try:
                while traci.gui.getIDList():          # 창을 닫으면 view가 없어진다
                    fit_pins()                    # 끝 화면에서 확대·축소해도 핀 크기를 맞춘다
                    time.sleep(0.3)
            except (traci.TraCIException, traci.FatalTraCIError, OSError):
                env._started = False          # 창이 닫혀 연결이 끊김
    finally:
        env.close()


if __name__ == "__main__":
    main()
