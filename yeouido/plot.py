"""도로망·측정 도로·랜드마크(1~7)·게이트(A~F, 샛강 다리 ㄱ~ㄷ)를 그림으로 저장 (env/yeouido/network.png).

    python -m yeouido.plot

발표 자료용이자, map_counts가 고른 도로가 맞는지 눈으로 확인하는 용도.
"""
import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from yeouido.config import LANDMARK_NO, NET, OUT, sumo_home  # noqa: E402
from yeouido.build_context import POLY  # noqa: E402
from yeouido.landmarks import RESULT as LANDMARK_FILE  # noqa: E402
from yeouido.map_counts import MAPPING  # noqa: E402

PNG = OUT / "network.png"
# 표준노드링크 도로 등급별 색, 지하차도·고가차도는 점선
COLORS = {"103": ("#e67e22", "일반국도"), "104": ("#2980b9", "특별·광역시도"),
          "107": ("#27ae60", "시군도")}
LEVEL_STYLE = {"001": ":", "002": "--"}   # 고가차도, 지하차도


def _korean_font():
    for f in ("AppleGothic", "Malgun Gothic", "NanumGothic"):
        if any(f == x.name for x in matplotlib.font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = f
            return


def _arrow(ax, edge, color, lw):
    xs, ys = zip(*edge.getShape())
    ax.plot(xs, ys, color=color, lw=lw, solid_capstyle="round", zorder=3)
    (x1, y1), (x2, y2) = edge.getShape()[-2:]
    ax.annotate("", xy=(x2, y2), xytext=(x1, y1), zorder=4,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw))


def _color(text):
    """SUMO 색 표기("r,g,b" 0~1 또는 0~255, 또는 white 같은 이름) → matplotlib 색."""
    if "," not in text:
        return text
    rgb = [float(c) for c in text.split(",")[:3]]
    return [c / 255 for c in rgb] if max(rgb) > 1 else rgb


def _draw_context(ax):
    """build_context가 만든 건물·공원·물 폴리곤을 도로 아래에 옅게 그린다."""
    if not POLY.exists():
        return
    import xml.etree.ElementTree as ET
    from matplotlib.patches import Polygon
    for p in ET.parse(POLY).getroot().iter("poly"):
        if p.get("fill") == "0" or p.get("fill") == "false":
            continue
        pts = [tuple(map(float, xy.split(","))) for xy in p.get("shape").split()]
        if len(pts) < 3:
            continue
        rgb = _color(p.get("color", "0.8,0.8,0.8"))
        ax.add_patch(Polygon(pts, closed=True, fc=rgb, ec="none",
                             zorder=-10 + float(p.get("layer", 0)) * 0.1))


def main():
    sumo_home()
    import sumolib
    _korean_font()
    plt.rcParams["axes.unicode_minus"] = False
    net = sumolib.net.readNet(str(NET))
    m = json.loads(MAPPING.read_text())

    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_facecolor("#f2efe9")
    _draw_context(ax)
    for e in net.getEdges():
        _, rank, level = ((e.getType() or "") + "..").split(".")[:3]
        xs, ys = zip(*e.getShape())
        ax.plot(xs, ys, color=COLORS.get(rank, ("#999", ""))[0], lw=0.3 + 0.45 * e.getLaneNumber(),
                ls=LEVEL_STYLE.get(level, "-"), alpha=0.8, zorder=1)
    for key, v in m["counts"].items():
        _arrow(ax, net.getEdge(v["edge"]), "#1a1a1a", 2.2)
    for g, v in m["gates"].items():
        e_in, e_out = net.getEdge(v["in"]), net.getEdge(v["out"])
        _arrow(ax, e_in, "#16a085", 3)
        _arrow(ax, e_out, "#8e44ad", 3)
        x, y = e_in.getShape()[0]
        ax.text(x, y, f" {g}", fontsize=18, weight="bold", color="#c0392b", zorder=5)
        ax.text(x, y - 120, f"  {v['name']}", fontsize=9, zorder=5)
    if LANDMARK_FILE.exists():
        for name, v in json.loads(LANDMARK_FILE.read_text()).items():
            ax.plot(v["x"], v["y"], marker="*", ms=22, color="#f1c40f", mec="#7d3c98", mew=1.5,
                    zorder=7)
            ax.annotate(f"{LANDMARK_NO.get(name, '')} {name}", (v["x"], v["y"]), xytext=(8, 8), textcoords="offset points",
                        fontsize=12, weight="bold", color="#4a235a", zorder=8,
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="#7d3c98", alpha=0.85))
        ax.plot([], [], marker="*", ms=14, color="#f1c40f", mec="#7d3c98", ls="", label="랜드마크")
    for c, label in COLORS.values():
        ax.plot([], [], color=c, lw=3, label=label)
    ax.plot([], [], color="#555", lw=2, ls="--", label="지하차도")
    ax.plot([], [], color="#555", lw=2, ls=":", label="고가차도")
    ax.plot([], [], color="#1a1a1a", lw=2.2, label="TOPIS 측정 도로")
    ax.plot([], [], color="#16a085", lw=3, label="게이트 진입(in)")
    ax.plot([], [], color="#8e44ad", lw=3, label="게이트 진출(out)")
    ax.legend(loc="upper right", fontsize=9)
    (x0, y0), (x1, y1) = net.getBBoxXY()
    ax.set_xlim(x0 - 200, x1 + 200)
    ax.set_ylim(y0 - 200, y1 + 200)
    ax.set_aspect("equal")
    ax.set_title("영등포구 북부(여의도) SUMO 도로망 (국가표준노드링크) — 랜드마크·측정 지점·게이트")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(PNG, dpi=130)
    print(f"[plot] → {PNG}")


if __name__ == "__main__":
    main()
