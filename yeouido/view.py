"""여의도 환경을 sumo-gui로 띄워 눈으로 확인한다. (학습 없이 도로·교통만)

    python -m yeouido.view                 # 출근(rush) 교통
    python -m yeouido.view --level night
    python -m yeouido.view --netedit       # 도로망을 netedit으로 열기

게이트 A~F(AV 출발/도착 후보)는 빨간 글자, TOPIS 측정 도로는 파란 점으로 표시된다.
창이 뜨면 ▶ 버튼을 눌러 재생. 상단 Delay(ms)로 재생 속도 조절.
"""
import argparse
import json
import os
import subprocess
from pathlib import Path

from yeouido.build_context import LAND, PLACES, POLY
from yeouido.config import LEVELS, NAME, NET, OUT, sumo_bin, sumo_home
from yeouido.landmarks import RESULT as LANDMARK_FILE
from yeouido.map_counts import MAPPING

MARKERS = OUT / "markers.add.xml"
GUI_SETTINGS = OUT / "gui.view.xml"


def gui_binary(name):
    """macOS에서는 직접 설치한 SUMO(SUMO_HOME)의 GUI가 더 잘 뜨므로 그쪽을 우선 쓴다."""
    home = os.environ.get("SUMO_HOME")
    if home and (Path(home) / "bin" / name).exists():
        return str(Path(home) / "bin" / name)
    return sumo_bin(name)


def write_markers():
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET))
    m = json.loads(MAPPING.read_text())
    lines = ["<additional>"]
    for g, v in m["gates"].items():
        x, y = net.getEdge(v["in"]).getShape()[0]
        lines.append(f'    <poi id="gate_{g}" type="{g} {v["name"]}" x="{x:.1f}" y="{y:.1f}" '
                     f'color="1,0,0" layer="100" width="12" height="12"/>')
    for key, v in m["counts"].items():
        shape = net.getEdge(v["edge"]).getShape()
        x, y = shape[len(shape) // 2]
        pid = key.replace("/", "_")
        lines.append(f'    <poi id="count_{pid}" type="TOPIS {key}" x="{x:.1f}" y="{y:.1f}" '
                     f'color="0,0.3,1" layer="99" width="6" height="6"/>')
    # 랜드마크: 정확한 위치에 큰 노란 핀 + 이름
    if LANDMARK_FILE.exists():
        for i, (name, lm) in enumerate(json.loads(LANDMARK_FILE.read_text()).items()):
            lines.append(f'    <poi id="landmark_{i}" type="{name}" x="{lm["x"]}" y="{lm["y"]}" '
                         f'color="1,0.8,0" layer="110" width="25" height="25"/>')
    lines.append("</additional>")
    MARKERS.write_text("\n".join(lines) + "\n")


def write_gui_settings():
    """배경을 땅색으로, 핀(POI) 이름을 크게 보이게 하는 sumo-gui 설정."""
    GUI_SETTINGS.write_text(
        '<viewsettings>\n'
        f'    <background backgroundColor="{LAND}"/>\n'
        '    <pois poiType_show="1" poiType_size="60" poiType_color="255,0,0"/>\n'
        '</viewsettings>\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--level", choices=list(LEVELS), default="rush")
    ap.add_argument("--netedit", action="store_true", help="netedit으로 도로망만 열기")
    args = ap.parse_args(argv)
    if args.netedit:
        subprocess.run([gui_binary("netedit"), str(NET)])
        return
    write_markers()
    write_gui_settings()
    subprocess.run([gui_binary("sumo-gui"), "-c", str(OUT / f"{NAME}_{args.level}.sumocfg"),
                    "--additional-files", f"{POLY},{MARKERS}", "--step-length", "0.5",
                    "--gui-settings-file", str(GUI_SETTINGS)])


if __name__ == "__main__":
    main()
