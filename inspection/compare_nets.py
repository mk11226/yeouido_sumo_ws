"""도로망 수정 전후 비교 보고서 — yeouido/corrections.json 이 도로망을 망가뜨리지 않았는지 확인.

    python -m inspection.compare_nets                 # 구조 비교만 (수 초)
    python -m inspection.compare_nets --sim rush      # + 시뮬레이션 비교 (시간대마다 수 분)
    python -m inspection.compare_nets --sim night day rush

같은 PC·같은 SUMO 로 두 도로망을 새로 만들어 비교한다 (env/yeouido 파일은 건드리지 않음).
  수정 전 = python -m yeouido.build_net --no-corrections
  수정 후 = python -m yeouido.build_net              (corrections.json 반영)
결과: inspection/out/compare/ (도로망 두 개, 보고서 report.md / report.json)

보는 것
  1. 바뀐 범위: 도로·연결(이동 가능 방향)·신호·교차로 모양. 수정한 도로의 앞뒤 교차로 밖에서
     바뀐 것이 있으면 ⚠ (netconvert 가 교차로를 다시 계산하며 옆 교차로까지 바꾸는 경우)
  2. netconvert 경고: 수정 후 새로 생긴 경고
  3. 연결성: 랜드마크 두 곳 사이 경로가 모두 있는지, 길이가 바뀌었는지
  4. (--sim) 같은 교통 수요로 두 도로망을 돌려 측정 지점 GEH·teleport·충돌(교차로 안 포함)·
     평균 속도·대기, 수정한 진입로의 대기·속도를 비교. 수요 경로는 잘린 도로(<ID> → <ID>,
     <ID>_pocket)에 맞게 고쳐 같은 차량이 그대로 다니게 한다.
"""
import argparse
import itertools
import json
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import sumolib

from yeouido import corrections
from yeouido.config import DEMAND_SCALE, END, LEVELS, MEASURE, OUT as YEOUIDO_OUT, WARMUP, sumo_bin
from yeouido.landmarks import RESULT as LANDMARKS
from yeouido.map_counts import MAPPING
from yeouido.prepare_counts import COUNTS
from yeouido.validate import STEP_LENGTH, geh

HERE = Path(__file__).resolve().parent
DIR = HERE / "out" / "compare"
BASE, NEW = DIR / "base.net.xml", DIR / "new.net.xml"
SUFFIX = corrections.POCKET_SUFFIX
root_id = lambda eid: eid[:-len(SUFFIX)] if eid.endswith(SUFFIX) else eid   # noqa: E731
# 새 도로를 붙이려고 자른 도로의 앞 조각 (<도로 ID>_a) → 원래 도로로 본다
cut_root = lambda eid: eid[:-2] if eid.endswith("_a") else eid   # noqa: E731


def build():
    for out, extra in ((BASE, ["--no-corrections"]), (NEW, [])):
        r = subprocess.run([sys.executable, "-m", "yeouido.build_net", "--out", str(out), *extra],
                           capture_output=True, text=True)
        if r.returncode:
            raise RuntimeError(r.stdout + r.stderr)
        for line in r.stdout.splitlines():
            if "수정 목록" in line or "건너뜀" in line:
                print(line)


# ── 1. 구조 비교 ────────────────────────────────────────────
def movements(net):
    """교차로마다 이동 가능 방향 {(들어오는 도로, 나가는 도로, 방향)} (포켓은 원래 도로 ID로)."""
    out = defaultdict(set)
    for e in net.getEdges():
        for to, conns in e.getOutgoing().items():
            for c in conns:
                out[e.getToNode().getID()].add((root_id(e.getID()), root_id(to.getID()), c.getDirection()))
    return out


def lane_links(net):
    out = defaultdict(set)
    for e in net.getEdges():
        for to, conns in e.getOutgoing().items():
            for c in conns:
                out[e.getToNode().getID()].add((e.getID(), c.getFromLane().getIndex(), to.getID(),
                                                c.getToLane().getIndex(), c.getDirection()))
    return out


def tls_programs(path):
    return {tl.get("id"): [(p.get("state"), p.get("duration")) for p in tl.findall("phase")]
            for tl in ET.parse(path).getroot().iter("tlLogic")}


def node_shift(a, b):
    """교차로 모양이 얼마나 바뀌었나: 꼭짓점 사이 최대 거리 (점 개수가 다르면 중심 이동)."""
    sa, sb = a.getShape(), b.getShape()
    if len(sa) == len(sb) and sa:
        return max(math.dist(p, q) for p, q in zip(sa, sb))
    ca = [sum(v) / len(sa) for v in zip(*sa)] if sa else a.getCoord()[:2]
    cb = [sum(v) / len(sb) for v in zip(*sb)] if sb else b.getCoord()[:2]
    return math.dist(ca, cb) + 1000   # 점 개수가 바뀌면 크게 표시


def structure(base, new):
    eb = {e.getID(): e for e in base.getEdges()}
    en = {e.getID(): e for e in new.getEdges()}
    split_edges = {root_id(e) for e in en if e.endswith(SUFFIX)}
    # 차로 수·차로 방향을 고친 도로 (수정 목록 기준)
    data = corrections.load()
    edited = split_edges | {c["edge"] for k in ("lane_counts", "lane_moves") for c in data[k] if c["edge"] in eb}
    # 새 도로 (add_links) 와 그 때문에 자른 도로
    new_roads = {c["id"] for c in data["add_links"]} & set(en)
    moved = {c["edge"] for c in data["lane_moves"]}
    edited |= {cut_root(e) for e in set(en) - set(eb) if e.endswith("_a") and cut_root(e) in eb}
    edited |= {c["from"] for c in data["add_links"] if c.get("replace") and c.get("from") in eb and c.get("from") in en}
    # 수정한 도로의 앞뒤 교차로 + 잘린 곳에 새로 생긴 점 = 바뀌어도 되는 곳
    expected = set()
    for eid in edited:
        expected |= {eb[eid].getFromNode().getID(), eb[eid].getToNode().getID(),
                     en[eid].getToNode().getID()}
    # 삭제한 도로 (끊겨서 같이 빠진 조각 포함) 의 양 끝 교차로도 바뀌어도 된다
    gone = set(eb) - set(en)
    for eid in gone:
        expected |= {eb[eid].getFromNode().getID(), eb[eid].getToNode().getID()}
    for eid in new_roads:
        expected |= {en[eid].getFromNode().getID(), en[eid].getToNode().getID()}
    # 지운 회전 제한: 그 회전이 새로 생기는 것은 정상
    freed = {(corrections.link_id(c["from"]), corrections.link_id(c["to"])) for c in data["remove_turns"]}
    for f, _ in freed:
        expected |= {e.getToNode().getID() for i, e in eb.items() if corrections.link_id(i) == f}

    rep = dict(split_edges=sorted(split_edges), edited_edges=sorted(edited), added_edges=sorted(set(en) - set(eb)),
               removed_edges=sorted(set(eb) - set(en)), changed_edges=[], movements=[], lane_links=[],
               tls=[], node_shapes=[], unexpected=[])
    for eid in sorted(set(eb) & set(en)):
        a, b = eb[eid], en[eid]
        diff = {}
        if a.getLaneNumber() != b.getLaneNumber():
            diff["lanes"] = [a.getLaneNumber(), b.getLaneNumber()]
        if abs(a.getLength() - b.getLength()) > 0.5:
            diff["length"] = [round(a.getLength(), 1), round(b.getLength(), 1)]
        if abs(a.getSpeed() - b.getSpeed()) > 0.01:
            diff["speed_kmh"] = [round(a.getSpeed() * 3.6), round(b.getSpeed() * 3.6)]
        if diff:
            # 수정한 교차로에 붙은 도로는 교차로 모양이 다시 계산되며 길이가 조금 바뀔 수 있다
            near = b.getFromNode().getID() in expected or b.getToNode().getID() in expected
            rep["changed_edges"].append(dict(edge=eid, name=b.getName(), split=eid in edited,
                                             expected=eid in edited or (near and set(diff) == {"length"}),
                                             **diff))
            if not rep["changed_edges"][-1]["expected"]:
                rep["unexpected"].append(f"도로 {eid} ({b.getName()}) 속성 바뀜: {diff}")

    mb, mn = movements(base), movements(new)
    lb, ln = lane_links(base), lane_links(new)
    base_nodes = {n.getID() for n in base.getNodes()}
    for nid in sorted(set(mb) | set(mn)):
        lost, gained = mb[nid] - mn[nid], mn[nid] - mb[nid]
        lost = {m for m in lost if m[0] not in gone and m[1] not in gone}   # 삭제한 도로로 가던 방향은 없어져야 맞다
        # 새 도로로 가는 방향은 생겨야 맞고, 자른 도로 앞 조각(_a)은 원래 도로와 같은 방향이다
        norm = {(cut_root(a), cut_root(b), d) for a, b, d in mn[nid]}
        lost = {m for m in lost if m not in norm}
        lost = {m for m in lost if m[0] not in moved}   # 차로 방향을 고쳐 일부러 없앤 회전
        gained = {m for m in gained if m[0] not in new_roads and m[1] not in new_roads
                  and (corrections.link_id(m[0]), corrections.link_id(m[1])) not in freed
                  and (cut_root(m[0]), cut_root(m[1]), m[2]) not in mb[nid]}
        if nid not in base_nodes:   # 도로를 자른 곳에 새로 생긴 점 (원래 도로 → 포켓)
            continue
        if lost or gained:
            rep["movements"].append(dict(node=nid, lost=sorted(lost), gained=sorted(gained)))
            # 이동 가능 방향은 수정한 교차로에서도 바뀌면 안 된다 (차로만 늘린 것이므로)
            rep["unexpected"].append(f"교차로 {nid}: 갈 수 있는 방향 바뀜 (잃음 {sorted(lost)}, 생김 {sorted(gained)})")
        if lb[nid] != ln[nid]:
            rep["lane_links"].append(dict(node=nid, base=len(lb[nid]), new=len(ln[nid]),
                                          expected=nid in expected))
            if nid not in expected:
                rep["unexpected"].append(f"교차로 {nid}: 차로 연결 바뀜 ({len(lb[nid])} → {len(ln[nid])}개)")

    tb, tn = tls_programs(BASE), tls_programs(NEW)
    tls_nodes = defaultdict(set)
    for n in new.getNodes():
        for c in n.getConnections():
            if c.getTLSID():
                tls_nodes[c.getTLSID()].add(n.getID())
    for tid in sorted(set(tb) | set(tn)):
        if tb.get(tid) != tn.get(tid):
            nb, nn = len(tb.get(tid, [])), len(tn.get(tid, []))
            protected = lambda prog: sum(1 for s, _ in prog if "G" in s and "g" not in s and set(s) <= {"G", "r"})  # noqa: E731
            ok = bool(tls_nodes[tid] & expected)
            rep["tls"].append(dict(tls=tid, phases=[nb, nn], expected=ok,
                                   protected_phases=[protected(tb.get(tid, [])), protected(tn.get(tid, []))]))
            if not ok:
                rep["unexpected"].append(f"신호 {tid}: 단계 바뀜 ({nb} → {nn}개)")

    nb_, nn_ = {n.getID(): n for n in base.getNodes()}, {n.getID(): n for n in new.getNodes()}
    for nid in sorted(set(nb_) & set(nn_)):
        d = node_shift(nb_[nid], nn_[nid])
        if d > 0.5:
            rep["node_shapes"].append(dict(node=nid, shift_m=round(d, 2), expected=nid in expected))
            if nid not in expected:
                rep["unexpected"].append(f"교차로 {nid}: 모양이 {d:.1f} m 바뀜")
    rep["expected_nodes"] = sorted(expected)
    rep["new_roads"] = sorted(new_roads)
    return rep


# ── 2. netconvert 경고 ───────────────────────────────────────
def warnings(path):
    log = path.with_name(f"{path.stem}.netconvert.log")
    return {l for l in log.read_text().splitlines() if l.startswith("Warning:") and "total messages" not in l}


# ── 3. 연결성 ───────────────────────────────────────────────
def connectivity(base, new):
    if not LANDMARKS.exists():
        return dict(pairs=0, unreachable=[], changed=[])
    marks = json.loads(LANDMARKS.read_text())
    out = dict(pairs=0, unreachable=[], changed=[])
    for a, b in itertools.permutations(marks, 2):
        out["pairs"] += 1
        lengths = []
        for net in (base, new):
            route, cost = net.getShortestPath(net.getEdge(marks[a]["edge"]), net.getEdge(marks[b]["edge"]),
                                              vClass="passenger")
            lengths.append(sum(e.getLength() for e in route) if route else None)
        if lengths[1] is None and lengths[0] is not None:
            out["unreachable"].append(f"{a} → {b}")
        elif lengths[0] and lengths[1] and abs(lengths[0] - lengths[1]) > 1:
            out["changed"].append(dict(pair=f"{a} → {b}", base=round(lengths[0]), new=round(lengths[1])))
    return out


# ── 4. 시뮬레이션 ───────────────────────────────────────────
def patch_routes(src, dst, split_edges):
    """경로의 <ID> 다음에 <ID>_pocket 을 끼워 넣는다 (경로가 그 도로에서 끝나면 그대로)."""
    def fix(m):
        edges = m.group(2).split()
        out = []
        for i, e in enumerate(edges):
            out.append(e)
            if e in split_edges and i < len(edges) - 1:
                out.append(e + SUFFIX)
        return f'{m.group(1)}"{" ".join(out)}"'
    text = src.read_text()
    dst.write_text(re.sub(r'(\bedges=)"([^"]*)"', fix, text))


def simulate(label, net_path, level, split_edges, focus):
    run = DIR / f"sim_{level}_{label}"
    run.mkdir(parents=True, exist_ok=True)
    routes = run / f"{level}.rou.xml"
    if label == "new":
        patch_routes(YEOUIDO_OUT / f"{level}.rou.xml", routes, split_edges)
    else:
        routes.write_text((YEOUIDO_OUT / f"{level}.rou.xml").read_text())
    (run / "edges.add.xml").write_text(
        f'<additional>\n    <edgeData id="m" file="edgedata.xml" begin="{WARMUP}" end="{END}" excludeEmpty="true"/>\n'
        f'</additional>\n')
    cmd = [sumo_bin("sumo"), "-n", str(net_path),
           "-r", f"{YEOUIDO_OUT / 'yeouido.vtypes.xml'},{routes}",
           "--additional-files", str(run / "edges.add.xml"),
           "--begin", "0", "--end", str(END), "--time-to-teleport", "300", "--ignore-route-errors", "true",
           "--seed", "42", "--step-length", str(STEP_LENGTH),
           "--statistic-output", str(run / "stats.xml"),
           "--collision.action", "warn", "--collision.check-junctions", "true",
           "--no-step-log", "--no-warnings", "--duration-log.statistics", "--error-log", str(run / "errors.log")]
    subprocess.run(cmd, check=True, cwd=run, stdout=subprocess.DEVNULL)

    st = ET.parse(run / "stats.xml").getroot()
    veh, tele, trips, safety = st.find("vehicles"), st.find("teleports"), st.find("vehicleTripStatistics"), st.find("safety")
    edges = {e.get("id"): e for e in ET.parse(run / "edgedata.xml").getroot().iter("edge")}
    counts, mapping = json.loads(COUNTS.read_text()), json.loads(MAPPING.read_text())["counts"]
    hours = LEVELS[level]["hours"]
    gehs = []
    for key, c in counts.items():
        target = sum(c["hourly"][h] for h in hours) / len(hours) * DEMAND_SCALE
        e = edges.get(mapping[key]["edge"])
        got = (float(e.get("entered", 0)) + float(e.get("departed", 0))) * 3600.0 / MEASURE if e is not None else 0.0
        gehs.append(geh(got, target))
    focus_stats = {}
    for eid in focus:
        e = edges.get(eid)
        if e is not None:
            focus_stats[eid] = dict(entered=int(float(e.get("entered", 0))),
                                    speed_kmh=round(float(e.get("speed", 0)) * 3.6, 1),
                                    waiting_s=round(float(e.get("waitingTime", 0)), 1),
                                    time_loss_s=round(float(e.get("timeLoss", 0)), 1))
    return dict(loaded=int(veh.get("loaded")), inserted=int(veh.get("inserted")),
                route_errors=sum(1 for l in (run / "errors.log").read_text().splitlines() if "Error" in l),
                teleports=int(tele.get("total")), jam_teleports=int(tele.get("jam")),
                collisions=int(safety.get("collisions")),
                geh_under5=sum(g < 5 for g in gehs), geh_points=len(gehs), geh_max=round(max(gehs), 2),
                mean_speed_kmh=round(float(trips.get("speed")) * 3.6, 1),
                mean_wait_s=round(float(trips.get("waitingTime")), 1), focus=focus_stats)


# ── 보고서 ──────────────────────────────────────────────────
def report(rep):
    s, lines = rep["structure"], []
    ok = not s["unexpected"] and not rep["new_warnings"] and not rep["connectivity"]["unreachable"]
    lines.append(f"# 도로망 수정 전후 비교 — {'✅ 이상 없음' if ok else '⚠ 확인 필요'}\n")
    lines.append(f"수정 목록: `yeouido/corrections.json` · 고친 도로 {len(s['edited_edges'])}곳 "
                 f"({', '.join(s['edited_edges']) or '없음'}), 그중 포켓 {len(s['split_edges'])}곳, "
                 f"삭제한 도로 {len(s['removed_edges'])}개 ({', '.join(s['removed_edges']) or '없음'}), "
                 f"새 도로 {len(s['new_roads'])}개 ({', '.join(s['new_roads']) or '없음'})\n")
    lines.append("## 1. 바뀐 범위\n")
    lines.append(f"- 새 도로 {len(s['added_edges'])}개 ({', '.join(s['added_edges'])}), "
                 f"없어진 도로 {len(s['removed_edges'])}개, 속성이 바뀐 도로 {len(s['changed_edges'])}개")
    for e in s["changed_edges"]:
        why = "수정한 도로" if e["split"] else "수정한 교차로에 붙은 도로" if e["expected"] else "⚠ 수정한 곳 밖"
        lines.append(f"  - {e['edge']} {e['name']}: " + ", ".join(f"{k} {v[0]}→{v[1]}" for k, v in e.items()
                                                                  if isinstance(v, list)) + f" ({why})")
    lines.append(f"- 차로 연결이 바뀐 교차로 {len(s['lane_links'])}곳 "
                 f"(수정한 도로 앞뒤 {sum(x['expected'] for x in s['lane_links'])}곳)")
    lines.append(f"- 이동 가능 방향이 바뀐 교차로 {len(s['movements'])}곳 (0이어야 함)")
    for t in s["tls"]:
        lines.append(f"- 신호 {t['tls'][:50]}: 단계 {t['phases'][0]}→{t['phases'][1]}개, "
                     f"보호 좌회전 단계 {t['protected_phases'][0]}→{t['protected_phases'][1]}개"
                     + ("" if t["expected"] else " ⚠ 수정한 곳 밖"))
    lines.append(f"- 모양이 바뀐 교차로 {len(s['node_shapes'])}곳 "
                 f"(수정한 도로 앞뒤 {sum(x['expected'] for x in s['node_shapes'])}곳)")
    if s["unexpected"]:
        lines.append("\n**⚠ 수정한 곳 밖에서 바뀐 것**")
        lines += [f"- {u}" for u in s["unexpected"]]
    lines.append("\n## 2. netconvert 경고\n")
    lines.append(f"- 새로 생긴 경고 {len(rep['new_warnings'])}개, 없어진 경고 {len(rep['gone_warnings'])}개")
    lines += [f"  - 생김: {w}" for w in rep["new_warnings"]] + [f"  - 없어짐: {w}" for w in rep["gone_warnings"]]
    c = rep["connectivity"]
    lines.append("\n## 3. 연결성 (랜드마크 사이 최단 경로)\n")
    lines.append(f"- {c['pairs']}쌍 중 끊긴 경로 {len(c['unreachable'])}개, 길이가 바뀐 경로 {len(c['changed'])}개"
                 " (수정한 교차로를 지나는 경로는 교차로 모양 때문에 몇 m 바뀔 수 있음)")
    lines += [f"  - 끊김: {u}" for u in c["unreachable"]]
    lines += [f"  - {x['pair']}: {x['base']} → {x['new']} m" for x in c["changed"]]
    if rep.get("sim"):
        lines.append("\n## 4. 시뮬레이션 (같은 교통 수요, 70분, 충돌은 교차로 안 포함)\n")
        lines.append("| 시간대 | 도로망 | 투입 | 경로 오류 | GEH<5 | teleport | 충돌 | 평균 속도 | 평균 대기 |")
        lines.append("|---|---|---|---|---|---|---|---|---|")
        for level, both in rep["sim"].items():
            for label in ("base", "new"):
                r = both[label]
                lines.append(f"| {level} | {'수정 전' if label == 'base' else '수정 후'} | {r['inserted']}/{r['loaded']} | "
                             f"{r['route_errors']} | {r['geh_under5']}/{r['geh_points']} | {r['teleports']} | "
                             f"{r['collisions']} | {r['mean_speed_kmh']} km/h | {r['mean_wait_s']} s |")
        lines.append("\n수정한 진입로 (원래 도로 + 포켓, 측정 구간 합계)\n")
        lines.append("| 시간대 | 도로 | 도로망 | 통과 | 평균 속도 | 대기 시간 합 | 손실 시간 합 |")
        lines.append("|---|---|---|---|---|---|---|")
        for level, both in rep["sim"].items():
            for eid in s["split_edges"]:
                for label in ("base", "new"):
                    f = both[label]["focus"]
                    parts = [f.get(eid), f.get(eid + SUFFIX)]
                    parts = [p for p in parts if p]
                    if not parts:
                        continue
                    lines.append(f"| {level} | {eid} | {'수정 전' if label == 'base' else '수정 후'} | "
                                 f"{parts[0]['entered']} | {' / '.join(str(p['speed_kmh']) for p in parts)} km/h | "
                                 f"{sum(p['waiting_s'] for p in parts):.0f} s | {sum(p['time_loss_s'] for p in parts):.0f} s |")
    text = "\n".join(lines) + "\n"
    (DIR / "report.md").write_text(text)
    (DIR / "report.json").write_text(json.dumps(rep, ensure_ascii=False, indent=1))
    return text


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", nargs="*", choices=list(LEVELS), default=None,
                    help="시뮬레이션으로도 비교할 시간대 (예: --sim rush)")
    ap.add_argument("--no-build", action="store_true", help="이미 만든 base/new 도로망으로 비교만")
    args = ap.parse_args(argv)
    DIR.mkdir(parents=True, exist_ok=True)
    if not args.no_build:
        build()
    base, new = sumolib.net.readNet(str(BASE)), sumolib.net.readNet(str(NEW))
    s = structure(base, new)
    wb, wn = warnings(BASE), warnings(NEW)
    rep = dict(structure=s, new_warnings=sorted(wn - wb), gone_warnings=sorted(wb - wn),
               connectivity=connectivity(base, new))
    if args.sim:
        focus = set()
        for eid in s["split_edges"]:
            focus |= {eid, eid + SUFFIX}
        rep["sim"] = {}
        for level in args.sim:
            print(f"[compare] {level} 시뮬레이션 (수정 전 / 수정 후)...")
            rep["sim"][level] = {label: simulate(label, path, level, set(s["split_edges"]), focus)
                                 for label, path in (("base", BASE), ("new", NEW))}
    print(report(rep))
    print(f"[compare] → {DIR / 'report.md'}")


if __name__ == "__main__":
    main(sys.argv[1:])
