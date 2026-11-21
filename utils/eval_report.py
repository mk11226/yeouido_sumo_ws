"""평가 시험지 결과(test.py --eval-set 이 저장한 JSON)를 종류별 표로 보여 준다.

    python -m utils.eval_report results/bc_…/eval_set_results.json
    python -m utils.eval_report results/sumo_expert/eval_set_results.json results/bc_…/eval_set_results.json
                                                   ← 여러 개면 모델별로 나란히 비교

같은 시험지를 푼 결과끼리만 비교한다 (문제 id 목록이 다르면 경고).
"""
import argparse
import json
from collections import OrderedDict
from pathlib import Path

LEVEL_NAME = {"night": "심야", "day": "주간", "rush": "출근"}
GROUPS = [("전체", lambda r: ["전체"]),
          ("시간대", lambda r: [LEVEL_NAME.get(r.get("level"), r.get("level"))]),
          ("종류", lambda r: [r.get("category", "-")]),
          ("샛강 연결로", lambda r: r.get("bridges") or ["건너지 않음"])]


def load(path):
    data = json.loads(Path(path).read_text())
    return data["name"], data["records"]


def stats(records):
    n = len(records)
    col = sum(r["ended_by"] == "collision" for r in records)
    arr = [r for r in records if r["ended_by"] == "arrival"]
    speed_n = sum(r["speed_n"] for r in records)
    return dict(n=n, collision=col / n, success=len(arr) / n,
                timeout=sum(r["ended_by"] in ("timeout", "teleport") for r in records) / n,   # 순간이동 = 못 가서 실패
                speed=sum(r["speed_sum"] for r in records) / speed_n * 3.6 if speed_n else 0.0,
                red=sum(r.get("red_lights") or 0 for r in records),
                minutes=(sum(r["length"] for r in arr) / len(arr) * 0.5 / 60) if arr else None,
                n_col=col)


def grouped(records):
    """{그룹 제목: {값: [record]}} — 표 순서를 고정한다."""
    out = OrderedDict()
    for title, key in GROUPS:
        g = OrderedDict()
        for r in records:
            for k in key(r):
                g.setdefault(k, []).append(r)
        out[title] = g
    return out


def print_single(name, records):
    print(f"\n=== {name} ({len(records)}문제) ===")
    print(f"{'':16s} {'문제':>4s} {'충돌률':>7s} {'완주율':>7s} {'시간초과':>7s} {'평균속도':>9s} "
          f"{'완주 소요':>8s} {'빨간불':>5s}")
    for title, g in grouped(records).items():
        if title != "전체":
            print(f"[{title}]")
        for k, rs in g.items():
            s = stats(rs)
            mins = f"{s['minutes']:.1f}분" if s["minutes"] is not None else "-"
            print(f"  {k:14s} {s['n']:4d} {s['collision']:7.1%} {s['success']:7.1%} {s['timeout']:7.1%} "
                  f"{s['speed']:6.1f}km/h {mins:>8s} {s['red']:5d}")
    bad = [r for r in records if r["ended_by"] != "arrival"]
    if bad:
        print("완주 못 한 문제:")
        for r in bad:
            print(f"  #{r['id']:<3d} {r['ended_by']:9s} {r.get('level', ''):5s} "
                  f"{r.get('origin', '')} → {r.get('destination', '')} (워밍업 {r.get('snapshot_time', '-')})")


def print_compare(results):
    names = [n for n, _ in results]
    ids = [sorted(r["id"] for r in recs) for _, recs in results]
    if any(i != ids[0] for i in ids):
        print("※ 경고: 결과마다 푼 문제 목록이 다릅니다 (같은 시험지가 아니면 비교 의미 없음)")
    width = max(12, *(len(n) for n in names))
    print(f"\n=== 충돌률 / 완주율 비교 ===")
    print(f"{'':16s} {'문제':>4s} " + " ".join(f"{n:>{width}s}" for n in names))
    per = [grouped(recs) for _, recs in results]
    for title in per[0]:
        if title != "전체":
            print(f"[{title}]")
        for k in per[0][title]:
            cells = []
            for g in per:
                rs = g[title].get(k)
                if not rs:
                    cells.append(f"{'-':>{width}s}")
                    continue
                s = stats(rs)
                cells.append(f"{s['collision']:5.1%} / {s['success']:5.1%}".rjust(width))
            print(f"  {k:14s} {len(per[0][title][k]):4d} " + " ".join(cells))
    print("(칸 = 충돌률 / 완주율)")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results", nargs="+", help="test.py --eval-set 결과 JSON")
    args = ap.parse_args(argv)
    results = [load(p) for p in args.results]
    if len(results) == 1:
        print_single(*results[0])
    else:
        print_compare(results)


if __name__ == "__main__":
    main()
