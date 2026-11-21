"""5단계 최종 평가표: 시험지 결과 JSON 여러 개를 같은 표로 정리하고, 최종 모델을 과제 기준으로 점검한다.

    python -m eval_set.final_report --final "PPO + 재탐색=results/reroute_check/ppo_best_reroute_random_60_s1.json" \\
        --compare "선생님=results/sumo_expert/random_60_s1_results.json" "PPO=results/ppo_v1/best/random_60_s1_results.json"

입력: test.py --eval-set 결과 또는 eval_set/reroute_verify.py 결과 (records 가 있으면 됨).
출력: results/final/final_eval.md (표 + 과제 기준 점검), results/final/final_eval.csv (모델별 지표), 콘솔에 같은 표.
"""
import argparse
import csv
import json
import os

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LEVELS = (("night", "심야"), ("day", "주간"), ("rush", "출근"))


def metrics(path):
    with open(os.path.join(BASE, path), encoding="utf-8") as f:
        recs = json.load(f)["records"]
    n = len(recs)
    cnt = lambda k: sum(r["ended_by"] == k for r in recs)
    ok = [r for r in recs if r["ended_by"] != "start_failed"]
    arr = [r for r in recs if r["ended_by"] == "arrival"]
    mean = lambda xs: sum(xs) / len(xs) if xs else None
    m = dict(problems=n, arrival=cnt("arrival"), collision=cnt("collision"), teleport=cnt("teleport"),
             timeout=cnt("timeout"), start_failed=cnt("start_failed"),
             red_lights=sum(r.get("red_lights") or 0 for r in ok),
             arrival_min=mean([r["length"] * 0.5 / 60 for r in arr]),
             shield_per_problem=mean([r["shield_steps"] for r in ok if r.get("shield_steps") is not None]),
             reroutes=sum(r.get("reroutes") or 0 for r in ok),
             hv_total=mean([r["hv_total_mean"] for r in ok if r.get("hv_total_mean") is not None]),
             hv_near=mean([r["hv_near_mean"] for r in ok if r.get("hv_near_mean") is not None]),
             junction_overlaps=sum(r.get("junction_overlaps") or 0 for r in ok))
    for lv, _ in LEVELS:
        rs = [r for r in recs if r["level"] == lv]
        m[f"arrival_{lv}"] = sum(r["ended_by"] == "arrival" for r in rs)
        m[f"n_{lv}"] = len(rs)
        hv = [r["hv_near_mean"] for r in rs if r.get("hv_near_mean") is not None]
        hvt = [r["hv_total_mean"] for r in rs if r.get("hv_total_mean") is not None]
        m[f"hv_near_{lv}"] = mean(hv)
        m[f"hv_total_{lv}"] = mean(hvt)
    m["arrival_rate"] = m["arrival"] / n
    m["collision_rate"] = m["collision"] / n
    return m


def fmt(x, nd=1, pct=False):
    if x is None:
        return "-"
    return f"{x:.{nd}%}" if pct else f"{x:,.{nd}f}"


def table(rows):
    head = ["모델", "도착", "충돌률", "순간이동", "시간초과", "출발실패", "빨간불", "심야", "주간", "출근",
            "평균 도착 시간", "쉴드 개입/문제", "경로 변경"]
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for name, m in rows:
        lines.append("| " + " | ".join([
            name, f"{m['arrival']}/{m['problems']} ({m['arrival_rate']:.0%})", fmt(m["collision_rate"], 1, True),
            str(m["teleport"]), str(m["timeout"]), str(m["start_failed"]), str(m["red_lights"]),
            *[f"{m[f'arrival_{lv}']}/{m[f'n_{lv}']}" for lv, _ in LEVELS],
            f"{fmt(m['arrival_min'])}분", fmt(m["shield_per_problem"], 0), str(m["reroutes"])]) + " |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--final", required=True, help='"이름=결과.json" 최종 모델')
    ap.add_argument("--compare", nargs="*", default=[], help='"이름=결과.json" 비교 대상')
    ap.add_argument("--out-dir", default="results/final")
    args = ap.parse_args()
    split = lambda s: s.split("=", 1)
    fname, fpath = split(args.final)
    rows = [(fname, metrics(fpath))] + [(n, metrics(p)) for n, p in map(split, args.compare)]
    fm = rows[0][1]

    checks = [
        ("충돌률 < 5 %", fm["collision_rate"] < 0.05, f"{fm['collision_rate']:.1%} ({fm['collision']}/{fm['problems']})"),
        ("목적지 도착", fm["arrival"] > 0, f"{fm['arrival']}/{fm['problems']} ({fm['arrival_rate']:.0%})"),
        ("빨간불 통과 없음", fm["red_lights"] == 0, f"{fm['red_lights']}회"),
        ("HV 충분", None if fm["hv_total"] is None else fm["hv_total"] > 0,
         f"도로망 평균 {fmt(fm['hv_total'], 0)}대, ego 100 m 안 평균 {fmt(fm['hv_near'])}대"
         if fm["hv_total"] is not None else "기록 없음"),
        ("ego 순간이동 (이상 정체)", None, f"{fm['teleport']}/{fm['problems']} 문제 (300초 정체 시 SUMO 가 옮김 → 실패로 처리)"),
    ]
    hv_lines = ["| 시간대 | 도로망 일반 차량 (평균) | ego 100 m 안 (평균) |", "|---|---|---|"]
    for lv, ko in LEVELS:
        hv_lines.append(f"| {ko} | {fmt(fm[f'hv_total_{lv}'], 0)}대 | {fmt(fm[f'hv_near_{lv}'])}대 |")

    md = [f"# 최종 평가표 — {fname}", "",
          f"시험지 {fm['problems']}문제 (문제마다 시간대 · 출발 · 도착 · 워밍업 시점 · SUMO 시드 고정).", "",
          "## 모델 비교", "", table(rows), "",
          "## 과제 기준 점검 (최종 모델)", "", "| 기준 | 판정 | 값 |", "|---|---|---|"]
    for name, ok, val in checks:
        md.append(f"| {name} | {'—' if ok is None else ('통과' if ok else '미달')} | {val} |")
    md += ["", "## 주변 일반 차량 수 (최종 모델 주행 중)", "", *hv_lines, ""]
    os.makedirs(os.path.join(BASE, args.out_dir), exist_ok=True)
    md_path = os.path.join(BASE, args.out_dir, "final_eval.md")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    csv_path = os.path.join(BASE, args.out_dir, "final_eval.csv")
    keys = list(rows[0][1])
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["model", *keys])
        for name, m in rows:
            w.writerow([name, *[m[k] for k in keys]])
    print("\n".join(md))
    print(f"→ {md_path}\n→ {csv_path}")


if __name__ == "__main__":
    main()
