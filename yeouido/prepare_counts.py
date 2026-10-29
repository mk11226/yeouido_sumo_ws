"""3단계: TOPIS 엑셀 → 측정 지점별 평일 시간대별 평균 교통량(counts.json).

    python -m yeouido.prepare_counts

엑셀 한 행 = (일자, 지점, 방향, 세부 구간) 의 0~23시 교통량.
  1) 평일 행만 쓴다 (주말·공휴일 제외).
  2) 결측(빈 칸)이 있거나 하루 합이 0인 날은 검지기 고장으로 보고 버린다.
  3) 같은 날·지점·방향의 세부 구간(차로 묶음)을 더해 지점 교통량을 만든다.
  4) 여러 날을 평균해 24시간 프로필을 만든다.
"""
import json
from collections import defaultdict

import openpyxl

from yeouido.config import COUNT_POINTS, OUT, TOPIS_MONTHS
from yeouido.download import topis_path

COUNTS = OUT / "counts.json"


def read_month(path):
    wb = openpyxl.load_workbook(path, read_only=True)
    data_sheet = next(ws for ws in wb.worksheets if ws.title.startswith("20"))
    rows = data_sheet.iter_rows(values_only=True)
    head = list(next(rows))
    col = {name: i for i, name in enumerate(head)}
    hours = [col[f"{h}시"] for h in range(24)]
    daily = defaultdict(lambda: [0.0] * 24)   # (지점, 방향, 일자) → 24시간
    bad = set()
    for r in rows:
        pid = r[col["지점번호"]]
        if pid not in COUNT_POINTS or r[col["요일(2)"]] != "평일":
            continue
        key = (pid, r[col["방향"]], r[col["일자"]])
        vals = [r[i] for i in hours]
        if any(not isinstance(v, (int, float)) for v in vals):
            bad.add(key)
            continue
        daily[key] = [a + b for a, b in zip(daily[key], vals)]

    # 지점 좌표·방향 설명 (지점번호는 첫 방향 행에만 적혀 있어서 위에서 이어받는다)
    coords, last = {}, None
    for r in wb["수집지점 주소 및 좌표"].iter_rows(min_row=2, values_only=True):
        last = r[0] or last
        if last in COUNT_POINTS and r[1]:
            coords[(last, r[1])] = dict(lat=float(r[4]), lon=float(r[5]), desc=r[8])
    return daily, bad, coords


def main():
    days = defaultdict(list)
    coords, dropped = {}, 0
    for m in TOPIS_MONTHS:
        daily, bad, c = read_month(topis_path(m))
        coords.update(c)
        for key, prof in daily.items():
            if key in bad or sum(prof) <= 0:
                dropped += 1
                continue
            days[key[:2]].append(prof)

    out = {}
    for (pid, direction), profs in sorted(days.items()):
        mean = [sum(p[h] for p in profs) / len(profs) for h in range(24)]
        out[f"{pid}/{direction}"] = dict(
            point=pid, direction=direction, name=COUNT_POINTS[pid]["name"],
            days=len(profs), hourly=[round(v, 1) for v in mean],
            **coords[(pid, direction)])
    COUNTS.parent.mkdir(parents=True, exist_ok=True)
    COUNTS.write_text(json.dumps(out, ensure_ascii=False, indent=1))
    print(f"[counts] {len(out)}개 지점·방향, 고장으로 제외한 날 {dropped}개 → {COUNTS}")
    for k, v in out.items():
        h = v["hourly"]
        print(f"   {k:10s} {v['name']:12s} {v['days']:2d}일  03시 {h[3]:6.0f}  "
              f"08시 {h[8]:6.0f}  12시 {h[12]:6.0f}  {v['desc']}")


if __name__ == "__main__":
    main()
