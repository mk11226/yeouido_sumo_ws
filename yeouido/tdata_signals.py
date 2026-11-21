"""Build SUMO signal programs from Seoul T DATA recorded signal states.

The original network is retained. Only confidently matched, complete cycles
are installed through additional files; other controllers keep SUMO's program.
See README.md for provenance, commands and mapping limitations.
"""
import argparse
import csv
import json
import math
import os
import xml.etree.ElementTree as ET
import zipfile
from collections import Counter, defaultdict
from pathlib import Path

from yeouido.config import BBOX, LEVELS, NET, OUT, ROOT, sumo_home

DIRECTIONS = ('NT', 'NE', 'ET', 'SE', 'ST', 'SW', 'WT', 'NW')
MOVEMENTS = ('STSG', 'LTSG', 'UTSG')
FIELDS = tuple(f'{d}_{m}_STAT_NM' for d in DIRECTIONS for m in MOVEMENTS)
FIELD_INDEX = {f: i for i, f in enumerate(FIELDS)}
SOURCE = OUT / 'source' / 'signals'
RECORDS = SOURCE / 'observed_cycles_20261116.json'
BEARINGS = SOURCE / 'map_bearings.json'
REPORT = OUT / 'signals_tdata_report.json'
API = 'https://t-data.seoul.go.kr/apig/apiman-gateway/tapi/'
STATE = {'stop-And-Remain': 'r', 'protected-Movement-Allowed': 'G',
         'permissive-Movement-Allowed': 'g', 'protected-clearance': 'y',
         'permissive-clearance': 'y', 'dark': '?', 'stop-Then-Proceed': '?',
         'caution-Conflicting-Traffic': '?', 'pre-Movement': '?',
         'unavailable': '?'}


def signal_file(level):
    return OUT / f'signals_{level}.add.xml'


def additional_files(*paths, level=None):
    """Combine explicit additions and available measured signal programs."""
    levels = [level] if level else LEVELS
    return ','.join(str(p) for p in [*paths, *(signal_file(lv) for lv in levels)]
                    if Path(p).exists())


def controlled_ids(level):
    path = signal_file(level)
    if not path.exists():
        return []
    return [tl.get('id') for tl in ET.parse(path).getroot().findall('tlLogic')]


def activate_program(traci, tid, level, now):
    """Start the measured program at the common clock offset, including in live."""
    path = signal_file(level)
    root = ET.parse(path).getroot() if path.exists() else ET.Element('additional')
    program = next((p for p in root if p.get('id') == tid), None)
    tls = traci.trafficlight
    if program is None:
        tls.setProgram(tid, '0')
        tls.setPhase(tid, 0)
        return
    tls.setProgram(tid, program.get('programID'))
    durations = [float(p.get('duration')) for p in program.findall('phase')]
    phase_time = (now + float(program.get('offset'))) % sum(durations)
    for i, duration in enumerate(durations):
        if phase_time < duration:
            tls.setPhase(tid, i)
            tls.setPhaseDuration(tid, duration - phase_time)
            return
        phase_time -= duration


def csv_rows(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as f:
        yield from csv.DictReader(f)


def local_key():
    key = os.environ.get('TDATA_API_KEY')
    if not key and (ROOT / '.env').exists():
        for line in (ROOT / '.env').read_text().splitlines():
            if line.startswith('TDATA_API_KEY='):
                key = line.split('=', 1)[1].strip().strip('\"\'')
    if not key:
        raise RuntimeError('TDATA_API_KEY를 환경변수 또는 로컬 .env에 설정하세요.')
    return key


def probe(itst_id):
    """Check the subscribed CURRENT phase service without a legacy fallback."""
    from yeouido.tdata_current import CurrentClient
    data = CurrentClient().request(intersection=itst_id, rows=1)
    if not data:
        raise RuntimeError('교차로 신호 레코드가 없는 응답입니다.')
    row = data[0]
    fields = {k: v for k, v in row.items() if k.endswith('StatNm') and v}
    print(json.dumps(dict(itst_id=row.get('itstId'), time=row.get('trsmTm'),
                          date=[row.get(k) for k in ('trsmYear', 'trsmMt', 'trsmDy')],
                          states=fields), ensure_ascii=False, indent=2))


def extract(history, crossroads, records_out=RECORDS):
    """Extract 03/08/12h states from official CSV or ZIP, sorting packet times.

    Large city-wide files are streamed. Only project-area samples at the chosen
    hours enter memory. Out-of-order packets are sorted before compression;
    contradictory states at the same timestamp remain explicitly unknown.
    """
    from contextlib import ExitStack
    history, crossroads, records_out = Path(history), Path(crossroads), Path(records_out)
    places = {}
    if crossroads.suffix.lower() == '.json':
        catalog = json.loads(crossroads.read_text())['intersections']
        source_places = [(str(r['itst_id']), r['name'], float(r['lat']), float(r['lon'])) for r in catalog]
    else:
        source_places = [(r['ITST_ID'], r['ITST_NM'], float(r['MAP_CTPT_INT_LAT']),
                          float(r['MAP_CTPT_INT_LOT'])) for r in csv_rows(crossroads)]
    for itst, name, lat, lon in source_places:
        if BBOX['west'] <= lon <= BBOX['east'] and BBOX['south'] <= lat <= BBOX['north']:
            places[itst] = dict(name=name, lat=lat, lon=lon)
    wanted = {itst.encode() for itst in places}
    hour_level = {int(spec['hours'][0]): level for level, spec in LEVELS.items()}
    samples = defaultdict(dict)
    count = out_of_order = duplicates = conflicts = 0
    dates = set()
    latest = {}
    with ExitStack() as stack:
        if zipfile.is_zipfile(history):
            archive = stack.enter_context(zipfile.ZipFile(history))
            names = [n for n in archive.namelist() if n.lower().endswith('.csv')]
            if len(names) != 1:
                raise RuntimeError('신호 CSV 하나가 담긴 ZIP이 필요합니다.')
            stream = stack.enter_context(archive.open(names[0]))
        else:
            stream = stack.enter_context(history.open('rb'))
        header = next(csv.reader([next(stream).decode('utf-8-sig')]))
        indices = [header.index(k) for k in FIELDS]
        if header[1] != 'TRSM_UTC_TIME' or header[5] != 'TRSM_HR' or header[8] != 'ITST_ID':
            raise RuntimeError('신호 CSV 앞쪽 열 순서가 예상과 다릅니다.')
        for line in stream:
            prefix = line.split(b',', 9)
            if len(prefix) < 10 or prefix[8].strip(b'"') not in wanted:
                continue
            hour = int(prefix[5].strip(b'"'))
            level = hour_level.get(hour)
            if level is None:
                continue
            row = next(csv.reader([line.decode('utf-8')]))
            itst = row[8]
            timestamp = float(row[1]) / 1000
            state = ''.join(STATE.get(row[i], '?' if row[i] else '-') for i in indices)
            key = (itst, level)
            if timestamp < latest.get(key, timestamp):
                out_of_order += 1
            latest[key] = max(timestamp, latest.get(key, timestamp))
            existing = samples[key].get(timestamp)
            if existing is not None:
                duplicates += 1
                if existing != state:
                    conflicts += 1
                    state = '?' * len(FIELDS)
            samples[key][timestamp] = state
            count += 1
            dates.add('-'.join(row[2:5]))
    timelines = {}
    for key, series in samples.items():
        runs = []
        for timestamp, state in sorted(series.items()):
            if runs and runs[-1][2] == state and timestamp - runs[-1][1] <= 3:
                runs[-1][1] = timestamp
            else:
                runs.append([timestamp, timestamp, state])
        timelines[key] = runs
    result = dict(source='서울특별시 T DATA 신호 정보 이력',
                  source_url='https://t-data.seoul.go.kr/', source_file=history.name,
                  source_bytes=history.stat().st_size, selected_rows=count,
                  out_of_order_packets=out_of_order, duplicate_timestamps=duplicates,
                  conflicting_timestamps=conflicts,
                  dates=sorted(dates), window_minutes=60, fields=FIELDS,
                  intersections={itst: {**places[itst], 'levels': {
                      level: timelines[(itst, level)] for level in LEVELS
                      if (itst, level) in timelines}}
                      for itst in places if any((itst, lv) in timelines for lv in LEVELS)})
    records_out.parent.mkdir(parents=True, exist_ok=True)
    records_out.write_text(json.dumps(result, ensure_ascii=False, separators=(',', ':')) + '\n')
    print(f'[signals] {count:,}개 관측 → {len(result["intersections"])}개 교차로, '
          f'역순 패킷 {out_of_order:,}, 모순 시각 {conflicts:,} → {records_out}')
    return result


def bearing(x, y):
    return math.degrees(math.atan2(x, y)) % 360


def angle_diff(a, b):
    return abs((a - b + 180) % 360 - 180)


def lane_bearing(lane):
    shape = lane.getShape()
    end = shape[-1]
    # Last 20m of approach; avoid a tiny stop-line geometry segment.
    point = shape[0]
    for p in reversed(shape[:-1]):
        point = p
        if math.dist(p, end) >= 20:
            break
    return bearing(point[0] - end[0], point[1] - end[1])


def map_links(tls, data, group_bearings):
    """Map each controlled link to a measured incoming compass direction.

    Official MAP lane shapes refine compass bearings on skewed intersections.
    Reject ambiguous approaches and controllers spanning multiple junctions.
    """
    connections = tls.getConnections()
    nodes = {c[0].getEdge().getToNode() for c in connections}
    if len(nodes) != 1:
        raise ValueError('여러 교차로를 묶은 제어기')
    active = {FIELDS[i].split('_')[0] for runs in data['levels'].values()
              for _, _, state in runs for i, c in enumerate(state) if c != '-'}
    directions = {d: group_bearings.get(d, DIRECTIONS.index(d) * 45) for d in active}
    if len(directions) < 2:
        raise ValueError('유효 진입 방향 부족')
    mapping = {}
    incoming = {}
    for lane, target, index in connections:
        if lane.getID() not in incoming:
            ranked = sorted((angle_diff(lane_bearing(lane), a), d) for d, a in directions.items())
            if ranked[0][0] > 35 or (len(ranked) > 1 and ranked[1][0] - ranked[0][0] < 12):
                raise ValueError('진입 방향 불일치/모호')
            incoming[lane.getID()] = ranked[0][1]
        matches = [c for c in lane.getOutgoing() if c.getToLane() == target
                   and c.getTLSID() == tls.getID() and c.getTLLinkIndex() == index]
        if len(matches) != 1:
            raise ValueError('차로 연결 모호')
        direction = incoming[lane.getID()]
        turn = matches[0].getDirection()
        movement = {'s': 'STSG', 'l': 'LTSG', 'L': 'LTSG', 't': 'UTSG',
                    'r': 'STSG', 'R': 'STSG'}.get(turn)
        if movement is None:
            raise ValueError('지원하지 않는 회전')
        item = (direction, movement, turn, matches[0])
        if index in mapping and mapping[index][:3] != item[:3]:
            raise ValueError('하나의 신호 인덱스에 다른 이동류')
        mapping[index] = item
    if set(mapping) != set(range(max(mapping) + 1)):
        raise ValueError('신호 인덱스 누락')
    return mapping


def translated(state, mapping):
    output = []
    for index in range(len(mapping)):
        d, movement, turn, _ = mapping[index]
        c = state[FIELD_INDEX[f'{d}_{movement}_STAT_NM']]
        if c == '-' and movement == 'UTSG':
            # A modeled U-turn without a separate arrow is permitted only with
            # its approach's observed left arrow, and must yield.
            c = state[FIELD_INDEX[f'{d}_LTSG_STAT_NM']]
            if c == 'G':
                c = 'g'
        if c == '-' and movement in ('LTSG', 'UTSG'):
            # Missing protected arrows are ambiguous; retain the old controller.
            raise ValueError('좌회전/유턴 관측 없음')
        if c in '?-':
            raise ValueError('신호 상태 결측/점멸')
        if turn in ('r', 'R') and c == 'G':
            c = 'g'  # right turns yield; no invented red-light permission
        output.append(c)
    return ''.join(output)


def complete_cycle(runs, mapping, validator=None):
    phases = []
    translation_errors = []
    for start, last, state in sorted(runs):
        try:
            value = translated(state, mapping)
        except ValueError as exc:
            # Keep a barrier at the missing observation instead of discarding
            # the entire hour or holding a guessed color through the gap.
            value = None
            translation_errors.append(str(exc))
        if phases and value == phases[-1][2] and start - phases[-1][1] <= 3:
            phases[-1][1] = last
        else:
            phases.append([start, last, value])
    # Find complete, uninterrupted cycles, then require a second matching
    # observation. The two cycles may be separated by telemetry gaps.
    # Discard partial first/last phases and never fill a gap with a held state.
    candidates = []
    for i in range(1, len(phases) - 2):
        if phases[i][2] is None or phases[i - 1][2] is None or phases[i][0] - phases[i - 1][1] > 3:
            continue  # first phase of a fresh segment may be partial
        for j in range(i + 2, len(phases) - 1):
            cycle = phases[j][0] - phases[i][0]
            if cycle > 240:
                break
            if (phases[j][0] - phases[j - 1][1] > 3
                    or phases[j - 1][2] is None or phases[j][2] is None):
                # A gap anywhere in this window invalidates longer windows too.
                break
            a = phases[i:j]
            if phases[i][2] != phases[j][2] or cycle < 30:
                continue
            durations = [phases[k + 1][0] - phases[k][0] for k in range(i, j)]
            if any(not .2 <= d <= 180 for d in durations):
                continue
            if any(phases[k + 1][0] - phases[k][1] > 3 for k in range(i, j)):
                continue
            candidates.append((cycle, phases[i][0], list(zip(durations, [p[2] for p in a]))))
    repeated = []
    signatures = defaultdict(list)
    for candidate in candidates:
        signature = tuple(s for _, s in candidate[2])
        for other in signatures[signature]:
            if abs(candidate[1] - other[1]) < min(candidate[0], other[0]) - 1:
                continue
            if all(abs(a[0] - b[0]) <= 2.5 for a, b in zip(candidate[2], other[2])):
                repeated.append(candidate)
                break
        signatures[signature].append(candidate)
    if not repeated:
        if not any(p[2] is not None for p in phases) and translation_errors:
            raise ValueError(translation_errors[0])
        raise ValueError('반복된 완전 주기 없음/관측 간격 과다')
    valid = []
    errors = []
    for candidate in repeated:
        try:
            validate_cycle(candidate[2], mapping)
            if validator is not None:
                validator(candidate[2])
            valid.append(candidate)
        except ValueError as exc:
            errors.append(str(exc))
    if not valid:
        raise ValueError(errors[0])
    # Prefer a frequently observed cycle length rather than an outlier.
    common = Counter(round(c[0]) for c in valid).most_common(1)[0][0]
    return min(valid, key=lambda c: abs(c[0] - common))


def validate_cycle(phases, mapping):
    """Require observed yellow for each ending green, without inventing all-red.

    Real overlapping movements can keep one approach green while another clears.
    A junction-wide all-red phase is therefore not required. Foe checks run
    separately on each candidate before a program can be installed.
    """
    states = [s for _, s in phases]
    for a, b in zip(states, states[1:] + states[:1]):
        if any(x in 'Gg' and y not in 'Ggy' for x, y in zip(a, b)):
            raise ValueError('녹색 종료 시 황색 관측 없음')
    for index in mapping:
        if not any(state[index] in 'Gg' for _, state in phases):
            raise ValueError('주기 내 녹색 없는 이동류')


def check_conflicts(phases, mapping):
    for _, state in phases:
        protected_or_yellow = [i for i, c in enumerate(state) if c in 'Gy']
        for n, i in enumerate(protected_or_yellow):
            a = mapping[i][3]
            for j in protected_or_yellow[n + 1:]:
                if state[i] != 'G' and state[j] != 'G':
                    continue
                b = mapping[j][3]
                if a.getJunction() == b.getJunction() and a.getJunction().areFoes(
                        a.getJunctionIndex(), b.getJunctionIndex()):
                    raise ValueError('SUMO 형상에서 보호녹색·대향 녹색/황색 충돌')


def official_bearings(nodes_path, connections_path):
    """Approach bearing from official MAP node offsets (cm) and signal groups."""
    groups = defaultdict(set)
    code = {'10': 'NT', '20': 'ET', '30': 'ST', '40': 'WT',
            '50': 'NE', '60': 'SE', '70': 'SW', '80': 'NW'}
    for r in csv_rows(connections_path):
        d = code.get(r['SIGNLGROUP_CD'])
        if d:
            groups[(r['ITST_ID'], r['LANE_ID'])].add(d)
    farthest = {}
    for r in csv_rows(nodes_path):
        key = (r['ITST_ID'], r['LANE_ID'])
        if key not in groups:
            continue
        x, y = float(r['OFST_XAXS_CRDNT']), float(r['OFST_YAXS_CRDNT'])
        if key not in farthest or math.hypot(x, y) > math.hypot(*farthest[key]):
            farthest[key] = x, y
    samples = defaultdict(list)
    for key, xy in farthest.items():
        for d in groups[key]:
            a = bearing(*xy)
            # Reject MAP shape/group inconsistencies rather than rotating a group.
            if angle_diff(a, DIRECTIONS.index(d) * 45) <= 50:
                samples[(key[0], d)].append(a)
    result = defaultdict(dict)
    for (itst, d), angles in samples.items():
        x = sum(math.sin(math.radians(a)) for a in angles)
        y = sum(math.cos(math.radians(a)) for a in angles)
        result[itst][d] = bearing(x, y)
    return result


def build(nodes_path=None, connections_path=None, records_path=None):
    sumo_home()
    import sumolib
    source = json.loads(Path(records_path or RECORDS).read_text())
    if source['fields'] != list(FIELDS):
        raise RuntimeError('관측 필드 목록이 다릅니다. 이력 자료를 다시 추출하세요.')
    net = sumolib.net.readNet(str(NET), withInternal=True)
    # Explicitly confirmed junction sequencing takes precedence over automatic
    # matching to recorded programs, including after future history updates.
    manual_tls = {net.getEdge('1180030100').getTLS().getID()} if net.hasEdge('1180030100') else set()
    if nodes_path and connections_path:
        bearings = official_bearings(nodes_path, connections_path)
        bearings = {itst: values for itst, values in bearings.items()
                    if itst in source['intersections']}
        BEARINGS.write_text(json.dumps(dict(
            source='서울특별시 T DATA 노드 MAP / 신호연결 MAP 정보 (2026-10-20)',
            source_urls=['https://t-data.seoul.go.kr/dataprovide/trafficdataviewfile.do?data_id=10147',
                         'https://t-data.seoul.go.kr/dataprovide/trafficdataviewfile.do?data_id=10146'],
            bearings=bearings), ensure_ascii=False, indent=2) + '\n')
    else:
        bearings = json.loads(BEARINGS.read_text())['bearings']
    positions = {t.getID(): {c[0].getEdge().getToNode().getCoord() for c in t.getConnections()}
                 for t in net.getTrafficLights()}
    nearest = {}
    for itst, data in source['intersections'].items():
        xy = net.convertLonLat2XY(data['lon'], data['lat'])
        distance, tid = min((min(math.dist(xy, p) for p in points), tid)
                            for tid, points in positions.items())
        if distance <= 50:
            nearest[itst] = (tid, distance)
    duplicates = Counter(tid for tid, _ in nearest.values())
    report = dict(source=source['source'], source_url=source['source_url'],
                  observations_file=Path(records_path or RECORDS).name,
                  dates=source['dates'], max_snap_m=50, total_sumo_tls=len(positions),
                  observed_intersections=len(source['intersections']),
                  real_world_equivalence_verified=False,
                  derivation='partial observed cycles with modeled right-turn/U-turn assumptions',
                  validation=['repeated uninterrupted cycles', 'observed yellow for ending green',
                              'green for every mapped movement', 'no protected green versus SUMO foe green/yellow'],
                  applied=[], skipped=[])
    roots = {lv: ET.Element('additional') for lv in LEVELS}
    for itst, (tid, distance) in sorted(nearest.items()):
        data = source['intersections'][itst]
        try:
            if tid in manual_tls:
                raise ValueError('사용자 지정 보호 좌회전 순서 유지 (여의2교 남단)')
            if duplicates[tid] != 1:
                raise ValueError('여러 실제 교차로가 같은 SUMO 제어기에 매칭')
            mapping = map_links(net.getTLS(tid), data, bearings.get(itst, {}))
            programs = {}
            for lv in LEVELS:
                try:
                    if lv not in data['levels']:
                        raise ValueError('관측 없음')
                    cycle, anchor, phases = complete_cycle(
                        data['levels'][lv], mapping,
                        validator=lambda p: check_conflicts(p, mapping))
                    programs[lv] = (cycle, anchor, phases)
                except ValueError as exc:
                    report['skipped'].append(dict(itst_id=itst, name=data['name'],
                                                  tls_id=tid, level=lv, reason=str(exc)))
            if not programs:
                continue
            for lv, (cycle, anchor, phases) in programs.items():
                # Epoch phase origin gives common offsets across controllers.
                # The history transmission epoch is UTC; local 03/08/12h start
                # is recovered from the source timestamp's KST day boundary.
                start = math.floor((anchor + 9 * 3600) / 86400) * 86400 - 9 * 3600
                start += LEVELS[lv]['hours'][0] * 3600
                offset = (cycle - (anchor - start) % cycle) % cycle
                tl = ET.SubElement(roots[lv], 'tlLogic', id=tid, type='static',
                                   programID=f'tdata_{lv}', offset=f'{offset:.3f}')
                for duration, state in phases:
                    ET.SubElement(tl, 'phase', duration=f'{duration:.3f}', state=state)
            report['applied'].append(dict(itst_id=itst, name=data['name'], tls_id=tid,
                                           distance_m=round(distance, 1),
                                           cycles_s={lv: round(p[0], 2) for lv, p in programs.items()}))
        except ValueError as exc:
            report['skipped'].append(dict(itst_id=itst, name=data['name'], tls_id=tid, reason=str(exc)))
    if not report['applied']:
        REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        raise RuntimeError(f'검증된 주기가 없어 적용하지 않았습니다. {REPORT}')
    for lv, root in roots.items():
        ET.indent(root)
        ET.ElementTree(root).write(signal_file(lv), encoding='utf-8', xml_declaration=True)
        cfg = OUT / f'yeouido_{lv}.sumocfg'
        tree = ET.parse(cfg)
        inp = tree.getroot().find('input')
        node = inp.find('additional-files')
        if node is None:
            node = ET.SubElement(inp, 'additional-files', value='')
        files = [p for p in node.get('value', '').split(',') if p and not p.startswith('signals_')]
        node.set('value', ','.join(files + [signal_file(lv).name]))
        ET.indent(tree)
        tree.write(cfg, encoding='utf-8', xml_declaration=True)
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(f'[signals] 관측 기반 주기 적용 {len(report["applied"])} / {len(positions)}, '
          f'매칭 후 제외 {len(report["skipped"])}. 현장 동일성은 미검증. 보고서: {REPORT}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--probe', metavar='ITST_ID', help='발급받은 신호 정보 API 확인 (300초 제한)')
    ap.add_argument('--history', type=Path, help='공식 신호 상태 이력 CSV 또는 ZIP')
    ap.add_argument('--crossroads', type=Path, help='공식 교차로 MAP CSV 또는 JSON')
    ap.add_argument('--records-out', type=Path, default=RECORDS, help='추출 관측 저장 위치')
    ap.add_argument('--observations', type=Path, default=RECORDS, help='신호 생성에 사용할 추출 관측 JSON')
    ap.add_argument('--nodes', type=Path, help='공식 노드 MAP CSV')
    ap.add_argument('--connections', type=Path, help='공식 신호연결 MAP CSV')
    args = ap.parse_args()
    if args.probe:
        probe(args.probe)
        return
    if args.history:
        if not args.crossroads:
            ap.error('--history에는 --crossroads가 필요합니다.')
        extract(args.history, args.crossroads, args.records_out)
    if args.nodes and args.connections:
        build(args.nodes, args.connections, args.records_out if args.history else args.observations)
    elif not args.history:
        if args.nodes or args.connections:
            ap.error('--nodes와 --connections를 함께 지정하세요.')
        build(records_path=args.observations)


if __name__ == '__main__':
    main()
