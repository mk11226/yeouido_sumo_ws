"""Read/record the subscribed CURRENT signal-state API, without legacy fallback.

python -m yeouido.tdata_current
python -m yeouido.tdata_current --seconds 480 --interval 3

Raw recordings are not time-of-day operating plans. This tool never installs
a state snapshot as a repeating signal program or overwrites another time slot.
"""
import argparse
import concurrent.futures
import datetime as dt
import gzip
import json
import math
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from yeouido.tdata_signals import API, SOURCE, local_key

ENDPOINT = API + 'v2xSignalPhaseCurrentInfo/1.0'
DISTRICTS = ('11190', '11200', '11150', '11170', '11140')


class CurrentAPIError(RuntimeError):
    def __init__(self, message, status=None, retry_after=None, quota=None):
        super().__init__(message)
        self.status = status
        self.retry_after = retry_after
        self.quota = quota or {}


class CurrentClient:
    def __init__(self, key=None, timeout=20, max_requests=900):
        self._key = key or local_key()
        self.timeout = timeout
        self.max_requests = max_requests
        self.request_count = 0
        self.quota = {}
        self._lock = threading.Lock()

    def remember_quota(self, headers):
        names = ('X-RateLimit-Limit', 'X-RateLimit-Remaining', 'X-RateLimit-Reset')
        with self._lock:
            for name in names:
                value = headers.get(name)
                if value is not None:
                    self.quota[name] = str(value)

    def request(self, *, district=None, intersection=None, page=1, rows=5000):
        if not district and not intersection:
            raise ValueError('sggCd 또는 itstId가 필요합니다.')
        if not 1 <= rows <= 5000 or page < 1:
            raise ValueError('numOfRows는 1~5000, pageNo는 1 이상이어야 합니다.')
        with self._lock:
            if self.request_count >= self.max_requests:
                raise CurrentAPIError('이번 실행의 요청 예산을 소진했습니다. 추가 호출을 중단합니다.')
            if int(self.quota.get('X-RateLimit-Remaining', '1')) <= 0:
                raise CurrentAPIError('서버가 호출 잔여량 0을 통보했습니다. 추가 호출을 중단합니다.',
                                      status=429, quota=dict(self.quota))
            self.request_count += 1
        params = dict(apikey=self._key, type='json', pageNo=page, numOfRows=rows)
        if district:
            params['sggCd'] = district
        if intersection:
            params['itstId'] = intersection
        url = ENDPOINT + '?' + urllib.parse.urlencode(params)
        try:
            with urllib.request.urlopen(url, timeout=self.timeout) as response:
                self.remember_quota(getattr(response, 'headers', {}))
                data = json.load(response)
        except urllib.error.HTTPError as exc:
            message = exc.read().decode(errors='replace').replace(self._key, '[REDACTED]')
            retry_after = exc.headers.get('Retry-After')
            try:
                error = json.loads(message)
                retry_after = error.get('retryAfterSeconds', retry_after)
                self.remember_quota(error.get('headers', {}))
            except (ValueError, AttributeError):
                pass
            raise CurrentAPIError(f'CURRENT API HTTP {exc.code}: {message[:300]}',
                                  exc.code, retry_after, dict(self.quota)) from None
        except (OSError, ValueError):
            raise CurrentAPIError('CURRENT API 연결 또는 JSON 응답 오류 (키·요청 URL 생략)') from None
        if not isinstance(data, list) or any(not isinstance(r, dict) or 'itstId' not in r for r in data):
            raise CurrentAPIError('CURRENT API가 교차로 목록 대신 오류/알 수 없는 형식을 반환했습니다.')
        return data

    def district(self, code):
        rows = []
        for page in range(1, 20):
            batch = self.request(district=code, page=page)
            rows.extend(batch)
            if len(batch) < 5000:
                return rows
        raise CurrentAPIError('페이지 상한에 도달했습니다. 전체 수집 완료로 처리하지 않습니다.')

    def region(self, districts=DISTRICTS):
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(districts)) as pool:
            batches = list(pool.map(self.district, districts))
        latest = {}
        for row in (r for batch in batches for r in batch):
            key = str(row['itstId'])
            if float(row.get('trsmUtcTime') or 0) >= float(latest.get(key, {}).get('trsmUtcTime') or 0):
                latest[key] = row
        return list(latest.values())


def summarize(rows, wanted, received_at, max_age=5):
    selected = {str(r['itstId']): r for r in rows if str(r['itstId']) in wanted}
    fresh = {key for key, row in selected.items()
             if row.get('trsmUtcTime') is not None
             and -.5 <= received_at - float(row['trsmUtcTime']) / 1000 <= max_age}
    return dict(requested_intersections=len(wanted), received_intersections=len(selected),
                fresh_intersections=len(fresh), stale_intersections=len(selected) - len(fresh),
                missing_intersection_ids=sorted(wanted - selected.keys()),
                stale_intersection_ids=sorted(selected.keys() - fresh),
                fresh_intersection_ids=sorted(fresh))


def collect(seconds, interval, directory, districts=DISTRICTS, max_requests=900):
    if seconds < 0 or interval < 1:
        raise ValueError('seconds는 0 이상, interval은 1초 이상이어야 합니다.')
    expected_requests = (math.ceil(seconds / interval) + 1) * len(districts)
    if expected_requests > max_requests:
        raise ValueError(f'예상 {expected_requests}회가 실행 예산 {max_requests}회를 초과합니다. '
                         '조회 간격을 늘리거나 구 범위를 줄이세요.')
    catalog = json.loads((SOURCE / 'crossroads.json').read_text())
    wanted = {r['itst_id'] for r in catalog['intersections']}
    directory.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    path = directory / f'current_{stamp}.jsonl.gz'
    client = CurrentClient(max_requests=max_requests)
    started = time.monotonic()
    manifest = dict(endpoint=ENDPOINT, districts=list(districts), interval_s=interval,
                    requested_duration_s=seconds, recording=str(path),
                    max_requests=max_requests,
                    full_operating_plan=False, simulation_programs_modified=False,
                    completed=False, frames=0, unique_received_ids=[], unique_fresh_ids=[], error=None)
    received, fresh = set(), set()
    next_log = 0
    try:
        with gzip.open(path, 'xt', encoding='utf-8') as output:
            while True:
                round_start = time.monotonic()
                rows = client.region(districts)
                now = time.time()
                selected = [r for r in rows if str(r['itstId']) in wanted]
                summary = summarize(selected, wanted, now)
                received.update(str(r['itstId']) for r in selected)
                fresh.update(summary['fresh_intersection_ids'])
                output.write(json.dumps(dict(received_utc=now, summary=summary,
                                             records=selected), ensure_ascii=False) + '\n')
                manifest['frames'] += 1
                manifest['latest_summary'] = summary
                elapsed = time.monotonic() - started
                if elapsed >= next_log:
                    print(f'[current] {elapsed:.0f}s / {seconds:g}s, frames={manifest["frames"]}, '
                          f'received={len(selected)}/{len(wanted)}, '
                          f'fresh={summary["fresh_intersections"]}', flush=True)
                    next_log = elapsed + 30
                if elapsed >= seconds:
                    manifest['completed'] = True
                    break
                time.sleep(max(0, min(interval - (time.monotonic() - round_start), seconds - elapsed)))
    except CurrentAPIError as exc:
        manifest['error'] = dict(message=str(exc), status=exc.status, retry_after=exc.retry_after,
                                 quota=exc.quota)
        # Abort rather than hammering a service that has rate-limited requests.
        raise
    finally:
        manifest['elapsed_s'] = round(time.monotonic() - started, 2)
        manifest['request_count'] = client.request_count
        manifest['unique_received_ids'] = sorted(received)
        manifest['unique_fresh_ids'] = sorted(fresh)
        path.with_suffix('.metadata.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
        print(f'[current] 기록: {path}', flush=True)
    return path


def analyze(path, destination):
    """Extract candidate cycles, retaining gaps/stale samples and unknown coverage.

    This deliberately does not install current-night observations as 03/08/12h
    plans, infer missing movements, or mark proximity matches as verified.
    """
    from collections import Counter, defaultdict
    from zoneinfo import ZoneInfo
    from yeouido.config import NET, sumo_home
    from yeouido.tdata_signals import (
        BEARINGS, FIELDS, STATE, check_conflicts, complete_cycle, map_links)
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET), withInternal=True)
    catalog = json.loads((SOURCE / 'crossroads.json').read_text())['intersections']
    bearings = json.loads(BEARINGS.read_text())['bearings']
    samples = defaultdict(dict)
    stale = Counter()
    frames = 0
    for_frame = [d.lower() + m[0] + m[1:].lower() + 'StatNm'
                 for d, m, *_ in (field.split('_') for field in FIELDS)]
    with gzip.open(path, 'rt', encoding='utf-8') as source:
        for line in source:
            frame = json.loads(line)
            frames += 1
            for row in frame['records']:
                itst = str(row['itstId'])
                stamp = float(row.get('trsmUtcTime') or 0) / 1000
                age = frame['received_utc'] - stamp
                if not -.5 <= age <= 5:
                    stale[itst] += 1
                    continue
                state = ''.join(STATE.get(row.get(key), '?' if row.get(key) else '-')
                                for key in for_frame)
                samples[itst][stamp] = state
    positions = {t.getID(): {c[0].getEdge().getToNode().getCoord() for c in t.getConnections()}
                 for t in net.getTrafficLights()}
    mapped = defaultdict(list)
    for entry in catalog:
        xy = net.convertLonLat2XY(entry['lon'], entry['lat'])
        distance, tid = min((min(math.dist(xy, point) for point in points), tid)
                            for tid, points in positions.items())
        if distance <= 50:
            mapped[tid].append({**entry, 'distance_m': round(distance, 2)})
    results = []
    for tid in sorted(positions):
        item = dict(tls_id=tid, mapping_verified=False, installed=False,
                    current_candidates=mapped[tid], current_cycle=None)
        entries = mapped[tid]
        if not entries:
            item['reason'] = '공식 MAP 대응 후보 없음'
        elif len(entries) != 1:
            item['reason'] = '공식 MAP 후보가 여러 개여서 제어기 대응 모호'
        else:
            entry = entries[0]
            itst = entry['itst_id']
            data = sorted(samples[itst].items())
            item['unique_fresh_samples'] = len(data)
            if not data:
                item['reason'] = '현재 API 신선한 관측 없음'
            else:
                runs = []
                for stamp, state in data:
                    if runs and runs[-1][2] == state and stamp - runs[-1][1] <= 3:
                        runs[-1][1] = stamp
                    else:
                        runs.append([stamp, stamp, state])
                try:
                    mapping = map_links(net.getTLS(tid), {'levels': {'current': runs}}, bearings.get(itst, {}))
                    cycle, anchor, phases = complete_cycle(runs, mapping)
                    check_conflicts(phases, mapping)
                    item['current_cycle'] = dict(
                        cycle_s=cycle,
                        anchor_kst=dt.datetime.fromtimestamp(anchor, ZoneInfo('Asia/Seoul')).isoformat(),
                        phases=[dict(duration_s=duration, state=state) for duration, state in phases],
                        link_mapping={i: dict(direction=entry[0], movement=entry[1], turn=entry[2])
                                      for i, entry in mapping.items()})
                    item['reason'] = '현재 시각의 관측 주기 후보; 차로 대응·운영계획 미검증'
                except ValueError as exc:
                    item['reason'] = str(exc)
        results.append(item)
    times = [t for series in samples.values() for t in series]
    local_times = [dt.datetime.fromtimestamp(t, ZoneInfo('Asia/Seoul')) for t in times]
    summary = dict(
        frames=frames, intersections_with_fresh_observations=sum(bool(s) for s in samples.values()),
        requested_intersections=len(catalog), modeled_controllers=len(positions),
        current_cycle_candidates=sum(r['current_cycle'] is not None for r in results),
        source_hours_kst=sorted({t.hour for t in local_times}),
        source_dates_kst=sorted({t.date().isoformat() for t in local_times}),
        first_observation_kst=min(local_times).isoformat() if local_times else None,
        last_observation_kst=max(local_times).isoformat() if local_times else None,
        fully_verified_controllers=0, programs_installed=0,
        all_real_world_signals_reproduced=False,
        reasons=dict(Counter(r['reason'] for r in results)))
    result = dict(endpoint=ENDPOINT, recording=str(path), summary=summary, controllers=results)
    metadata = path.with_suffix('.metadata.json')
    if metadata.exists():
        capture = json.loads(metadata.read_text())
        error = capture.get('error')
        quota = (error or {}).get('quota', {})
        if error and not quota:
            try:
                quota = json.loads(error['message'].split(': ', 1)[1]).get('headers', {})
            except (KeyError, ValueError, IndexError, AttributeError):
                pass
        result['collection_status'] = dict(
            planned_duration_s=capture.get('requested_duration_s'),
            actual_duration_s=capture.get('elapsed_s'),
            completed=capture.get('completed', not error and
                                  capture.get('elapsed_s', 0) >= capture.get('requested_duration_s', 0)),
            http_error=(error or {}).get('status'), quota=quota)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f'[current] 분석: {destination}')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=0, help='기록 시간; 0이면 한 번 조회')
    parser.add_argument('--interval', type=float, default=1, help='조회 간격(초), 최소 1초')
    parser.add_argument('--districts', nargs='+', default=DISTRICTS, help='조회할 시군구 코드')
    parser.add_argument('--max-requests', type=int, default=900, help='이번 실행의 최대 요청 수')
    parser.add_argument('--output-dir', type=Path, default=SOURCE / 'live')
    parser.add_argument('--analyze', type=Path, help='수집한 JSONL.GZ의 신선한 신호·주기 후보 분석')
    parser.add_argument('--report', type=Path, default=SOURCE.parent.parent / 'signal_audit' / 'current_service.json')
    args = parser.parse_args()
    if args.analyze:
        analyze(args.analyze, args.report)
    else:
        collect(args.seconds, args.interval, args.output_dir, args.districts, args.max_requests)


if __name__ == '__main__':
    main()
