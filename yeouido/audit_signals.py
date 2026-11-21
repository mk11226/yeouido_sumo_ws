"""Inventory every modeled signal without claiming observed cycles are TOD plans.

python -m yeouido.audit_signals
python -m yeouido.audit_signals --require-verified

The latter exits with code 2 until every controller has a verified real-world
plan. Candidate MAP proximity and SUMO foe checks are not field verification.
No signal program, route, or simulation setting is modified by this command.
"""
import argparse
import csv
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path

from yeouido.config import LEVELS, NET, OUT, sumo_home
from yeouido.tdata_signals import RECORDS, REPORT, SOURCE, signal_file

DEST = OUT / 'signal_audit'


def _movement_summary(program, connections):
    """Describe generated permissions, not whether a real intersection permits them."""
    by_index = defaultdict(list)
    for c in connections:
        by_index[c.getTLLinkIndex()].append(c)
    permissive = set()
    protected_foes = set()
    for phase_index, phase in enumerate(program.findall('phase')):
        state = phase.get('state')
        for index, group in by_index.items():
            if state[index] == 'g' and any(c.getDirection() in ('l', 'L') for c in group):
                permissive.add(index)
        indices = [i for i in by_index if state[i] == 'G']
        for a_idx, i in enumerate(indices):
            for j in indices[a_idx + 1:]:
                for a in by_index[i]:
                    for b in by_index[j]:
                        node = a.getJunction()
                        if node == b.getJunction() and node.areFoes(
                                a.getJunctionIndex(), b.getJunctionIndex()):
                            protected_foes.add((phase_index, i, j))
    return dict(program_id=program.get('programID'), type=program.get('type'),
                phase_count=len(program.findall('phase')),
                permissive_left_indices=sorted(permissive),
                protected_foe_pairs=[list(pair) for pair in sorted(protected_foes)])


def inventory():
    sumo_home()
    import sumolib
    net = sumolib.net.readNet(str(NET), withInternal=True)
    network = ET.parse(NET).getroot()
    base = {t.get('id'): t for t in network.findall('tlLogic') if t.get('programID') == '0'}
    observations = json.loads(RECORDS.read_text())
    catalog = json.loads((SOURCE / 'crossroads.json').read_text())
    report = json.loads(REPORT.read_text())
    skipped = defaultdict(list)
    for item in report['skipped']:
        skipped[item['tls_id']].append({k: item[k] for k in ('itst_id', 'level', 'reason') if k in item})
    overrides = {}
    for level in LEVELS:
        path = signal_file(level)
        overrides[level] = ({t.get('id'): t for t in ET.parse(path).getroot().findall('tlLogic')}
                            if path.exists() else {})
    tls_list = sorted(net.getTrafficLights(), key=lambda t: t.getID())
    connections_by_tls = defaultdict(list)
    for edge in net.getEdges():
        for group in edge.getOutgoing().values():
            for connection in group:
                if connection.getTLSID():
                    connections_by_tls[connection.getTLSID()].append(connection)
    positions = {t.getID(): {c[0].getEdge().getToNode().getCoord() for c in t.getConnections()}
                 for t in tls_list}
    candidates = defaultdict(list)
    for row in catalog['intersections']:
        xy = net.convertLonLat2XY(row['lon'], row['lat'])
        distance, tid = min((min(math.dist(xy, p) for p in points), tid)
                            for tid, points in positions.items())
        if distance <= 50:
            candidates[tid].append({**row, 'distance_m': round(distance, 2)})
    manual = net.getEdge('1180030100').getTLS().getID()
    controllers = []
    for tls in tls_list:
        tid = tls.getID()
        connections = connections_by_tls[tid]
        # Each listed controller remains unverified: the current inputs are
        # partial observations plus geometry, not complete, field-confirmed TODs.
        levels = {}
        for level in LEVELS:
            measured = overrides[level].get(tid)
            program = measured if measured is not None else base[tid]
            levels[level] = dict(
                source='observed_cycle_approximation' if measured is not None else
                       'user_confirmed_sequence' if tid == manual else 'sumo_generated',
                real_world_match='not_verified',
                **_movement_summary(program, connections))
        seen = {level for candidate in candidates[tid]
                for level in observations['intersections'].get(candidate['itst_id'], {}).get('levels', {})}
        point = tuple(sum(p[i] for p in positions[tid]) / len(positions[tid]) for i in (0, 1))
        lon, lat = net.convertXY2LonLat(*point)
        controllers.append(dict(
            tls_id=tid, lon=round(lon, 7), lat=round(lat, 7),
            incoming_edges=sorted({lane.getEdge().getID() for lane, _, _ in tls.getConnections()}),
            road_names=sorted({lane.getEdge().getName() for lane, _, _ in tls.getConnections()}),
            map_candidates=sorted(candidates[tid], key=lambda x: x['distance_m']),
            observed_levels=sorted(seen), levels=levels,
            previous_exclusions=skipped[tid],
            evidence_needed=['actual controller-to-lane mapping and protected/permissive turn rules',
                             'phase sequence and yellow/clearance intervals',
                             'time-of-day plans, cycle lengths and offsets',
                             'plan date and provenance; field/controller comparison']))
    count = len(controllers)
    summary = dict(
        modeled_controllers=count, fully_verified_controllers=0,
        controllers_with_nearby_map_candidate=sum(bool(c['map_candidates']) for c in controllers),
        controllers_without_map_candidate=sum(not c['map_candidates'] for c in controllers),
        controllers_with_multiple_map_candidates=sum(len(c['map_candidates']) > 1 for c in controllers),
        sources_by_level={level: dict(Counter(c['levels'][level]['source'] for c in controllers))
                          for level in LEVELS},
        controllers_with_permissive_left_by_level={level: sum(
            bool(c['levels'][level]['permissive_left_indices']) for c in controllers) for level in LEVELS},
        all_real_world_signals_reproduced=False)
    return dict(
        network_sha256=hashlib.sha256(NET.read_bytes()).hexdigest(),
        observation_dates=observations['dates'], map_source=catalog['source_url'],
        limits=['nearby MAP candidates are not verified controller matches',
                'permissive left is an inventory item, not proof of an error',
                'absence of SUMO foe conflicts is not proof of real-world equivalence',
                'current right-turn/U-turn rules contain modeling assumptions'],
        summary=summary, controllers=controllers)


def write(report, destination):
    destination.mkdir(parents=True, exist_ok=True)
    (destination / 'inventory.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    fields = ['tls_id', 'road_names', 'lon', 'lat', 'map_candidates', 'observed_levels',
              *[f'{lv}_source' for lv in LEVELS], 'real_world_match', 'evidence_needed']
    with (destination / 'inventory.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.DictWriter(f, fields, lineterminator='\n')
        writer.writeheader()
        for c in report['controllers']:
            writer.writerow(dict(
                tls_id=c['tls_id'], road_names=' / '.join(c['road_names']), lon=c['lon'], lat=c['lat'],
                map_candidates=' / '.join(f'{p["itst_id"]} {p["name"]} ({p["distance_m"]}m)'
                                          for p in c['map_candidates']),
                observed_levels=','.join(c['observed_levels']),
                **{f'{lv}_source': c['levels'][lv]['source'] for lv in LEVELS},
                real_world_match='not_verified', evidence_needed='; '.join(c['evidence_needed'])))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--require-verified', action='store_true',
                    help='전체 실제 운영계획 검증이 완료되지 않았으면 종료 코드 2')
    ap.add_argument('--output-dir', type=Path, default=DEST)
    args = ap.parse_args()
    report = inventory()
    write(report, args.output_dir)
    print(json.dumps(report['summary'], ensure_ascii=False, indent=2))
    print(f'목록: {args.output_dir / "inventory.csv"}')
    if args.require_verified and not report['summary']['all_real_world_signals_reproduced']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
