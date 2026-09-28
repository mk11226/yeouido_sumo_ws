"""Reallocate existing greens to straight-through movements near Gubanpo and SKKU stations."""
import copy
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from Road_Environment.paths import SCENARIO as OUT
# Approach edges on the Seoul -> Suwon route; retain every existing signal state.
APPROACHES = {
    '1252436244#4': '구반포역 서쪽 신반포로',
    '-1117538960': '구반포역 서쪽 동작대로 접속부',
    '1212097845#0': '구반포역 남쪽 동작대로',
    '775456751#6': '성균관대역 북쪽 서부로',
    '555231771#2': '성균관대역 앞 서부로 첫 직진',
    '488392556': '성균관대역 앞 서부로 다음 직진',
}


def build():
    from Road_Environment.build.build_skku_commute import write_xml, write_json, sha256
    root = ET.parse(OUT / 'skku_commute.net.xml').getroot()
    route = json.loads((OUT / 'route.json').read_text())['edge_ids']
    targets = {}
    for edge, label in APPROACHES.items():
        following = route[route.index(edge) + 1]
        connections = [c for c in root.findall('connection')
                       if c.get('from') == edge and c.get('to') == following
                       and c.get('dir') == 's' and c.get('tl')]
        if not connections:
            raise ValueError(f'No controlled straight movement: {edge}')
        for c in connections:
            targets.setdefault(c.get('tl'), {})[int(c.get('linkIndex'))] = label
    additional = ET.Element('additional')
    changes = []
    for tid, links in targets.items():
        original = root.find(f"tlLogic[@id='{tid}'][@programID='0']")
        if original is None or original.get('type') != 'static':
            raise ValueError(f'Expected baseline static program: {tid}')
        program = copy.deepcopy(original)
        program.set('programID', 'station_straight_priority')
        phases = program.findall('phase')
        prioritized = {i: sorted({label for link, label in links.items()
                                  if phase.get('state')[link] == 'G'})
                       for i, phase in enumerate(phases)}
        prioritized = {i: labels for i, labels in prioritized.items() if labels}
        greens = [i for i, phase in enumerate(phases) if 'G' in phase.get('state')
                  and 'y' not in phase.get('state')]
        if not prioritized or not set(prioritized) <= set(greens):
            raise ValueError(f'No protected straight green: {tid}')
        old_durations = [float(p.get('duration')) for p in phases]
        budget = sum(old_durations[i] for i in greens)
        # Calibrated against per-station stopped time, rather than maximizing throughput.
        if tid in ('4683581778', 'cluster_2551950297_921145139', '2551950286'):
            increment = 14
            program.set('offset', '10')
        elif tid == 'cluster_495951062_5357160054':
            increment = 15
        else:
            increment = 7
        desired = {i: old_durations[i] + increment for i in prioritized}
        others = [i for i in greens if i not in prioritized]
        remaining = budget - sum(desired.values())
        if not others or remaining < 5 * len(others):
            raise ValueError(f'Cannot preserve cycle with minimum 5s cross green: {tid}')
        other_budget = sum(old_durations[i] for i in others)
        edits = []
        for i in greens:
            duration = desired[i] if i in desired else remaining * old_durations[i] / other_budget
            if duration < 5:
                raise ValueError(f'Cross green too short: {tid}, {i}')
            phases[i].set('duration', f'{duration:g}')
            edits.append(dict(phase_index=i, approaches=prioritized.get(i, ['교차 방향']),
                              prioritized=i in prioritized, before_s=old_durations[i], after_s=duration))
        additional.append(program)
        changes.append(dict(tls_id=tid, phases=edits, offset_s=float(program.get('offset', '0')),
                            cycle_before_s=sum(float(p.get('duration')) for p in original.findall('phase')),
                            cycle_after_s=sum(float(p.get('duration')) for p in program.findall('phase'))))
    write_xml(OUT / 'station_signals.add.xml', additional)
    report = dict(synthetic=True, program_id='station_straight_priority',
                  network_sha256=sha256(OUT / 'skku_commute.net.xml'),
                  unchanged='signal states, total cycle, yellow and all-red durations', changes=changes)
    write_json(OUT / 'station_signals.manifest.json', report)
    return report


if __name__ == '__main__':
    print(json.dumps(build(), ensure_ascii=False, indent=2))
